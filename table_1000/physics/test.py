"""Run asset physics TESTS on one or more backends and summarize each asset."""
import argparse
import csv
import datetime
import faulthandler
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
import traceback

from table_1000.common.batch import add_arguments, job_count, process_name, run_batch, worker_gpu


def load_tests(path):
    spec = importlib.util.spec_from_file_location('table1000_asset_tests',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.TESTS


def stage_event(stage, **details):
    """Emit a timestamped breadcrumb for locating stalls in Isaac workers."""
    event={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
           'monotonic_s':time.monotonic(),'pid':os.getpid(),
           'thread':threading.current_thread().name,'stage':stage,**details}
    print('PHYSICS_TEST_STAGE '+json.dumps(event,ensure_ascii=False,default=repr),flush=True)


def run_selected_cases(selected, run):
    """Run each selected case even after a prior case fails; return failure state."""
    failed=False
    for name,test in selected.items():
        stage_event('case.begin',test=name)
        try:
            run(name,test)
            stage_event('case.return',test=name,status='completed')
        except Exception:
            failed=True
            stage_event('case.return',test=name,status='failed')
            traceback.print_exc()
    return failed


def ground_clearance(model, rows, instance_names, ground_z):
    """Evaluate collision hull vertices against the plane from recorded poses."""
    import numpy as np
    from table_1000.physics.simulation import rotation
    worlds, owners, points, spheres = {}, {}, {}, {}
    for node in model['nodes']:
        name,parent=node['name'],node['parent']
        worlds[name]=worlds.get(parent,np.eye(4)) @ np.array(node['matrix'])
        owners[name]=name if node['rigid'] else owners.get(parent)
        if node.get('role')=='collision':
            body=owners[name]
            transform=np.linalg.inv(worlds[body]) @ worlds[name]
            if node.get('shape')=='SPHERE':
                spheres.setdefault(body,[]).append((transform[:3,3],node['radius']))
            else:
                vertices=np.c_[node['vertices'],np.ones(len(node['vertices']))]
                points.setdefault(body,[]).extend((vertices @ transform.T)[:,:3])
    points={body:np.array(vertices) for body,vertices in points.items()}
    minimum=[]
    for row in rows:
        values=[]
        for instance in instance_names:
            for body,vertices in points.items():
                prefix=f'{instance}.{body}.'
                quaternion=[row[prefix+'quaternion.'+axis] for axis in 'wxyz']
                values.append(float((vertices @ rotation(quaternion).T)[:,2].min()+row[prefix+'position.z']-ground_z))
            for body,entries in spheres.items():
                prefix=f'{instance}.{body}.'
                quaternion=[row[prefix+'quaternion.'+axis] for axis in 'wxyz']
                r=rotation(quaternion)
                values.extend(float((r@center)[2]+row[prefix+'position.z']-radius-ground_z) for center,radius in entries)
        minimum.append(min(values))
    index=int(np.argmin(minimum))
    return {'minimum_collision_clearance_m':minimum[index],
            'maximum_geometry_penetration_m':max(0.,-minimum[index]),'time_s':rows[index]['time'],
            'sampling_interval_s':rows[1]['time']-rows[0]['time'],
            'method':'collision hull vertices and analytic sphere extents transformed by recorded rigid poses; geometric overlap excludes contact margins'}


def retain_failure(ctx, asset, name, directory, error, *, physics, num_envs, phase, gpu=None):
    """Save evidence already produced; never advance physics or claim acceptance."""
    directory.mkdir(parents=True, exist_ok=True)
    diagnostics = directory / 'diagnostics'
    diagnostics.mkdir(exist_ok=True)
    result = {'test':name, 'asset':str(asset), 'execution_status':'failed',
              'failed_phase':phase, 'error':error,
              'environments':{'count':num_envs},
              'diagnostics':{'acceptance_eligible':False,
                             'scope':'available samples before failure; may be incomplete',
                             'files':{}, 'capture_errors':{}}}
    captured = result['diagnostics']

    def attempt(key, action):
        try:
            return action()
        except Exception:
            captured['capture_errors'][key] = traceback.format_exc()
            return None

    def field(key, value):
        # One unsupported measurement must not discard other evidence.
        json.dumps(value)
        result[key] = value

    for key, attribute in [('columns','units'), ('plots','plot_columns'),
                           ('metrics','metrics'), ('experiment','experiment_details')]:
        attempt(key, lambda key=key, attribute=attribute: field(key, getattr(ctx,attribute,{})))
    attempt('conditions', lambda: field('conditions', {**conditions(ctx), 'gpu':gpu}))
    if 'conditions' not in result:
        result['conditions'] = {'physics_backend':physics, 'dt':getattr(ctx,'dt',None),
                                'gpu':gpu, 'complete':False,
                                'note':'full engine readback unavailable; see diagnostics.capture_errors.conditions'}

    rows = getattr(ctx,'rows',[])
    result['steps'] = getattr(getattr(ctx,'session',None),'steps',None)
    captured['sample_count'] = len(rows)
    if rows:
        def save_trace():
            columns = list(dict.fromkeys(key for row in rows for key in row))
            with (diagnostics/'trace.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,columns)
                writer.writeheader()
                writer.writerows(rows)
            captured['files']['trace'] = 'diagnostics/trace.csv'
        attempt('trace',save_trace)
    images = getattr(ctx,'images',[])
    captured['frame_count_available'] = len(images)
    captured['frame_count_saved'] = 0
    if images:
        frames=diagnostics/'frames'
        attempt('frames_directory',lambda: frames.mkdir(exist_ok=True))
        for i,image in enumerate(images):
            def save_frame(i=i,image=image):
                image.save(frames/f'{i:05}.jpg',quality=95)
                captured['frame_count_saved'] += 1
            attempt(f'frame_{i}',save_frame)
        if captured['frame_count_saved']:
            captured['files']['frames'] = 'diagnostics/frames/'
    (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')


def capture_failure(ctx, asset, name, directory, *, physics, num_envs, phase, gpu=None):
    """Best-effort diagnostics must not replace the active experiment exception."""
    error=traceback.format_exc()
    try:
        retain_failure(ctx,asset,name,directory,error,physics=physics,num_envs=num_envs,phase=phase,gpu=gpu)
    except Exception:
        print('PHYSICS_TEST_DIAGNOSTICS_FAILED',name,traceback.format_exc(),file=sys.stderr,flush=True)


def close_context(ctx):
    """Keep an active experiment error authoritative if cleanup also fails."""
    if sys.exc_info()[0] is None:
        ctx.close()
    else:
        try:
            ctx.close()
        except Exception:
            print('PHYSICS_TEST_CLEANUP_FAILED',traceback.format_exc(),file=sys.stderr,flush=True)


def benchmark(asset,name,test,app,physics="cpu",num_envs=1,utilization=None,diagnostics_directory=None):
    from table_1000.physics.testing import TestContext
    # Complete short experiments in fresh scenes until the warmup totals 1 s.
    # Restoring poses alone cannot restore the contact solver's internal state;
    # neither warmup nor measured repeats append an idle tail to the experiment.
    warmup = warm_seconds = 0.
    warm_repeats = 0
    while warm_seconds < 1-1e-8:
        warm = TestContext(asset, name=name, app=app, physics=physics, num_envs=num_envs)
        start = time.perf_counter()
        try:
            test(warm)
            duration = warm.session.time
            assert math.isfinite(duration) and duration > 0, 'RTF warmup requires a finite positive experiment duration'
            warm_seconds += duration
            warmup += time.perf_counter()-start
            warm_repeats += 1
        except Exception:
            if diagnostics_directory is not None:
                capture_failure(warm,asset,name,diagnostics_directory,physics=physics,num_envs=num_envs,phase='benchmark_warmup')
            raise
        finally:
            close_context(warm)
    rounds=[]
    for _ in range(3):
        seconds=wall=prepare=0.
        finals=[]
        while seconds < 10-1e-8:
            start=time.perf_counter()
            ctx=TestContext(asset,name=name,app=app,physics=physics,num_envs=num_envs)
            ctx.utilization=utilization
            construction=time.perf_counter()-start
            try:
                test(ctx)
                seconds+=ctx.session.time;wall+=time.perf_counter()-ctx.experiment_started
                if utilization: utilization.stop()
                prepare+=construction+ctx.preparation_seconds
                finals.append(ctx.rows[-1])
            except Exception:
                if diagnostics_directory is not None:
                    capture_failure(ctx,asset,name,diagnostics_directory,physics=physics,num_envs=num_envs,phase='benchmark_round')
                raise
            finally:
                close_context(ctx)
        rounds.append({'simulated_seconds':seconds,'wall_seconds':wall,'RTF':seconds/wall,
                       'independent_repeats':len(finals),'preparation_reset_seconds':prepare,
                       'final_states':finals})
    median=statistics.median(r['RTF'] for r in rounds)
    return {'warmup':{'wall_seconds':warmup,'simulated_seconds':warm_seconds,'independent_repeats':warm_repeats,
                      'method':'complete independent experiments totaling at least one simulated second in disposable scenes before measured rounds'},
            'rounds':rounds,'median_RTF':median,'environments':num_envs,'median_environment_seconds_per_second':median*num_envs,
            'utilization':utilization.summary() if utilization else None,
            'includes':'physical steps, official behavior, test actions, state reading and in-memory sampling; no camera or rendering'}


def conditions(ctx):
    import carb
    import numpy as np
    from pxr import PhysxSchema,UsdPhysics
    from omni.physx.bindings import _physx
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.version import get_version
    from threadpoolctl import threadpool_info
    from table_1000.sim.isaaclab.fixtures import LINK_MASS, LINK_INERTIA
    settings=carb.settings.get_settings()
    engine={}
    for instance in ctx.session.instances.values():
        shapes={}
        for name in instance.body_names:
            # A combined rigid view pads to the largest body's shape count.
            # One-body views report only that body's actual collision shapes.
            tensor=SimulationManager.get_physics_sim_view().create_rigid_body_view(instance.paths[name])
            shapes[name]={'count':tensor.max_shapes,
                'materials_static_dynamic_restitution':np.unique(np.asarray(tensor.get_material_properties().cpu()).reshape(-1,3),axis=0).tolist(),
                'contact_offsets_m':np.unique(np.asarray(tensor.get_contact_offsets().cpu())).tolist(),
                'rest_offsets_m':np.unique(np.asarray(tensor.get_rest_offsets().cpu())).tolist()}
        engine[instance.name]={'mass_kg':instance.mass.tolist(),'center_of_mass_m':instance.local_com.tolist(),
              'inertia_kg_m2':instance.local_inertia.tolist(),
              'collision_shapes':shapes,
              'rigid_bodies':{name:{'position_iterations':PhysxSchema.PhysxRigidBodyAPI(ctx.session.stage.GetPrimAtPath(instance.paths[name])).GetSolverPositionIterationCountAttr().Get(),
                                   'velocity_iterations':PhysxSchema.PhysxRigidBodyAPI(ctx.session.stage.GetPrimAtPath(instance.paths[name])).GetSolverVelocityIterationCountAttr().Get(),
                                   'ccd':PhysxSchema.PhysxRigidBodyAPI(ctx.session.stage.GetPrimAtPath(instance.paths[name])).GetEnableCCDAttr().Get()}
                              for name in instance.body_names}}
        if instance.articulation is not None:
            engine[instance.name]['articulation_dof_parameters'] = {
                'friction_coefficients':instance.articulation.get_dof_friction_coefficients().tolist(),
                'friction_properties_static_dynamic_viscous':instance.articulation.get_dof_friction_properties().tolist(),
                'stiffnesses':instance.articulation.get_dof_stiffnesses().tolist(),
                'dampings':instance.articulation.get_dof_dampings().tolist()}
    cpu=next(line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name'))
    scene=next(p for p in ctx.session.stage.Traverse() if p.IsA(UsdPhysics.Scene))
    scene_api=PhysxSchema.PhysxSceneAPI(scene)
    scene_ccd=scene_api.GetEnableCCDAttr().Get()
    return {'engine':'Isaac Lab / Isaac Sim / '+ctx.physics.upper()+' PhysX','isaac_version':get_version(),'dt':ctx.dt,'solver':scene_api.GetSolverTypeAttr().Get(),'gpu_dynamics':scene_api.GetEnableGPUDynamicsAttr().Get(),'device':ctx.world.device,'scene_CCD':scene_ccd,
            'solver_configuration':ctx.world.cfg.physx.to_dict(),
            'fixtures':[({'body':f.body.path,'kind':'fixed articulation root','locked':True}
                         if not hasattr(f, 'stiffness') else {'body':f.body.path,'kind':'six-axis tensor articulation','locked':f.locked,
                         'virtual_link_mass_kg':LINK_MASS,'virtual_link_inertia_kg_m2':LINK_INERTIA,
                         'virtual_links':6,'stiffness':f.stiffness.tolist(),'damping':f.damping.tolist()})
                        for f in ctx.fixtures],
            'instances':len(ctx.session.instances),'gravity':ctx.gravity,'ground':getattr(ctx,'ground_config',None),
            'physics_values_from_engine':engine,'cpu':cpu,
            'load_average':list(os.getloadavg()),'environment_threads':{key:os.getenv(key) for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']},
            'worker_controls':{'carb_tasking':settings.get('/plugins/carb.tasking.plugin/threadCount'),
                 'tbb':settings.get('/plugins/omni.tbb.globalcontrol/maxThreadCount'),
                 'physx':settings.get(_physx.SETTING_NUM_THREADS)},'native_math_threadpools':threadpool_info(),
            'render_settings':{path:settings.get(path) for path in ['/rtx/rendermode','/persistent/rtx/modes/rt2/enabled',
              '/rtx/rtpt/maxBounces','/rtx/rtpt/maxSpecularAndTransmissionBounces','/rtx/post/motionblur/enabled',
                '/rtx/directLighting/sampledLighting/enabled','/rtx/directLighting/sampledLighting/samplesPerPixel',
                '/rtx/directLighting/sampledLighting/denoisingTechnique','/rtx/directLighting/domeLight/sampleCount',
                '/rtx/directLighting/domeLight/denoisingTechnique','/rtx/shadows/sampleCount','/rtx/shadows/denoiser/enable',
                '/rtx/reflections/sampledLighting/samplesPerPixel','/rtx/reflections/denoiser/enabled','/rtx/post/taa/samples']},
            'camera':ctx.camera_config,
            'render_passes_per_capture':2 if ctx.camera_instance else 0,
            'capture_render_pass_counts':ctx.camera_instance.capture_render_counts if ctx.camera_instance else [],
            'lighting':{'dome_intensity':ctx.session.stage.GetPrimAtPath('/World/Light').GetAttribute('inputs:intensity').Get(),
                        'sun_intensity':ctx.session.stage.GetPrimAtPath('/World/Sun').GetAttribute('inputs:intensity').Get(),
                        'sun_rotation_xyz_degrees':list(ctx.session.stage.GetPrimAtPath('/World/Sun').GetAttribute('xformOp:rotateXYZ').Get())}
                       if ctx.camera_instance else None,
            'standard_dt':ctx.dt==.004}


def run_video(asset,name,test,output,app,startup,rtf,gpu,physics="cpu",num_envs=1,utilization=None):
    from table_1000.physics.testing import TestContext
    from table_1000.physics.plots import plots
    directory=output/name
    started=time.perf_counter()
    stage_event('video.context.construct.begin',test=name,backend=physics)
    ctx=TestContext(asset,name=name,render=True,app=app,physics=physics,num_envs=num_envs)
    ctx.utilization=utilization
    construction=time.perf_counter()-started
    stage_event('video.context.construct.return',test=name,backend=physics,seconds=construction)
    try:
        stage_event('video.experiment.begin',test=name)
        test(ctx)
        stage_event('video.experiment.return',test=name,status='completed',steps=ctx.session.steps)
        replica_deviation=max(instance.view.replica_deviation() for instance in ctx.session.instances.values())
        frames=directory/'frames';frames.mkdir()
        stage_event('video.media.write.begin',test=name,frames=len(ctx.images),samples=len(ctx.rows))
        for i,image in enumerate(ctx.images):image.save(frames/f'{i:05}.jpg',quality=95)
        columns=list(ctx.rows[0])
        with (directory/'trace.csv').open('w') as stream:
            writer=csv.DictWriter(stream,columns);writer.writeheader();writer.writerows(ctx.rows)
        fps=ctx.camera_config['fps']
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-framerate',str(fps),'-i',str(frames/'%05d.jpg'),
                        '-c:v','libx264','-pix_fmt','yuv420p','-crf','20',str(output/(name+'.mp4'))],check=True)
        wall=time.perf_counter()-ctx.video_started
        stage_event('video.media.write.return',test=name,seconds=wall)
        stage_event('video.utilization.summary.begin',test=name)
        video_utilization=utilization.summary() if utilization else None
        stage_event('video.utilization.summary.return',test=name,
                    status=video_utilization.get('status') if video_utilization else 'not_configured')
        plots(ctx.rows,ctx.plot_columns,directory,ctx.units)
        stage_event('video.plots.return',test=name)
        # Fresh independent input executions, not subtraction of render costs.
        errors=[max(abs(float(row[key])-float(ctx.rows[-1][key])) for key in row.keys() & ctx.rows[-1].keys() if key!='time')
                for round in rtf['rounds'] for row in round['final_states']] if rtf else []
        if rtf is None:
            # One pass: physics steps and sampling of this run, camera capture excluded.
            rtf={'method':'physics steps and sampling within the video run, camera capture excluded; use --benchmark for the standard measurement',
                 'simulated_seconds':ctx.session.time,'wall_seconds':ctx.integrating_seconds,
                 'RTF':ctx.session.time/ctx.integrating_seconds,'environments':num_envs,
                 'environment_seconds_per_second':num_envs*ctx.session.time/ctx.integrating_seconds}
        result={'test':name,'execution_status':'completed','asset':str(asset),
            'conditions':{**conditions(ctx),'gpu':gpu},
            'environments':{'count':num_envs,'recorded_and_rendered':'environment 0; replicas receive identical inputs and are hidden',
                'spacing_m':float(ctx.session.offsets[1][1]) if num_envs>1 else None,
                'final_replica_max_position_deviation_m':replica_deviation},'columns':ctx.units,'plots':ctx.plot_columns,
            'steps':ctx.session.steps,'frames':len(ctx.images),'sampling':'state after integration at t; action columns are the force applied over [t-dt,t); initial t=0 has zero actions',
            'physics_callbacks':'actions/state priority -100; official BehaviorScript priority 0 when present; batched force flush priority 100; one external physical step; actual solver and iterations in conditions',
            'performance':{'no_render':rtf,'video_end_to_end':{'simulated_seconds':ctx.session.time,'wall_seconds':wall,'RTF':ctx.session.time/wall,
                     'environments':num_envs,'environment_seconds_per_second':num_envs*ctx.session.time/wall,
                     'utilization':video_utilization},
                 'preparation':{'process_startup_seconds':startup,'construction_seconds':construction,'initialization_render_warmup_reset_seconds':ctx.preparation_seconds},
                 'independent_no_render_vs_video_final_max_scalar_error':max(errors) if errors else None},
            'reaction_forces':'contact reaction forces not sampled; grip columns record the force each grip applies'}
        result['metrics'] = ctx.metrics
        if getattr(ctx, 'experiment_details', None):
            result['experiment'] = ctx.experiment_details
        if ctx.grips:
            result['grips'] = ctx.grip_details()
        if getattr(ctx,'ground_config',None):
            result['ground_contact']=ground_clearance(json.loads((asset.parent/'model.json').read_text()),
                ctx.rows,ctx.session.instances,ctx.ground_config['z'])
        (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        stage_event('video.result.write.return',test=name)
        print('PHYSICS_TEST_COMPLETED',name,flush=True)
        return result
    except Exception:
        stage_event('video.exception.capture.begin',test=name)
        capture_failure(ctx,asset,name,directory,physics=physics,num_envs=num_envs,phase='video',gpu=gpu)
        stage_event('video.exception.capture.return',test=name)
        raise
    finally:
        stage_event('video.context.close.begin',test=name)
        close_context(ctx)
        stage_event('video.context.close.return',test=name)


def clear(output,name):
    """Remove one test's previous results, so a rerun never reports them."""
    if (output/name).exists():shutil.rmtree(output/name)
    (output/(name+'.mp4')).unlink(missing_ok=True)


def run_case(asset,name,test,output,app,startup,gpu,physics="cpu",num_envs=1,utilization=None,standard=False):
    directory=output/name
    clear(output,name)
    directory.mkdir(parents=True)
    rtf = None
    phase = 'benchmark' if standard else 'video'
    try:
        stage_event('run_case.begin',test=name,backend=physics,standard=standard)
        stage_event('run_case.benchmark.begin',test=name)
        rtf=benchmark(asset,name,test,app,physics=physics,num_envs=num_envs,utilization=utilization,diagnostics_directory=directory) if standard else None
        stage_event('run_case.benchmark.return',test=name,status='completed' if standard else 'skipped')
        phase = 'video'
        stage_event('run_case.video.begin',test=name)
        result=run_video(asset,name,test,output,app,startup,rtf,gpu,physics=physics,num_envs=num_envs,utilization=utilization)
        stage_event('run_case.video.return',test=name,status=result.get('execution_status','completed'))
        return result
    except Exception:
        error=traceback.format_exc()
        stage_event('run_case.exception',test=name,phase=phase)
        failure={}
        # clear() removed prior results; only merge this run's diagnostics.
        if (directory/'result.json').exists():
            try:
                saved=json.loads((directory/'result.json').read_text())
                if saved.get('execution_status')=='failed':failure=saved
            except (ValueError,OSError):
                pass
        failure.update(test=name,execution_status='failed',error=error)
        failure.setdefault('failed_phase',phase)
        if rtf is not None:
            failure['performance'] = {'no_render':rtf}
            failure['benchmark_status'] = 'completed'
        (directory/'result.json').write_text(json.dumps(failure,indent=2)+'\n')
        stage_event('run_case.failure_result.write.return',test=name,phase=failure['failed_phase'])
        raise


# Backend name -> PhysX device. Each backend writes physics_test/<name>/.
BACKENDS={'isaac-cpu':'cpu','isaac-gpu':'gpu'}
ROOT=Path(__file__).resolve().parents[2]
ENTRY=ROOT/'scripts/assets/physics_test.py'


def significant(value,digits=4):
    return float(f'{value:.{digits}g}')


def summary(directory,names,error=None):
    """Collect each backend's result.json into one human-facing digest."""
    asset=str(directory.relative_to(directory.parents[1]))
    if error:
        return {'asset':asset,'error':error,'backends':{},'tests':{}}
    backends,tests={},{}
    for backend in BACKENDS:
        output=directory/'physics_test'/backend
        for name in names:
            path=output/name/'result.json'
            if not path.exists():continue
            result=json.loads(path.read_text())
            test=tests.setdefault(name,{'status':{},'metrics':{},'rtf':{}})
            status=test['status'][backend]=result['execution_status']
            entry=backends.setdefault(backend,{'completed':0,'failed':[]})
            if status!='completed':
                entry['failed'].append(name)
                test.setdefault('error',{})[backend]=result['error'].strip().splitlines()[-1]
                continue
            entry['completed']+=1
            entry.setdefault('engine',f"Isaac Sim {result['conditions']['isaac_version'][0]} {BACKENDS[backend].upper()} PhysX")
            entry.setdefault('environments',result['environments']['count'])
            for key,metric in result.get('metrics',{}).items():
                test['metrics'].setdefault(key,{'label':metric['label'],'unit':metric['unit']})[backend]=significant(metric['value'])
            performance=result['performance']
            physics=performance['no_render']
            test['rtf'][backend]={'physics':significant(physics.get('median_RTF',physics.get('RTF')),3),
                                  'video':significant(performance['video_end_to_end']['RTF'],3)}
    ordered={name:tests[name] for name in names if name in tests}
    return {'asset':asset,'backends':backends,'tests':ordered}


def write_summary(directory,names,error=None):
    path=directory/'physics_test'/'summary.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(summary(directory,names,error),indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    return path


def entry(directory):
    return directory/('object.usda' if (directory/'object.usda').exists() else 'object.usdz')


def run_backend(asset,backend,*,gpu,script,tests=None,num_envs=None,benchmark=False):
    """Run one asset on one backend in this process; Kit's close ends the process."""
    physics=BACKENDS[backend]
    num_envs=num_envs or (4 if physics=='gpu' else 1)
    output=asset.parent/'physics_test'/backend
    output.mkdir(parents=True,exist_ok=True)
    stage_event('backend.begin',asset=str(asset),backend=backend,gpu=gpu,tests=tests)
    started=time.perf_counter()
    gpu_info={'physical_index':gpu,'before_process_start':subprocess.run(
        ['nvidia-smi','--query-gpu=index,name,driver_version,memory.used,utilization.gpu','--format=csv'],
        check=True,capture_output=True,text=True).stdout.strip()}
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    app=start_app(gpu=gpu, device='cpu' if physics=='cpu' else 'cuda:0',
                  extra_args=(*RT2_ARGS,'--/rtx/post/motionblur/enabled=False'))
    startup=time.perf_counter()-started
    stage_event('backend.app.ready',asset=str(asset),backend=backend,startup_s=startup)
    if hasattr(signal,'SIGUSR1'):
        try:
            faulthandler.register(signal.SIGUSR1,file=sys.stderr,all_threads=True,chain=False)
            stage_event('backend.thread_dump.registered',signal='SIGUSR1')
        except (RuntimeError,ValueError,OSError) as error:
            stage_event('backend.thread_dump.unavailable',error=f'{type(error).__name__}: {error}')
    failed=False
    try:
        selected=load_tests(script)
        if tests:selected={name:selected[name] for name in tests}
        import carb
        from threadpoolctl import threadpool_limits
        from table_1000.physics.simulation import enable_scripting
        enable_scripting()
        # Complete extension startup before replacing the launcher stage.
        for _ in range(5):
            app.update()
        carb.settings.get_settings().set('/persistent/physics/numThreads',4)
        carb.settings.get_settings().set('/rtx/post/motionblur/enabled',False)
        # OpenMP is set before startup; changing it after Kit starts can stall RTX.
        from table_1000.physics.utilization import Utilization
        utilization=Utilization(gpu)
        try:
            with threadpool_limits(limits=1, user_api='blas'):
                def run(name,test):
                    return run_case(asset,name,test,output,app,startup,gpu_info,physics=physics,num_envs=num_envs,
                                    utilization=utilization,standard=benchmark)
                failed=run_selected_cases(selected,run)
        finally:
            stage_event('backend.utilization.close.begin',asset=str(asset),backend=backend)
            utilization.close()
            stage_event('backend.utilization.close.return',asset=str(asset),backend=backend,
                        status=getattr(utilization,'close_status',None))
    except BaseException:
        failed=True
        traceback.print_exc()
        raise
    finally:
        stage_event('backend.app.close.begin',asset=str(asset),backend=backend,failed=failed)
        app.close(exit_code=int(failed))
        stage_event('backend.app.close.return',asset=str(asset),backend=backend,failed=failed)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=Path('assets/objects'),help='built assets root')
    parser.add_argument('--assets',nargs='+',help='asset directories relative to --source; default all with physics_test.py')
    parser.add_argument('--physics',nargs='+',choices=tuple(BACKENDS),required=True,help='physics backends to run')
    parser.add_argument('--tests',nargs='+',help='only these tests')
    parser.add_argument('--scripts',type=Path,help='read physics_test.py from this root (e.g. asset_sources/objects) instead of the build snapshot')
    parser.add_argument('--num-envs',type=int,help='parallel copies of each experiment; default 1 on CPU, 4 on GPU')
    parser.add_argument('--benchmark',action='store_true',help='standard RTF measurement: warmup and three timed rounds before the video run')
    parser.add_argument('--plots-only',action='store_true')
    add_arguments(parser)
    parser.add_argument('--worker',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--script',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args(argv)
    if args.num_envs is not None and args.num_envs<1:parser.error('--num-envs must be positive')
    source=args.source.resolve()
    if args.worker:
        run_backend(entry(args.worker),args.physics[0],gpu=worker_gpu(args.gpu),script=args.script,tests=args.tests,
                    num_envs=args.num_envs,benchmark=args.benchmark)
        return
    references=([Path(reference) for reference in args.assets] if args.assets is not None
                else sorted(path.parent.relative_to(source) for path in source.rglob('physics_test.py')))
    scripts={reference:(args.scripts.resolve()/reference if args.scripts else source/reference)/'physics_test.py'
             for reference in references}
    if not args.scripts and not args.plots_only:
        for reference,script in scripts.items():
            edited=ROOT/'asset_sources'/'objects'/reference/'physics_test.py'
            if edited.is_file() and script.is_file() and edited.read_bytes()!=script.read_bytes():
                print(f'WARNING {reference}: {script} differs from {edited}; rebuild with build_physics.py '
                      'or test the edited one with --scripts asset_sources/objects',flush=True)
    names,broken={},{}
    for reference,script in scripts.items():
        try:
            names[reference]=list(load_tests(script))
        except Exception:
            # A broken definition fails only its own asset.
            traceback.print_exc()
            broken[reference]=traceback.format_exc().strip().splitlines()[-1]
            continue
        unknown=set(args.tests or ())-set(names[reference])
        if unknown:parser.error(f'{reference}: unknown tests {sorted(unknown)}')
    if args.plots_only:
        from table_1000.physics.plots import plot_trace
        for reference in names:
            for backend in args.physics:
                output=source/reference/'physics_test'/backend
                for name in args.tests or names[reference]:
                    result=json.loads((output/name/'result.json').read_text())
                    plot_trace(output/name/'trace.csv',result['plots'],result['columns'])
        if broken:
            raise SystemExit('failed: '+', '.join(f'{reference} (physics_test.py)' for reference in broken))
        return
    count=job_count(parser,args)
    selected={}
    for reference in names:
        for backend in args.physics:
            selected[reference,backend]=args.tests or names[reference]
            for name in selected[reference,backend]:clear(source/reference/'physics_test'/backend,name)
            for log in (source/reference/'physics_test'/backend).glob('run-retry*.log'):log.unlink()

    def results(unit):
        output=source/unit[0]/'physics_test'/unit[1]
        return {name:output/name/'result.json' for name in selected[unit]}

    def command(group,index,attempt):
        (reference,backend),=group
        command=[sys.executable,str(ENTRY),'--worker',str(source/reference),'--physics',backend,
                 '--script',str(scripts[reference])]
        # A rerun runs only the tests its stopped process left without a result.
        remaining=[name for name,path in results(group[0]).items() if not path.exists()]
        if remaining:command+=['--tests',*remaining]
        if args.num_envs:command+=['--num-envs',str(args.num_envs)]
        if args.benchmark:command.append('--benchmark')
        log=source/reference/'physics_test'/backend/f"{process_name('run',attempt)}.log"
        return command,log,f'{reference} {backend}'

    # One process per asset and backend: Kit's close ends it.
    run_batch(list(selected),key=lambda unit:unit,command=command,
              missing=lambda unit:not all(path.exists() for path in results(unit).values()),
              count=count,gpus=args.gpus,launcher=args.launcher,retries=args.retries)
    failed=[]
    for (reference,backend),tests in selected.items():
        output=source/reference/'physics_test'/backend
        for name in tests:
            # The worker exited before reaching this test, e.g. at startup, in every attempt.
            if not (output/name/'result.json').exists():
                (output/name).mkdir(parents=True,exist_ok=True)
                (output/name/'result.json').write_text(json.dumps({'test':name,'execution_status':'failed',
                    'error':f'not run: the {backend} process exited before this test; see {output}/run*.log'},indent=2)+'\n')
        if any(json.loads(path.read_text())['execution_status']!='completed' for path in results((reference,backend)).values()):
            failed.append(f'{reference} {backend}')
    for reference in references:
        print('SUMMARY',write_summary(source/reference,names.get(reference,[]),broken.get(reference)),flush=True)
    failed+=[f'{reference} (physics_test.py)' for reference in broken]
    if failed:
        raise SystemExit('failed: '+', '.join(failed))


if __name__ == '__main__':
    main()
