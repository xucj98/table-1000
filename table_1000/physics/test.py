"""Execute native USD asset TESTS and measure simulation and video costs."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import time
import traceback


def load_tests(path):
    spec = importlib.util.spec_from_file_location('table1000_asset_tests',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.TESTS


def ground_clearance(model, rows, instance_names, ground_z):
    """Evaluate collision hull vertices against the plane from recorded poses."""
    import numpy as np
    from table_1000.physics.simulation import rotation
    worlds, owners, points = {}, {}, {}
    for node in model['nodes']:
        name,parent=node['name'],node['parent']
        worlds[name]=worlds.get(parent,np.eye(4)) @ np.array(node['matrix'])
        owners[name]=name if node['rigid'] else owners.get(parent)
        if node.get('role')=='collision':
            body=owners[name]
            transform=np.linalg.inv(worlds[body]) @ worlds[name]
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
        minimum.append(min(values))
    index=int(np.argmin(minimum))
    return {'minimum_collision_clearance_m':minimum[index],
            'maximum_geometry_penetration_m':max(0.,-minimum[index]),'time_s':rows[index]['time'],
            'sampling_interval_s':rows[1]['time']-rows[0]['time'],
            'method':'collision hull vertices transformed by recorded rigid poses; geometric overlap excludes contact margins'}


def benchmark(asset,name,test,app):
    from table_1000.physics.testing import TestContext
    rounds=[]
    for _ in range(3):
        seconds=wall=prepare=warmup=0.
        finals=[]
        while seconds < 10-1e-8:
            start=time.perf_counter()
            ctx=TestContext(asset,name=name,app=app,warmup=True)
            construction=time.perf_counter()-start
            test(ctx)
            seconds+=ctx.session.time;wall+=ctx.integrating_seconds
            prepare+=construction+ctx.preparation_seconds;warmup+=ctx.warmup_seconds
            finals.append(ctx.rows[-1])
            ctx.close()
        rounds.append({'simulated_seconds':seconds,'wall_seconds':wall,'RTF':seconds/wall,
                       'independent_repeats':len(finals),'preparation_reset_seconds':prepare,'warmup_seconds':warmup,
                       'final_states':finals})
    return {'rounds':rounds,'median_RTF':statistics.median(r['RTF'] for r in rounds),
            'includes':'physical steps, official behavior, test actions, state reading and in-memory sampling; no camera or rendering'}


def conditions(ctx):
    import carb
    import numpy as np
    from pxr import PhysxSchema,UsdPhysics
    from omni.physx.bindings import _physx
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.version import get_version
    from threadpoolctl import threadpool_info
    settings=carb.settings.get_settings()
    engine={}
    for instance in ctx.session.instances.values():
        shapes={}
        for name in instance.body_names:
            # A combined rigid view pads to the largest body's shape count.
            # One-body views report only that body's actual collision shapes.
            tensor=SimulationManager.get_physics_sim_view().create_rigid_body_view(instance.paths[name])
            shapes[name]={'count':tensor.max_shapes,
                'materials_static_dynamic_restitution':np.unique(np.asarray(tensor.get_material_properties()).reshape(-1,3),axis=0).tolist(),
                'contact_offsets_m':np.unique(np.asarray(tensor.get_contact_offsets())).tolist(),
                'rest_offsets_m':np.unique(np.asarray(tensor.get_rest_offsets())).tolist()}
        engine[instance.name]={'mass_kg':instance.mass.tolist(),'center_of_mass_m':instance.local_com.tolist(),
              'inertia_kg_m2':instance.local_inertia.tolist(),
              'collision_shapes':shapes,
              'rigid_bodies':{name:{'position_iterations':PhysxSchema.PhysxRigidBodyAPI(ctx.session.stage.GetPrimAtPath(instance.paths[name])).GetSolverPositionIterationCountAttr().Get(),
                                   'velocity_iterations':PhysxSchema.PhysxRigidBodyAPI(ctx.session.stage.GetPrimAtPath(instance.paths[name])).GetSolverVelocityIterationCountAttr().Get(),
                                   'ccd':PhysxSchema.PhysxRigidBodyAPI(ctx.session.stage.GetPrimAtPath(instance.paths[name])).GetEnableCCDAttr().Get()}
                              for name in instance.body_names}}
    cpu=next(line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name'))
    return {'engine':'Isaac Sim / CPU PhysX','isaac_version':get_version(),'dt':ctx.dt,'solver':'TGS','gpu_dynamics':False,'CCD':True,
            'instances':len(ctx.session.instances),'gravity':ctx.gravity,'ground':getattr(ctx,'ground_config',None),
            'physics_values_from_engine':engine,'cpu':cpu,
            'worker_controls':{'carb_tasking':settings.get('/plugins/carb.tasking.plugin/threadCount'),
                 'tbb':settings.get('/plugins/omni.tbb.globalcontrol/maxThreadCount'),
                 'physx':settings.get(_physx.SETTING_NUM_THREADS)},'native_math_threadpools':threadpool_info(),
            'render_settings':{path:settings.get(path) for path in ['/rtx/rendermode','/rtx/post/motionblur/enabled',
                '/rtx/directLighting/sampledLighting/enabled','/rtx/directLighting/sampledLighting/samplesPerPixel',
                '/rtx/directLighting/sampledLighting/denoisingTechnique','/rtx/directLighting/domeLight/sampleCount',
                '/rtx/directLighting/domeLight/denoisingTechnique','/rtx/shadows/sampleCount','/rtx/shadows/denoiser/enable',
                '/rtx/reflections/sampledLighting/samplesPerPixel','/rtx/reflections/denoiser/enabled','/rtx/post/taa/samples']},
            'camera':ctx.camera_config,'standard_dt':ctx.dt==.004}


def run_video(asset,name,test,output,app,startup,rtf,gpu):
    from table_1000.physics.testing import TestContext
    from table_1000.physics.plots import plots
    directory=output/name
    started=time.perf_counter()
    ctx=TestContext(asset,name=name,render=True,app=app,warmup=True)
    construction=time.perf_counter()-started
    try:
        test(ctx)
        frames=directory/'frames';frames.mkdir()
        for i,image in enumerate(ctx.images):image.save(frames/f'{i:05}.jpg',quality=95)
        columns=list(ctx.rows[0])
        with (directory/'trace.csv').open('w') as stream:
            writer=csv.DictWriter(stream,columns);writer.writeheader();writer.writerows(ctx.rows)
        fps=ctx.camera_config['fps']
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-framerate',str(fps),'-i',str(frames/'%05d.jpg'),
                        '-c:v','libx264','-pix_fmt','yuv420p','-crf','20',str(output/(name+'.mp4'))],check=True)
        wall=time.perf_counter()-ctx.video_started
        plots(ctx.rows,ctx.plot_columns,directory,ctx.units)
        # Fresh independent input executions, not subtraction of render costs.
        errors=[max(abs(float(row[key])-float(ctx.rows[-1][key])) for key in row.keys() & ctx.rows[-1].keys() if key!='time')
                for round in rtf['rounds'] for row in round['final_states']]
        result={'test':name,'execution_status':'completed','review_status':'pending','asset':str(asset),
            'conditions':{**conditions(ctx),'gpu':gpu},'columns':ctx.units,'plots':ctx.plot_columns,
            'steps':ctx.session.steps,'frames':len(ctx.images),'sampling':'state after integration at t; action columns are the force applied over [t-dt,t); initial t=0 has zero actions',
            'physics_callbacks':'actions/state priority -100; official BehaviorScript priority 0; batched force flush priority 100; one physical step, no hidden substeps',
            'performance':{'no_render':rtf,'video_end_to_end':{'simulated_seconds':ctx.session.time,'wall_seconds':wall,'RTF':ctx.session.time/wall},
                 'preparation':{'process_startup_seconds':startup,'construction_seconds':construction,'initialization_render_warmup_reset_seconds':ctx.preparation_seconds,'physical_warmup_seconds':ctx.warmup_seconds},
                 'independent_no_render_vs_video_final_max_scalar_error':max(errors)},
            'reaction_forces':'not sampled; recorded test forces are not contact or fixture reaction forces'}
        if getattr(ctx,'ground_config',None):
            result['ground_contact']=ground_clearance(json.loads((asset.parent/'model.json').read_text()),
                ctx.rows,ctx.session.instances,ctx.ground_config['z'])
        (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print('PHYSICS_TEST_COMPLETED',name,flush=True)
        return result
    finally:
        ctx.close()


def run_case(asset,name,test,output,app,startup,gpu):
    directory=output/name
    if directory.exists():shutil.rmtree(directory)
    (output/(name+'.mp4')).unlink(missing_ok=True)
    directory.mkdir(parents=True)
    try:
        rtf=benchmark(asset,name,test,app)
        return run_video(asset,name,test,output,app,startup,rtf,gpu)
    except Exception:
        (directory/'result.json').write_text(json.dumps({'test':name,'execution_status':'failed',
            'review_status':'pending','error':traceback.format_exc()},indent=2)+'\n')
        raise


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset',type=Path,help='asset directory or USD entry')
    parser.add_argument('--script',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--tests',nargs='+')
    parser.add_argument('--gpu',type=int,default=0)
    parser.add_argument('--plots-only',action='store_true')
    args=parser.parse_args(argv)
    asset=args.asset.resolve()
    if asset.is_dir():asset=asset/('object.usda' if (asset/'object.usda').exists() else 'object.usdz')
    tests=load_tests(args.script or asset.parent/'physics_test.py')
    if args.tests:tests={name:tests[name] for name in args.tests}
    output=(args.output or asset.parent/'physics_test').resolve();output.mkdir(parents=True,exist_ok=True)
    if args.plots_only:
        from table_1000.physics.plots import plot_trace
        for name in tests:
            result=json.loads((output/name/'result.json').read_text())
            plot_trace(output/name/'trace.csv',result['plots'],result['columns'])
        return
    started=time.perf_counter()
    gpu={'physical_index':args.gpu,'before_process_start':subprocess.run(
        ['nvidia-smi','--query-gpu=index,name,driver_version,memory.used,utilization.gpu','--format=csv'],
        check=True,capture_output=True,text=True).stdout.strip()}
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'create_new_stage':False,'active_gpu':args.gpu,'physics_gpu':args.gpu,'multi_gpu':False,
         'disable_viewport_updates':True,'limit_cpu_threads':4,'renderer':'RaytracedLighting','width':960,'height':480,
         'extra_args':['--/persistent/physics/numThreads=4','--/rtx/post/motionblur/enabled=False']})
    startup=time.perf_counter()-started
    try:
        import carb
        from threadpoolctl import threadpool_limits
        from table_1000.physics.simulation import enable_scripting
        enable_scripting()
        carb.settings.get_settings().set('/persistent/physics/numThreads',4)
        carb.settings.get_settings().set('/rtx/post/motionblur/enabled',False)
        with threadpool_limits(limits=1):
            for name,test in tests.items():
                run_case(asset,name,test,output,app,startup,gpu)
    except Exception:
        traceback.print_exc()
        raise
    finally:
        app.close()
