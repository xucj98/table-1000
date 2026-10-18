"""Execute native USD asset TESTS and measure simulation and video costs."""
import argparse
import csv
import importlib.util
import json
import os
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


def benchmark(asset,name,test,app,physics="cpu"):
    from table_1000.physics.testing import TestContext
    # Warm a disposable scene: restoring poses alone cannot restore the
    # contact solver's internal state. Every measured repeat starts fresh.
    warm = TestContext(asset, name=name, app=app, physics=physics)
    start = time.perf_counter()
    try:
        test(warm)
        assert warm.session.time >= 1., 'RTF warmup requires at least one simulated second'
        warmup = time.perf_counter()-start
    finally:
        warm.close()
    rounds=[]
    for _ in range(3):
        seconds=wall=prepare=0.
        finals=[]
        while seconds < 10-1e-8:
            start=time.perf_counter()
            ctx=TestContext(asset,name=name,app=app,physics=physics)
            construction=time.perf_counter()-start
            try:
                test(ctx)
                seconds+=ctx.session.time;wall+=time.perf_counter()-ctx.experiment_started
                prepare+=construction+ctx.preparation_seconds
                finals.append(ctx.rows[-1])
            finally:
                ctx.close()
        rounds.append({'simulated_seconds':seconds,'wall_seconds':wall,'RTF':seconds/wall,
                       'independent_repeats':len(finals),'preparation_reset_seconds':prepare,
                       'final_states':finals})
    return {'warmup':{'wall_seconds':warmup,'method':'one complete experiment in a disposable scene before measured rounds'},
            'rounds':rounds,'median_RTF':statistics.median(r['RTF'] for r in rounds),
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
            'lighting':{'dome_intensity':ctx.session.stage.GetPrimAtPath('/World/Light').GetAttribute('inputs:intensity').Get(),
                        'sun_intensity':ctx.session.stage.GetPrimAtPath('/World/Sun').GetAttribute('inputs:intensity').Get(),
                        'sun_rotation_xyz_degrees':list(ctx.session.stage.GetPrimAtPath('/World/Sun').GetAttribute('xformOp:rotateXYZ').Get())}
                       if ctx.camera_instance else None,
            'standard_dt':ctx.dt==.004}


def run_video(asset,name,test,output,app,startup,rtf,gpu,physics="cpu"):
    from table_1000.physics.testing import TestContext
    from table_1000.physics.plots import plots
    directory=output/name
    started=time.perf_counter()
    ctx=TestContext(asset,name=name,render=True,app=app,physics=physics)
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
            'physics_callbacks':'actions/state priority -100; official BehaviorScript priority 0 when present; batched force flush priority 100; one external physical step; actual solver and iterations in conditions',
            'performance':{'no_render':rtf,'video_end_to_end':{'simulated_seconds':ctx.session.time,'wall_seconds':wall,'RTF':ctx.session.time/wall},
                 'preparation':{'process_startup_seconds':startup,'construction_seconds':construction,'initialization_render_warmup_reset_seconds':ctx.preparation_seconds},
                 'independent_no_render_vs_video_final_max_scalar_error':max(errors)},
            'reaction_forces':'contact reaction forces not sampled; grip columns record the force each grip applies'}
        if getattr(ctx, 'experiment_details', None):
            result['experiment'] = ctx.experiment_details
        if ctx.grips:
            result['grips'] = ctx.grip_details()
        if getattr(ctx,'ground_config',None):
            result['ground_contact']=ground_clearance(json.loads((asset.parent/'model.json').read_text()),
                ctx.rows,ctx.session.instances,ctx.ground_config['z'])
        (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print('PHYSICS_TEST_COMPLETED',name,flush=True)
        return result
    finally:
        ctx.close()


def run_case(asset,name,test,output,app,startup,gpu,physics="cpu"):
    directory=output/name
    if directory.exists():shutil.rmtree(directory)
    (output/(name+'.mp4')).unlink(missing_ok=True)
    directory.mkdir(parents=True)
    try:
        rtf=benchmark(asset,name,test,app,physics=physics)
        return run_video(asset,name,test,output,app,startup,rtf,gpu,physics=physics)
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
    parser.add_argument('--physics',choices=('cpu','gpu'),default='cpu',help='physics backend; --gpu selects the physical device')
    parser.add_argument('--plots-only',action='store_true')
    args=parser.parse_args(argv)
    asset=args.asset.resolve()
    if asset.is_dir():asset=asset/('object.usda' if (asset/'object.usda').exists() else 'object.usdz')
    script=args.script or asset.parent/'physics_test.py'
    output=(args.output or asset.parent/'physics_test').resolve();output.mkdir(parents=True,exist_ok=True)
    if args.plots_only:
        tests=load_tests(script)
        if args.tests:tests={name:tests[name] for name in args.tests}
        from table_1000.physics.plots import plot_trace
        for name in tests:
            result=json.loads((output/name/'result.json').read_text())
            plot_trace(output/name/'trace.csv',result['plots'],result['columns'])
        return
    started=time.perf_counter()
    gpu={'physical_index':args.gpu,'before_process_start':subprocess.run(
        ['nvidia-smi','--query-gpu=index,name,driver_version,memory.used,utilization.gpu','--format=csv'],
        check=True,capture_output=True,text=True).stdout.strip()}
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    app=start_app(gpu=args.gpu, device='cpu' if args.physics=='cpu' else 'cuda:0',
                  extra_args=(*RT2_ARGS,'--/rtx/post/motionblur/enabled=False'))
    startup=time.perf_counter()-started
    failed=False
    try:
        tests=load_tests(script)
        if args.tests:tests={name:tests[name] for name in args.tests}
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
        with threadpool_limits(limits=1, user_api='blas'):
            for name,test in tests.items():
                run_case(asset,name,test,output,app,startup,gpu,physics=args.physics)
    except BaseException:
        failed=True
        traceback.print_exc()
        raise
    finally:
        app.close(exit_code=int(failed))
