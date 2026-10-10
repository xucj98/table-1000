"""Run configured real Isaac physics tests with portable asset behaviors."""

import argparse
import csv
import json
from pathlib import Path
import subprocess
import shutil
import time
import traceback
import numpy as np
from table_1000.physics.config import test_configs


def initial_state(runtime, instance, config):
    from table_1000.physics.joints import set_initial
    set_initial(instance, config.get('joints', {}))
    pos, quat = instance.view.get_world_poses()
    for name, values in config.get('rigid_bodies', {}).items():
        i = instance.indices[name]
        pos[i] = values.get('position', pos[i])
        quat[i] = values.get('rotation', quat[i])
    instance.view.set_world_poses(pos.astype(np.float32), quat.astype(np.float32))
    runtime.refresh_states()
    centers = np.array([runtime.state(instance.body_key(name))['com'] for name in instance.body_names])
    omega = np.asarray(config.get('angular_velocity', [0, 0, 0]))
    linear = np.asarray(config.get('linear_velocity', [0, 0, 0]))
    origin = np.asarray(config.get('position', [0, 0, 0]))
    velocity = np.c_[linear + np.cross(omega, centers - origin), np.tile(omega, (len(pos), 1))]
    for name, values in config.get('rigid_bodies', {}).items():
        i = instance.indices[name]
        velocity[i, :3] = values.get('linear_velocity', velocity[i, :3])
        velocity[i, 3:] = values.get('angular_velocity', velocity[i, 3:])
    instance.view.set_velocities(velocity.astype(np.float32))
    runtime.refresh_states()


def record(runtime, instance, actions, observe):
    row = {'time': runtime.time}
    for name in observe['rigid_bodies']:
        state = runtime.state(instance.body_key(name))
        for field, labels in [('position', 'xyz'), ('quaternion', 'wxyz'), ('velocity', ('vx', 'vy', 'vz', 'wx', 'wy', 'wz'))]:
            for label, value in zip(labels, state[field]):
                row[f'rigid_bodies.{name}.{label if field == "velocity" else field + "." + label}'] = float(value)
    for name in observe['actions']:
        for field, value in actions.records.get(name, {'active': 0}).items():
            array = np.asarray(value).reshape(-1)
            for i, component in enumerate(array):
                row[f'actions.{name}.{field}' + (f'.{i}' if len(array) > 1 else '')] = float(component)
    from table_1000.physics.joints import frames
    for name in observe['joints']:
        info = frames(instance, name)
        row[f'joints.{name}.position'] = info['position']
        row[f'joints.{name}.velocity'] = info['velocity']
    for name, behavior in instance.behaviors.items():
        for field, value in behavior.metrics.items():
            row[f'behaviors.{name}.{field}'] = float(value)
    return row


class Camera:
    def __init__(self, stage, config):
        import carb
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom, UsdLux
        for setting in ['/omni/replicator/captureOnPlay', '/omni/replicator/asyncRendering', '/app/asyncRendering']:
            carb.settings.get_settings().set(setting, False)
        self.rep, self.size = rep, config['resolution']
        camera = UsdGeom.Camera.Define(stage, '/World/Camera')
        camera.CreateFocalLengthAttr(28)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.001, 100))
        transform = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*config['position']), Gf.Vec3d(*config['target']), Gf.Vec3d(*config['up'])).GetInverse()
        UsdGeom.Xformable(camera).AddTransformOp().Set(transform)
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000)
        self.product = rep.create.render_product(camera.GetPath(), tuple(self.size))
        self.rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        self.rgb.attach([self.product])

    def capture(self, runtime, name, actions):
        from PIL import Image, ImageDraw
        from omni.physx import get_physx_interface
        get_physx_interface().update_transformations(True, True, True, False)
        before = {key: value['position'].copy() for key, value in runtime._states.items()}
        self.rep.orchestrator.step(rt_subframes=1, delta_time=0., pause_timeline=False)
        runtime.refresh_states()
        if any(not np.array_equal(value, runtime.state(key)['position']) for key, value in before.items()):
            raise RuntimeError('Rendering advanced physical state')
        image = Image.fromarray(self.rgb.get_data()[:, :, :3])
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, self.size[0], 35), fill='white')
        from table_1000.physics.actions import interpolate
        active = ', '.join(key for key, value in actions.definitions.items() if interpolate(value['keyframes'], runtime.time) is not None)
        draw.text((10, 5), f'Isaac 5.1 | {name} | t={runtime.time:.2f}s | 1x', fill='black')
        draw.text((10, 20), f'Actions: {active or "none"}', fill='black')
        return image

    def close(self):
        self.rgb.detach()
        self.product.destroy()


def plots(rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    columns = sorted(set().union(*(row.keys() for row in rows)))
    groups = [('positions', [key for key in columns if '.position.' in key]),
              ('velocities', [key for key in columns if key.startswith('rigid_bodies.') and key.rsplit('.', 1)[-1] in ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')]),
              ('actions', [key for key in columns if key.startswith('actions.') and '.value' in key]),
              ('behaviors', [key for key in columns if key.startswith('behaviors.')])]
    groups.append(('joints', [key for key in columns if key.startswith('joints.')]))
    for title, selected in groups:
        if not selected:
            continue
        fig, ax = plt.subplots(figsize=(10, 5))
        for key in selected:
            ax.plot([row['time'] for row in rows], [row.get(key, 0) for row in rows], label=key)
        ax.set_xlabel('Simulation time (s)');ax.set_title(title);ax.legend(fontsize=6)
        fig.tight_layout();fig.savefig(output / (title + '.png'));plt.close(fig)


def run_test(asset, name, config, output, startup):
    from isaacsim.core.api import World
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    from table_1000.physics.runtime import Runtime
    from table_1000.physics.actions import Actions
    dt, duration = config['simulation']['dt'], config['duration']
    World.clear_instance()
    import omni.usd
    omni.usd.get_context().new_stage()
    world = World(physics_dt=dt, rendering_dt=1 / config['camera']['fps'], stage_units_in_meters=1, backend='numpy', device='cpu')
    pc = world.get_physics_context();pc.enable_gpu_dynamics(False);pc.set_solver_type('TGS');pc.set_broadphase_type('MBP');pc.enable_ccd(True)
    gravity = np.asarray(config['simulation']['gravity'], dtype=float)
    scene = UsdPhysics.Scene.Get(omni.usd.get_context().get_stage(), pc.prim_path)
    magnitude = np.linalg.norm(gravity)
    scene.CreateGravityMagnitudeAttr(float(magnitude))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(*(gravity / magnitude if magnitude else [0, 0, -1])))
    if config.get('ground'):
        g = config['ground']
        floor = UsdGeom.Cube.Define(omni.usd.get_context().get_stage(), '/World/Ground');floor.CreateSizeAttr(1)
        transform = UsdGeom.Xformable(floor);transform.AddTranslateOp().Set(Gf.Vec3d(0, 0, g['z'] - .05));transform.AddScaleOp().Set(Gf.Vec3f(4, 4, .1))
        floor.CreateDisplayColorAttr([Gf.Vec3f(.46, .49, .53)]);UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(floor.GetPrim()).CreateContactOffsetAttr(.02)
        material = UsdShade.Material.Define(omni.usd.get_context().get_stage(), '/World/GroundMaterial')
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        for key, method in [('static_friction', 'CreateStaticFrictionAttr'), ('dynamic_friction', 'CreateDynamicFrictionAttr'), ('restitution', 'CreateRestitutionAttr')]:
            getattr(api, method)(g[key])
        UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material, materialPurpose='physics')
    runtime = Runtime(world)
    initial = config.get('initial', {})
    instance = runtime.load(asset, 'asset', initial.get('position', [0, 0, 0]), initial.get('rotation', [1, 0, 0, 0]))
    runtime.initialize();initial_state(runtime, instance, initial)
    actions = Actions(runtime, instance, config['actions']);runtime.external_actions = actions
    camera = Camera(runtime.stage, config['camera'])
    directory = output / Path(name).stem;directory.mkdir(parents=True, exist_ok=True)
    frames = directory / 'frames'
    if frames.exists():
        shutil.rmtree(frames)
    frames.mkdir()
    times = {'physics_seconds': 0., 'sampling_seconds': 0., 'render_encode_seconds': 0., 'startup_seconds': startup}
    stride = round(1 / (dt * config['camera']['fps']))
    rows = []
    def sample():
        start = time.perf_counter()
        rows.append(record(runtime, instance, actions, config['observe']))
        times['sampling_seconds'] += time.perf_counter() - start
    runtime.observer = sample
    start = time.perf_counter();camera.capture(runtime, name, actions);times['render_encode_seconds'] += time.perf_counter() - start
    for step in range(round(duration / dt) + 1):
        if step % stride == 0:
            start = time.perf_counter();camera.capture(runtime, name, actions).save(frames / f'{step // stride:05}.jpg');times['render_encode_seconds'] += time.perf_counter() - start
        if step < round(duration / dt):
            sampled = times['sampling_seconds']
            start = time.perf_counter();runtime.step();times['physics_seconds'] += time.perf_counter() - start - (times['sampling_seconds'] - sampled)
    actions(duration, dt)
    sample()
    fields = sorted(set().union(*(row.keys() for row in rows)))
    with (directory / 'trace.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fields);writer.writeheader();writer.writerows(rows)
    start = time.perf_counter()
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(config['camera']['fps']), '-i', str(frames / '%05d.jpg'), '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', str(output / name)], check=True)
    times['render_encode_seconds'] += time.perf_counter() - start
    plots(rows, directory)
    acceptance = {'execution_status': 'completed', 'review_status': 'pending', 'config': config, 'engine': 'Isaac Sim 5.1 / CPU PhysX TGS',
                  'behavior_load': instance.load_evidence, 'runtime_manifest': str(instance.manifest_path),
                  'physics': json.loads((asset.parent / 'physics_build.json').read_text()), 'timing': times,
                  'physics_RTF': duration / times['physics_seconds'], 'steps': round(duration / dt), 'frames': len(list(frames.glob('*.jpg'))),
                  'sampling': 'State and applied actions at the start of each physical step; final state at duration. Rendering advances zero time.',
                  'step_callback': 'PhysX subscribe_physics_on_step_events(pre_step=True)',
                  'reaction_forces': 'Not sampled; test forces are not constraint reaction forces'}
    (directory / 'acceptance.json').write_text(json.dumps(acceptance, indent=2) + '\n')
    camera.close();actions.close();runtime.close()
    print('PHYSICS_TEST_COMPLETED', name, flush=True)
    return acceptance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset', type=Path, help='object.usdz or its directory')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, help='Results directory; defaults to the asset physics_test directory')
    parser.add_argument('--tests', nargs='+')
    parser.add_argument('--gpu', type=int, default=0)
    args = parser.parse_args(argv)
    asset = args.asset.resolve();asset = asset / 'object.usdz' if asset.is_dir() else asset
    output = (args.output or asset.parent / 'physics_test').resolve()
    model = json.loads((asset.parent / 'model.json').read_text())
    config = json.loads((args.config or asset.parent / 'physics_test.json').read_text())
    tests = test_configs(config, model)
    if args.tests:
        tests = {name: tests[name] for name in args.tests}
    output.mkdir(parents=True, exist_ok=True)
    start = time.perf_counter()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'create_new_stage': False, 'active_gpu': args.gpu, 'physics_gpu': args.gpu,
                         'multi_gpu': False, 'disable_viewport_updates': True, 'limit_cpu_threads': 4})
    startup = time.perf_counter() - start
    try:
        for name, test in tests.items():
            directory = output / Path(name).stem
            if directory.exists():
                shutil.rmtree(directory)
            (output / name).unlink(missing_ok=True)
            run_test(asset, name, test, output, startup)
    except Exception:
        traceback.print_exc()
        failed = output / Path(name).stem / 'acceptance.json'
        failed.parent.mkdir(parents=True, exist_ok=True)
        failed.write_text(json.dumps({'execution_status': 'failed', 'review_status': 'pending',
                                      'config': test, 'error': traceback.format_exc()}, indent=2) + '\n')
        raise
    finally:
        app.close()
