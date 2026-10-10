"""Actual Isaac integration check: portable load, isolation, reset and unload.

Run in the configured Isaac environment with --asset and --output. This check
imports only the shared loader; behavior classes are discovered from each asset.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    portable = args.output / 'relocated_asset'
    shutil.copytree(args.asset, portable, dirs_exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'create_new_stage': False, 'active_gpu': args.gpu, 'physics_gpu': args.gpu,
                         'multi_gpu': False, 'disable_viewport_updates': True, 'limit_cpu_threads': 4})
    try:
        import numpy as np
        from isaacsim.core.api import World
        from table_1000.physics.runtime import Runtime
        world = World(physics_dt=.004, rendering_dt=.04, stage_units_in_meters=1, backend='numpy', device='cpu')
        world.get_physics_context().set_gravity(0)
        runtime = Runtime(world)
        a = runtime.load(portable / 'object.usdz', 'a', position=(0, -.1, .15))
        b = runtime.load(portable / 'object.usdz', 'b', position=(0, .1, .15))
        runtime.initialize()
        for _ in range(10):
            runtime.step()
        pairs = dict(runtime.pairs)
        assert pairs == {('a', 'cap_mouth'): ('a', 'tip'), ('b', 'cap_mouth'): ('b', 'tip')}
        # A weak external pull only on instance a must leave b unchanged.
        baseline = b.behaviors['cap_fit'].metrics['opening']
        def pull(time, dt):
            runtime.apply_force(a.body_key('cap'), [-.5, 0, 0])
        runtime.external_actions = pull
        for _ in range(40):
            runtime.step()
        independent_error = abs(b.behaviors['cap_fit'].metrics['opening'] - baseline)
        assert independent_error < 1e-6
        runtime.external_actions = None
        runtime.reset()
        assert not runtime.pairs and all(not item.behaviors['cap_fit'].metrics['engaged'] for item in (a, b))
        for _ in range(5):
            runtime.step()
        assert runtime.pairs == pairs
        # At initialization only, exchange the cap positions. Matching is not
        # hard-wired to the original pen body or scene paths.
        ap, aq = a.view.get_world_poses();bp, bq = b.view.get_world_poses()
        ai, bi = a.indices['cap'], b.indices['cap']
        ap[ai], bp[bi] = bp[bi].copy(), ap[ai].copy()
        a.view.set_world_poses(ap, aq);b.view.set_world_poses(bp, bq)
        runtime.pairs.clear()
        for _ in range(5):
            runtime.step()
        exchanged = dict(runtime.pairs)
        assert exchanged == {('a', 'cap_mouth'): ('b', 'tip'), ('b', 'cap_mouth'): ('a', 'tip')}
        before_unload = (*b.view.get_world_poses(), b.view.get_velocities())
        runtime.unload('a')
        assert not runtime.stage.GetPrimAtPath(a.path).IsValid() and not runtime.pairs
        after_unload = (*b.view.get_world_poses(), b.view.get_velocities())
        unload_error = max(float(np.max(np.abs(old - new))) for old, new in zip(before_unload, after_unload))
        assert unload_error < 1e-6
        runtime.step()
        assert b.view.is_physics_handle_valid()
        runtime.reset()
        for _ in range(5):
            runtime.step()
        assert runtime.pairs == {('b', 'cap_mouth'): ('b', 'tip')}
        result = {'execution_status': 'completed', 'relocated_usdz': str(portable / 'object.usdz'),
                  'loaded': {item.name: item.load_evidence for item in (a, b)},
                  'normal_pairs': [[list(f), list(m)] for f, m in pairs.items()],
                  'exchanged_pairs': [[list(f), list(m)] for f, m in exchanged.items()],
                  'unaffected_instance_opening_error_m': independent_error,
                  'reset_reengaged': True, 'unload_removed_prim_and_pairs': True,
                  'unload_preserved_remaining_state_error': unload_error,
                  'continued_after_unload_without_reset': True}
        runtime.close()
        (args.output / 'runtime.json').write_text(json.dumps(result, indent=2) + '\n')
        print('RUNTIME_INTEGRATION_COMPLETED', args.output / 'runtime.json', flush=True)
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == '__main__':
    main()
