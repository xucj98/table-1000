"""Isaac check: run-local Kit settings, worker readback and a supervised shutdown.

Run with the Isaac Sim Python from the repository root, e.g.
``"$ISAAC_PYTHON" tests/runtime/isaac_app.py --output outputs/isaac-app --gpu 0``.
The script starts itself under the detached parent and waits for its record.
"""

import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from table_1000.runtime.app import isolated_environment, start_app
from table_1000.runtime.process import LIFECYCLE, mark_shutdown, start_detached


def child(output, gpu):
    app = start_app(output, gpu=gpu, workers=8)
    try:
        import carb
        from isaacsim.core.api import World
        world = World(physics_dt=.004, rendering_dt=.04, stage_units_in_meters=1, backend='numpy', device='cpu')
        world.reset()
        for _ in range(10):
            world.step(render=False)
        result = {'status': 'PASS', 'physics_dt': world.get_physics_dt(), 'steps': 10,
                  'physics_workers_after_world': carb.settings.get_settings().get('/persistent/physics/numThreads')}
        if result['physics_dt'] != .004 or result['physics_workers_after_world'] != 8:
            result['status'] = 'FAIL'
    except Exception as error:
        result = {'status': 'FAIL', 'error': repr(error)}
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    mark_shutdown(output, result['status'])
    app.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--child', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    if args.child:
        return child(output, args.gpu)
    output.mkdir(parents=True, exist_ok=False)
    start_detached([sys.executable, __file__, '--child', '--output', output, '--gpu', args.gpu],
                   cwd=Path.cwd(), output=output, env=isolated_environment(output))
    while True:
        time.sleep(2)
        record = json.loads((output / LIFECYCLE).read_text()) if (output / LIFECYCLE).exists() else {}
        if record.get('status') in ('exited', 'parent_exception'):
            break
    result = json.loads((output / 'result.json').read_text()) if (output / 'result.json').exists() else {}
    settings = json.loads((output / 'kit-settings.json').read_text()) if (output / 'kit-settings.json').exists() else {}
    print(json.dumps({'result': result, 'kit_settings': settings, 'returncode': record.get('returncode'),
                      'interventions': record.get('interventions')}, indent=2))
    return 0 if result.get('status') == 'PASS' and record.get('returncode') == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
