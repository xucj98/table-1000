"""Mirror the Isaac Sim cloud assets that runs load into ``assets/isaac`` (see
``table_1000/sim/isaaclab/app.py``), so simulations need no network.

``python scripts/assets/download_isaac_assets.py``
"""

import argparse
from pathlib import Path
import shutil
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
CLOUD = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1'
# Isaac Lab's ground plane and the texture it references (OmniPBR.mdl ships with Kit).
FILES = ('Isaac/Environments/Grid/default_environment.usd',
         'Isaac/Environments/Grid/Materials/Textures/Wireframe_blue.png')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, default=ROOT / 'assets' / 'isaac')
    args = parser.parse_args(argv)
    for name in FILES:
        target = args.output / name
        target.parent.mkdir(parents=True, exist_ok=True)
        partial = target.with_name(target.name + '.part')
        with urllib.request.urlopen(f'{CLOUD}/{name}', timeout=60) as response, open(partial, 'wb') as out:
            shutil.copyfileobj(response, out)
        partial.replace(target)
        print(target)


if __name__ == '__main__':
    main()
