"""Archive selected ``assets/`` directories in a Hugging Face dataset.

``assets/`` stays a local, Git-ignored cache shared by worktrees. Directories
worth keeping (a robot's raw vendor package, released assets) are uploaded to
the dataset; ``assets.lock.json`` at the repository root names the dataset, the
pinned dataset commit and the archived paths, so ``pull`` restores exactly that
version. ``push`` uploads one directory and updates the lock; commit the lock.

    python scripts/assets/remote.py pull [PATH ...]
    python scripts/assets/remote.py push robots/x1pro/raw/ex001_render -m "message"
    python scripts/assets/remote.py list

Needs ``huggingface_hub`` and a login (``hf auth login``) with access to the dataset.
"""

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOCK = ROOT / 'assets.lock.json'
ASSETS = ROOT / 'assets'


def read_lock(path=LOCK):
    return json.loads(Path(path).read_text())


def write_lock(lock, path=LOCK):
    lock = {**lock, 'paths': sorted(set(lock['paths']))}
    Path(path).write_text(json.dumps(lock, indent=2) + '\n')


def patterns(paths):
    """``allow_patterns`` selecting every file under each archived directory."""
    return [f'{path.strip("/")}/**' for path in paths]


def pull(lock, paths=None, assets=ASSETS):
    from huggingface_hub import snapshot_download
    selected = paths or lock['paths']
    unknown = sorted(set(selected) - set(lock['paths']))
    if unknown:
        raise SystemExit(f'not archived in {LOCK.name}: {unknown}')
    snapshot_download(lock['repo'], repo_type=lock['repo_type'], revision=lock['revision'],
                      allow_patterns=patterns(selected), local_dir=assets)
    return selected


def push(lock, path, message, assets=ASSETS):
    from huggingface_hub import HfApi
    path = path.strip('/')
    folder = Path(assets) / path
    if not folder.is_dir():
        raise SystemExit(f'{folder} is not a directory')
    commit = HfApi().upload_folder(repo_id=lock['repo'], repo_type=lock['repo_type'], folder_path=folder,
                                   path_in_repo=path, commit_message=message)
    return {**lock, 'revision': commit.oid, 'paths': [*lock['paths'], path]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    pull_parser = commands.add_parser('pull', help='download archived paths at the pinned revision')
    pull_parser.add_argument('paths', nargs='*')
    push_parser = commands.add_parser('push', help='upload one assets/ directory and pin the new revision')
    push_parser.add_argument('path')
    push_parser.add_argument('-m', '--message', required=True)
    commands.add_parser('list', help='show the lock')
    args = parser.parse_args(argv)
    lock = read_lock()
    if args.command == 'pull':
        print('pulled', pull(lock, args.paths), 'at', lock['revision'])
    elif args.command == 'push':
        lock = push(lock, args.path, args.message)
        write_lock(lock)
        print('pushed', args.path, '->', lock['revision'], f'(commit {LOCK.name})')
    else:
        print(json.dumps(lock, indent=2))
