"""Batch runs of the scene scripts: seeds, the split into processes and the commit they ran from.

Seeds are written as in ``--seeds``: ``a:b`` (``a`` to ``b - 1``) and single seeds,
e.g. ``0:100`` or ``20 64``. A batch is split into ``len(gpus) × jobs_per_gpu``
contiguous parts, one process each; a part that fails does not stop the others.
"""

import subprocess


def parse_seeds(values):
    """Seeds of ``--seeds`` values, sorted without duplicates."""
    result = set()
    for value in values:
        if ':' in value:
            start, stop = map(int, value.split(':'))
            result.update(range(start, stop))
        else:
            result.add(int(value))
    return sorted(result)


def seed_range(seeds):
    """The shortest ``--seeds`` text of ``seeds``: ``0:100``, ``20 64``, ``0:50 60:70``."""
    seeds, parts, index = sorted(seeds), [], 0
    while index < len(seeds):
        stop = index
        while stop + 1 < len(seeds) and seeds[stop + 1] == seeds[stop] + 1:
            stop += 1
        parts.append(str(seeds[index]) if stop == index else f'{seeds[index]}:{seeds[stop] + 1}')
        index = stop + 1
    return ' '.join(parts)


def seeds_record(seeds):
    """``{'range', 'count'}`` of a batch's seeds, as summaries report them."""
    return {'range': seed_range(seeds), 'count': len(seeds)}


def split(values, count):
    """``values`` in ``count`` contiguous parts of nearly equal size (no empty parts)."""
    count = max(1, min(count, len(values)))
    size, extra = divmod(len(values), count)
    parts, start = [], 0
    for index in range(count):
        stop = start + size + (index < extra)
        parts.append(values[start:stop])
        start = stop
    return parts


def run_parts(jobs, gpus, jobs_per_gpu):
    """Run ``(command, log, label)`` jobs, ``jobs_per_gpu`` at a time on each GPU; returns the failed labels."""
    from table_1000.physics.test import run_jobs
    return run_jobs(jobs, [gpu for _ in range(jobs_per_gpu) for gpu in gpus])


def commit(root):
    """The checked-out commit of ``root``, ``+dirty`` with uncommitted changes (or None)."""
    try:
        head = subprocess.run(['git', '-C', str(root), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', str(root), 'status', '--porcelain', '--untracked-files=no'],
                               capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    return head + ('+dirty' if dirty else '')
