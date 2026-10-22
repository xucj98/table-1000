"""Batch runs of the scripts: work units packed into processes, seeds and the commit they ran from.

A batch script's work units (seeds, cases, asset backends) are packed into ``--jobs`` processes
(:func:`plan`) that run at once: spread evenly over the local ``--gpus``, or each submitted through
``--launcher``, a job scheduler's command prefix such as ``srun --gres=gpu:1`` that picks the GPU.
Units whose process stopped before writing their result run again (``--retries``); see
:func:`run_batch`. Seeds are written as in ``--seeds``: ``a:b`` (``a`` to ``b - 1``) and single
seeds, e.g. ``0:100`` or ``20 64``.
"""

import argparse
import os
import shlex
import subprocess
import sys

# Processes at once through a launcher, unless --jobs says otherwise.
LAUNCHED_JOBS = 16


def add_arguments(parser):
    """``--jobs``, ``--gpus``, ``--launcher`` and ``--retries`` of a batch script, and its workers' ``--gpu``."""
    parser.add_argument('--jobs', type=int, help=f'processes at once (default: one per --gpus; {LAUNCHED_JOBS} with --launcher)')
    parser.add_argument('--gpus', type=int, nargs='+', help='local GPUs the processes are spread over (default 0); '
                        'not with --launcher')
    parser.add_argument('--launcher', help='command prefix of a job scheduler that runs a process on a free GPU it sets '
                        'CUDA_VISIBLE_DEVICES to; instead of --gpus')
    parser.add_argument('--retries', type=int, default=1, help='reruns of work whose process stopped before its result')
    parser.add_argument('--gpu', type=int, help=argparse.SUPPRESS)


def job_count(parser, args):
    """The processes at once of :func:`add_arguments`' options; sets the default ``args.gpus``."""
    if args.launcher and args.gpus:
        parser.error('--gpus runs locally; with --launcher the scheduler picks the GPUs')
    if args.jobs is not None and args.jobs < 1:
        parser.error('--jobs must be positive')
    args.gpus = args.gpus or [0]
    return args.jobs or (LAUNCHED_JOBS if args.launcher else len(args.gpus))


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


def run_jobs(jobs, gpus):
    """Run ``(command, log, label)`` jobs, one at a time per entry of ``gpus`` (a GPU listed twice runs two),
    each command given ``--gpu``; returns the failed labels."""
    from concurrent.futures import ThreadPoolExecutor
    import queue
    free = queue.Queue()
    for gpu in gpus:
        free.put(gpu)

    def run(job):
        command, log, label = job
        gpu = free.get()
        try:
            print('START', label, 'GPU', gpu, flush=True)
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open('w') as stream:
                code = subprocess.run([*command, '--gpu', str(gpu)], stdout=stream, stderr=subprocess.STDOUT).returncode
            print('DONE' if code == 0 else 'FAILED', label, log, flush=True)
            return code
        finally:
            free.put(gpu)

    with ThreadPoolExecutor(len(gpus)) as pool:
        codes = list(pool.map(run, jobs))
    return [job[2] for job, code in zip(jobs, codes) if code]


def launch(jobs, launcher, limit):
    """Run ``(command, log, label)`` jobs, ``limit`` at a time, each as ``<launcher> bash -c '<command> > <log> 2>&1'``;
    returns the failed labels. The launcher waits for a free GPU, sets ``CUDA_VISIBLE_DEVICES`` to it and returns
    the command's status; the command gets no ``--gpu`` and reads its GPU with :func:`worker_gpu`."""
    from concurrent.futures import ThreadPoolExecutor
    import signal
    import threading
    processes, lock = [], threading.Lock()

    def run(job):
        command, log, label = job
        log.parent.mkdir(parents=True, exist_ok=True)
        line = f'{launcher} bash -c {shlex.quote(f"{shlex.join(map(str, command))} > {shlex.quote(str(log))} 2>&1")}'
        print('SUBMIT', label, flush=True)
        with lock:
            process = subprocess.Popen(line, shell=True, start_new_session=True)
            processes.append(process)
        code = process.wait()
        print('DONE' if code == 0 else 'FAILED', label, log, flush=True)
        return code

    # Stopping the batch (Ctrl-C, SIGTERM) stops the launcher clients too, which cancel what they started;
    # the pool is shut down only after that, as its threads wait for the clients.
    previous = signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    pool = ThreadPoolExecutor(min(limit, len(jobs)))
    try:
        codes = list(pool.map(run, jobs))
    except BaseException:
        pool.shutdown(wait=False, cancel_futures=True)
        with lock:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
        raise
    finally:
        signal.signal(signal.SIGTERM, previous)
        pool.shutdown(wait=True)
    return [job[2] for job, code in zip(jobs, codes) if code]


def pack(items, cost, count):
    """``items`` in at most ``count`` groups of nearly equal total ``cost(item)``: the costliest first, each
    onto the lightest group. No group is empty."""
    groups = [[] for _ in range(max(1, min(count, len(items))))]
    totals = [0.] * len(groups)
    for item in sorted(items, key=cost, reverse=True):
        index = totals.index(min(totals))
        groups[index].append(item)
        totals[index] += cost(item)
    return groups


def plan(items, key, cost, count):
    """Groups of ``items`` for ``count`` processes: items of one ``key(item)`` share a process kind (e.g. one
    robot and physics backend) and are packed by :func:`pack`, or without ``cost`` (all alike) split into
    contiguous runs; each key gets processes in proportion to its total cost, at least one and at most one
    per item. With more keys than ``count``, there are more groups than ``count``: :func:`run_batch` runs
    ``count`` of them at a time."""
    weight = cost or (lambda item: 1)
    by_key = {}
    for item in items:
        by_key.setdefault(key(item), []).append(item)
    totals = {k: sum(map(weight, values)) for k, values in by_key.items()}
    counts = {k: max(1, round(count * total / sum(totals.values()))) for k, total in totals.items()}
    while sum(counts.values()) > count and any(c > 1 for c in counts.values()):
        k = min((k for k in counts if counts[k] > 1), key=lambda k: totals[k] / counts[k])
        counts[k] -= 1
    counts = {k: min(c, len(by_key[k])) for k, c in counts.items()}
    while sum(counts.values()) < count:
        open_keys = [k for k in counts if counts[k] < len(by_key[k])]
        if not open_keys:
            break
        counts[max(open_keys, key=lambda k: totals[k] / counts[k])] += 1
    return [group for k, values in by_key.items()
            for group in (pack(values, cost, counts[k]) if cost else split(values, counts[k]))]


def run_batch(items, *, command, missing, count, gpus, launcher=None, retries=1, key=lambda item: None, cost=None):
    """Run ``items`` (work units) as processes (:func:`plan`), at most ``count`` at a time, on ``gpus``
    (spread evenly, see :func:`run_jobs`) or each through ``launcher`` (see :func:`launch`).
    ``command(group, index, attempt)`` gives a process's ``(argv, log, label)``. Items still ``missing(item)``
    after a round (their process died, was killed or failed to start before writing them) run again, up to
    ``retries`` more rounds, in as many processes as before at most; items that wrote a result, even a failing
    one, do not. Returns the items still missing."""
    pending = list(items)
    for attempt in range(retries + 1):
        if not pending:
            break
        if attempt:
            print(f'RETRY {len(pending)} missing (attempt {attempt})', flush=True)
        jobs = [command(group, index, attempt) for index, group in enumerate(plan(pending, key, cost, count))]
        if launcher:
            launch(jobs, launcher, count)
        else:
            run_jobs(jobs, [gpus[index % len(gpus)] for index in range(min(count, len(jobs)))])
        pending = [item for item in pending if missing(item)]
    return pending


def worker_gpu(gpu):
    """The physical GPU of a worker process: ``gpu`` when given (``--gpu``), else the one a launcher made visible."""
    if gpu is not None:
        return gpu
    visible = os.environ.get('CUDA_VISIBLE_DEVICES', '0').split(',')[0]
    if not visible.isdigit():
        raise SystemExit(f'CUDA_VISIBLE_DEVICES={visible}: a GPU index is needed (not a UUID or MIG id)')
    return int(visible)


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
