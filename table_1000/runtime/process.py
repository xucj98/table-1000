"""Run a simulator process under a detached parent that records its real exit.

Isaac Sim can hang in ``app.close()`` after a run has saved its results. The
child marks that point with :func:`mark_shutdown`; from then on the parent allows
``shutdown_timeout_s`` before sending SIGTERM to the child's process group and
``term_grace_s`` more before SIGKILL. Without the marker no deadline applies, so
initialization and long runs are never cut short. The task result and the OS
exit status are recorded separately; a signal never turns a saved FAIL into PASS.

Start a run with :func:`start_detached`; the parent writes
``parent-lifecycle.json`` (status, child PID, returncode/signal, interventions)
and ``parent-heartbeat.json`` next to the child's ``stdout.log`` in ``output``.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

LIFECYCLE = 'parent-lifecycle.json'
SHUTDOWN = 'shutdown.json'


def write_json(path, value):
    """Replace ``path`` atomically so readers never see a partial file."""
    path = Path(path)
    temporary = path.with_name(path.name + '.writing')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def mark_shutdown(output, task_status):
    """Called by the child after all results are saved, right before ``app.close()``."""
    write_json(Path(output) / SHUTDOWN, {'stage': 'shutdown_started', 'task_status': task_status,
                                         'monotonic': time.monotonic(), 'epoch': time.time()})


def supervise(command, *, cwd, output, env=None, interval_s=1., shutdown_timeout_s=60., term_grace_s=15.,
              stall_timeout_s=300.):
    """Run ``command`` in its own process group and return its returncode.

    The child is stopped (TERM, then KILL after ``term_grace_s``) when
    ``app.close()`` overruns ``shutdown_timeout_s`` after the results were saved,
    or when its ``heartbeat.json`` (or, before the first heartbeat, its start)
    is older than ``stall_timeout_s`` -- e.g. a renderer that never finishes
    starting.
    """
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    record = {'status': 'starting', 'command': [str(part) for part in command], 'cwd': str(cwd),
              'parent_pid': os.getpid(), 'child_pid': None, 'returncode': None, 'signal': None,
              'start_epoch': time.time(), 'shutdown_marker': None, 'interventions': [], 'parent_signals': []}
    write_json(output / LIFECYCLE, record)
    started = time.monotonic()
    process = None

    def forward(signum, _frame):
        record['parent_signals'].append(signum)
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signum)

    previous = {value: signal.signal(value, forward) for value in (signal.SIGTERM, signal.SIGINT)}
    try:
        with (output / 'stdout.log').open('w') as log:
            process = subprocess.Popen(record['command'], cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            record.update(status='running', child_pid=process.pid)
            write_json(output / LIFECYCLE, record)
            shutdown_seen = stalled = None
            beat = record['start_epoch']
            while process.poll() is None:
                now = time.monotonic()
                if shutdown_seen is None:
                    marker = read_json(output / SHUTDOWN)
                    if marker and marker.get('stage') == 'shutdown_started':
                        shutdown_seen = now
                        record['shutdown_marker'] = marker
                        write_json(output / LIFECYCLE, record)
                if shutdown_seen is None and stalled is None:
                    # A heartbeat written in place may be read half-written; keep the last one read.
                    beat = (read_json(output / 'heartbeat.json') or {}).get('epoch', beat)
                    if time.time() - beat > stall_timeout_s:
                        stalled = now
                sent = [row['signal'] for row in record['interventions']]
                due = reason = None
                if shutdown_seen is not None:
                    reason = 'app.close exceeded the shutdown deadline after results were saved'
                    if not sent and now - shutdown_seen >= shutdown_timeout_s:
                        due = signal.SIGTERM
                    elif sent == [signal.SIGTERM] and now - shutdown_seen >= shutdown_timeout_s + term_grace_s:
                        due = signal.SIGKILL
                elif stalled is not None:
                    reason = f'no heartbeat for {stall_timeout_s:.0f} s'
                    if not sent:
                        due = signal.SIGTERM
                    elif sent == [signal.SIGTERM] and now - stalled >= term_grace_s:
                        due = signal.SIGKILL
                if due is not None:
                    try:
                        os.killpg(process.pid, due)
                    except ProcessLookupError:
                        pass
                    record['interventions'].append({
                        'signal': int(due), 'epoch': time.time(), 'target_process_group': process.pid,
                        'reason': reason})
                    write_json(output / LIFECYCLE, record)
                write_json(output / 'parent-heartbeat.json', {
                    'parent_pid': os.getpid(), 'child_pid': process.pid, 'epoch': time.time(),
                    'child_heartbeat': read_json(output / 'heartbeat.json')})
                time.sleep(interval_s)
        code = int(process.returncode)
        record.update(status='exited', returncode=code, signal=-code if code < 0 else None,
                      end_epoch=time.time(), elapsed_wall_s=time.monotonic() - started)
        write_json(output / LIFECYCLE, record)
        return code
    except BaseException as error:
        record.update(status='parent_exception', error=repr(error), end_epoch=time.time())
        write_json(output / LIFECYCLE, record)
        raise
    finally:
        for value, handler in previous.items():
            signal.signal(value, handler)


def start_detached(command, *, cwd, output, env=None):
    """Start a parent that outlives the caller's terminal; return its PID."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    spec = output / 'parent-spec.json'
    write_json(spec, {'command': [str(part) for part in command], 'cwd': str(cwd),
                      'output': str(output), 'env': env})
    package_root = Path(__file__).resolve().parents[2]
    with (output / 'parent.log').open('w') as log:
        parent = subprocess.Popen([sys.executable, '-m', 'table_1000.runtime.process', str(spec)],
                                  cwd=package_root, stdin=subprocess.DEVNULL, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
    return parent.pid


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('spec', type=Path)
    args = parser.parse_args(argv)
    spec = json.loads(args.spec.read_text())
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    code = supervise(spec['command'], cwd=spec['cwd'], output=spec['output'], env=spec.get('env'))
    return 128 - code if code < 0 else code


if __name__ == '__main__':
    raise SystemExit(main())
