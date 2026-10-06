#!/usr/bin/env python3
"""Wait for pinned benchmark runs, finalize even failed output, then optionally suspend."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
from provenance import portable_metadata


ROOT = Path(__file__).resolve().parents[1]


def now():
    return datetime.now(timezone.utc).isoformat()


def process_identity(pid):
    """Return Linux start ticks, excluding zombies, so reused PIDs do not block."""
    if not pid:
        return None
    try:
        tail = (Path('/proc') / str(int(pid)) / 'stat').read_text().rsplit(')', 1)[1].split()
        return None if tail[0] == 'Z' else int(tail[19])
    except (FileNotFoundError, ProcessLookupError):
        return None


def snapshot(runs, identities):
    snapshots = {}
    all_finished = True
    for name, path in runs.items():
        try:
            status = json.loads((path / 'status.json').read_text())
        except (OSError, ValueError) as error:
            snapshots[name] = {'read_error': str(error), 'run_dir': str(path)}
            all_finished = False
            continue
        processes = {'supervisor': {'pid': status.get('supervisor_pid')}}
        processes.update(status.get('processes', {}))
        active = {}
        for role, info in processes.items():
            if role != 'supervisor' and info.get('returncode') is not None:
                continue
            pid = info.get('pid')
            key = (name, role, pid)
            identity = process_identity(pid)
            # First sight of an already exited process stays exited if its PID
            # is subsequently reused. New subprocesses have their own keys.
            if key not in identities:
                identities[key] = identity
            if identity is not None and identity == identities[key]:
                active[role] = pid
        snapshots[name] = dict(run_dir=str(path), phase=status.get('phase'),
            message=status.get('message'), updated_utc=status.get('updated_utc'),
            active_processes=active,
            images=status.get('image_pairs_processed', status.get('images_received')),
            expected_images=status.get('expected_image_pairs', status.get('expected_images')),
            error_count=status.get('error_count'))
        all_finished = all_finished and not active
    return all_finished, snapshots


def write_state(path, state):
    state['updated_utc'] = now()
    temporary = path.with_suffix('.json.tmp')
    with temporary.open('w') as output:
        json.dump(state, output, indent=2)
        output.write('\n')
        output.flush()
        os.fsync(output.fileno())
    temporary.replace(path)


def finish(runs, results, state_dir, state, suspend):
    state_path = state_dir / 'status.json'
    state.update(phase='finalizing', message='All run processes exited; scoring completed and failed output')
    write_state(state_path, state)
    command = [sys.executable, str(Path(__file__).with_name('finalize_runs.py')),
               '--results', str(results)]
    for name, path in runs.items():
        command.extend(['--run', f'{name}={path}'])
    state['finalize_command'] = command
    write_state(state_path, state)
    with (state_dir / 'finalize.log').open('w') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    report = json.loads((results / 'statistics.json').read_text())
    if not all(name in report['methods'] for name in runs):
        raise RuntimeError('Final statistics are missing a requested method; suspend withheld')
    for name, path in runs.items():
        provenance = json.loads((results / f'{name}.source.json').read_text())
        # Stored run references are repository-relative or symbolic. Resolve
        # neither against the monitor's working directory; normalize both sides
        # through the same export contract used by calculate_statistics.py.
        recorded = portable_metadata(provenance['source_run'], repo_root=ROOT)
        pinned = portable_metadata(str(path.resolve()), repo_root=ROOT)
        if recorded != pinned:
            raise RuntimeError(f'{name}: final table belongs to a different run; suspend withheld')
        finalization_path = path/'finalization.json'
        finalization = json.loads(finalization_path.read_text()) if finalization_path.exists() else {}
        trajectory = (path/finalization.get('trajectory_file', 'est.csv')).resolve()
        if not trajectory.is_relative_to(path.resolve()):
            raise RuntimeError(f'{name}: final trajectory is outside its run; suspend withheld')
        from calculate_statistics import validate_trajectory
        try:
            validate_trajectory(trajectory, allow_empty=provenance.get('incomplete_collection', False))
        except (OSError, ValueError, KeyError) as exc:
            raise RuntimeError(f'{name}: invalid final trajectory; suspend withheld') from exc
        if report['methods'][name]['n_window'] != 4040:
            raise RuntimeError(f'{name}: wrong evaluation denominator; suspend withheld')
    state.update(statistics_saved_utc=now(), statistics=str(results / 'statistics.md'))
    os.sync()
    if suspend:
        state.update(phase='suspend_requested', suspend_requested_utc=now(),
                     message='Statistics saved; requesting Ubuntu suspend')
        write_state(state_path, state)
        with (state_dir / 'suspend.log').open('w') as log:
            subprocess.run(['systemctl', '--no-ask-password', 'suspend'],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        state['suspend_command_returncode'] = 0
    state.update(phase='completed', message='Statistics saved; suspend request accepted' if suspend
                 else 'Statistics saved; suspend was not requested')
    write_state(state_path, state)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', action='append', required=True, help='METHOD=run-directory; repeat per method')
    ap.add_argument('--results', type=Path, default=Path('results/final'))
    ap.add_argument('--state-dir', type=Path, default=Path('results/monitor_then_suspend'))
    ap.add_argument('--poll-seconds', type=float, default=300)
    ap.add_argument('--suspend', action='store_true', help='Explicitly authorize suspend after successful table update')
    args = ap.parse_args()
    if args.poll_seconds <= 0:
        ap.error('poll interval must be positive')
    runs = {}
    for specification in args.run:
        name, directory = specification.split('=', 1)
        if name in runs:
            ap.error('duplicate method')
        path = Path(directory).resolve(strict=True)
        if not (path / 'status.json').is_file():
            ap.error(f'missing run status: {path}')
        runs[name] = path
    results, state_dir = args.results.resolve(), args.state_dir.resolve()
    state_dir.mkdir(parents=True, exist_ok=True)
    with (state_dir / '.monitor.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        state_path = state_dir / 'status.json'
        if state_path.exists():
            ap.error('state directory already used; choose a new one to prevent duplicate suspend requests')
        (state_dir / 'watcher.pid').write_text(str(os.getpid()) + '\n')
        state = dict(phase='monitoring', started_utc=now(), watcher_pid=os.getpid(),
                     suspend_enabled=args.suspend, poll_seconds=args.poll_seconds,
                     runs={n: str(p) for n, p in runs.items()})
        def cancel(signum, _):
            state.update(phase='cancelled', message='Monitor cancelled; no further suspend request will be made')
            write_state(state_path, state)
            raise SystemExit(128 + signum)
        signal.signal(signal.SIGTERM, cancel)
        signal.signal(signal.SIGINT, cancel)
        identities = {}
        try:
            while True:
                ready, progress = snapshot(runs, identities)
                state['progress'] = progress
                write_state(state_path, state)
                if ready:
                    break
                time.sleep(args.poll_seconds)
            finish(runs, results, state_dir, state, args.suspend)
        except Exception as error:
            state.update(phase='error', message=str(error), traceback=traceback.format_exc())
            write_state(state_path, state)
            raise


if __name__ == '__main__':
    main()
