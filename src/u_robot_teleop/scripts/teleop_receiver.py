#!/usr/bin/env python3
"""Supervise native A2JP receiver without loading DDS into Python/ROS."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import yaml
from teleop_runtime import acquire_lock, owns_udp_port, receiver_command


def main():
    # Do not resolve this script's symlink: sibling executables live in install/.
    here = Path(os.path.abspath(__file__)).parent
    share = here.parent.parent / 'share' / 'u_robot_teleop'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pc1-ip', required=True)
    parser.add_argument('--config', default=str(share / 'config' / 'teleop.yaml'))
    parser.add_argument('--real-control', action='store_true')
    parser.add_argument('--special-actions', action='store_true')
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--port', type=int)
    parser.add_argument('--ack-port', type=int)
    parser.add_argument('--print-command', action='store_true')
    parser.add_argument('--watch-stdin', action='store_true', help='Exit if SSH heartbeat stops for 5 s or stdin reaches EOF')
    args = parser.parse_args()
    with open(args.config) as stream:
        config = yaml.safe_load(stream)
    for name in ('port', 'ack_port'):
        if getattr(args, name) is not None:
            config['receiver'][name] = getattr(args, name)
    binary = here / 'a2_network_bridge'
    cmd = receiver_command(config, binary, args.pc1_ip, args.real_control,
                           args.special_actions, args.duration)
    if args.print_command:
        print(json.dumps(cmd))
        return 0
    if not binary.is_file():
        raise RuntimeError('Native receiver not installed; build u_robot_teleop first')
    env = os.environ.copy()
    env.pop('CYCLONEDDS_URI', None)
    # These private libraries belong to the native SDK process only.
    env['LD_LIBRARY_PATH'] = str(here / 'native') + ':' + env.get('LD_LIBRARY_PATH', '')
    locks = []
    child = None
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        locks.append(acquire_lock(f"/tmp/u_robot_teleop_{config['receiver']['port']}.lock", 'teleop'))
        if args.real_control:
            if config['sport_lock'] != '/home/unitree/data/logs/a2_sport_control.lock':
                raise ValueError('sport_lock must match the navigation backend shared lock')
            locks.append(acquire_lock(config['sport_lock'], 'teleop'))
        print('TELEOP_START ' + json.dumps({'real_control': args.real_control,
              'peer': args.pc1_ip, 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'command': cmd}), flush=True)
        child = subprocess.Popen(cmd, env=env, start_new_session=True, pass_fds=tuple(locks))
        ready = False
        started = time.monotonic()
        last_heartbeat = started
        while child.poll() is None and not stopping:
            if not ready and owns_udp_port(child.pid, config['receiver']['port']):
                ready = True
                print('TELEOP_READY ' + json.dumps({'pid': child.pid, 'port': config['receiver']['port'],
                      'real_control': args.real_control}), flush=True)
            if not ready and time.monotonic() - started > 15:
                raise RuntimeError('Native receiver did not bind its UDP socket within 15 seconds')
            if args.watch_stdin:
                readable, _, _ = select.select([sys.stdin], [], [], 0.05)
                if readable:
                    if not os.read(sys.stdin.fileno(), 4096):
                        print('TELEOP_STOP SSH stdin closed', flush=True)
                        stopping = True
                    last_heartbeat = time.monotonic()
                if time.monotonic() - last_heartbeat > 5:
                    print('TELEOP_STOP SSH heartbeat expired', flush=True)
                    stopping = True
            else:
                time.sleep(0.05)
        return 0 if stopping else child.returncode
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        for fd in reversed(locks):
            os.close(fd)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
        print(f'TELEOP_ERROR {error}', file=sys.stderr, flush=True)
        sys.exit(2)
