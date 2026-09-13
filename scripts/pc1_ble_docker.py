#!/usr/bin/env python3
"""Run on PC1 WSL: start Docker receiver over SSH, then optionally start BLE sender.

Everything after -- is the existing Windows Python/BLE command, passed verbatim.
This entry neither decodes BLE nor rewrites A2JP packets.
"""
import argparse
import ipaddress
import queue
import re
import shlex
import signal
import subprocess
import sys
import threading
import time


def ssh_command(host, peer, real, container):
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*', host):
        raise ValueError('Invalid SSH alias/user@host')
    addr = ipaddress.IPv4Address(peer)
    if addr.is_unspecified or addr.is_multicast or int(addr) == 0xffffffff:
        raise ValueError('PC1 address must be its Windows LAN IPv4')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', container):
        raise ValueError('Invalid Docker container name')
    remote = ['bash', '/home/unitree/unitree_robot_development/u_robot_move/scripts/run_teleop_docker.sh',
              '--container', container, '--pc1-ip', str(addr), '--watch-stdin']
    if real:
        remote.append('--real-control')
    return ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=3', host,
            shlex.join(remote)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot', default='unitree-robot')
    p.add_argument('--pc1-ip', required=True, help='Windows LAN address, NOT WSL NAT address')
    p.add_argument('--container', default='unitree-dev')
    p.add_argument('--real-control', action='store_true')
    p.add_argument('--print-command', action='store_true')
    p.add_argument('sender', nargs=argparse.REMAINDER)
    args = p.parse_args()
    cmd = ssh_command(args.robot, args.pc1_ip, args.real_control, args.container)
    if args.print_command:
        print(shlex.join(cmd))
        return 0
    sender_command = args.sender[1:] if args.sender[:1] == ['--'] else args.sender
    messages = queue.Queue()
    stopped = threading.Event()
    receiver = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1)
    sender = None

    def stop(_sig, _frame):
        stopped.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def collect():
        for line in receiver.stdout:
            print(line, end='', flush=True)
            if line.startswith('TELEOP_READY '):
                messages.put(line)

    def heartbeat():
        while not stopped.is_set() and receiver.poll() is None:
            try:
                receiver.stdin.write('.\n')
                receiver.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                break
            stopped.wait(1)

    reader = threading.Thread(target=collect, daemon=True)
    beat = threading.Thread(target=heartbeat, daemon=True)
    reader.start()
    beat.start()
    try:
        deadline = time.monotonic() + 25
        ready = False
        while not stopped.is_set() and time.monotonic() < deadline and receiver.poll() is None:
            try:
                if messages.get(timeout=0.2).startswith('TELEOP_READY '):
                    ready = True
                    break
            except queue.Empty:
                pass
        if not ready:
            if stopped.is_set():
                return 130
            raise RuntimeError('Docker receiver did not become ready; BLE sender was not started')
        if sender_command:
            sender = subprocess.Popen(sender_command)
        else:
            print('Receiver ready. Start your unchanged Windows BLE sender in another terminal; Ctrl+C closes the receiver.', flush=True)
        while not stopped.wait(0.2):
            if receiver.poll() is not None:
                return receiver.returncode or 1
            if sender is not None and sender.poll() is not None:
                return sender.returncode
        return 130
    finally:
        # Close control channel FIRST: even if Windows Python remains alive,
        # its UDP data cannot continue reaching an enabled receiver.
        stopped.set()
        beat.join(timeout=2)
        try:
            receiver.stdin.close()
        except (OSError, BrokenPipeError):
            pass
        try:
            receiver.wait(timeout=10)
        except subprocess.TimeoutExpired:
            receiver.terminate()
            try:
                receiver.wait(timeout=3)
            except subprocess.TimeoutExpired:
                receiver.kill()
                receiver.wait()
        reader.join(timeout=2)
        receiver.stdout.close()
        if sender is not None and sender.poll() is None:
            sender.terminate()
            try:
                sender.wait(timeout=3)
            except subprocess.TimeoutExpired:
                sender.kill()
                sender.wait()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(f'PC1_TELEOP_ERROR {error}', file=sys.stderr)
        sys.exit(2)
