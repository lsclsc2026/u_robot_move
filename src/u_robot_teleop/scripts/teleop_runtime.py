"""Configuration and process ownership for the unmodified native receiver."""
import fcntl
import ipaddress
import math
import os


def number(value, low, high, name, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be numeric')
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be in [{low}, {high}]')
    if integer and int(value) != value:
        raise ValueError(f'{name} must be an integer')
    return int(value) if integer else float(value)


def peer_address(value):
    addr = ipaddress.IPv4Address(value)
    if addr.is_unspecified or addr.is_multicast or int(addr) == 0xffffffff:
        raise ValueError('PC1 must be a specific unicast IPv4 address')
    return str(addr)


def receiver_command(config, binary, peer, real, special, duration):
    peer = peer_address(peer)
    r = config['receiver']
    duration = number(duration, 0, 86400 * 30, 'duration')
    port = number(r['port'], 1, 65535, 'port', True)
    ack = number(r['ack_port'], 1, 65535, 'ack_port', True)
    if port == ack:
        raise ValueError('command and ACK ports must differ')
    hz = number(r['command_hz'], 1, 100, 'command_hz', True)
    soft = number(r['soft_timeout_ms'], 50, 10000, 'soft_timeout_ms', True)
    hard = number(r['hard_timeout_ms'], soft + 1, 60000, 'hard_timeout_ms', True)
    deadzone = number(r['axis_deadzone'], 0, 0.99, 'axis_deadzone')
    listen = str(ipaddress.IPv4Address(r['listen']))
    if special and not real:
        raise ValueError('special actions require explicit real control')
    if r['deadman_button'] not in ('none', 'F1'):
        raise ValueError('deadman_button must be none or F1')
    cmd = [str(binary), '--mode', 'receiver', '--interface', str(r['interface']),
           '--listen', listen, '--port', str(port), '--ack-port', str(ack),
           '--allowed-source', peer, '--duration', str(duration),
           '--command-hz', str(hz), '--soft-timeout-ms', str(soft),
           '--hard-timeout-ms', str(hard), '--axis-deadzone', str(deadzone),
           '--deadman-button', r['deadman_button']]
    for axis in ('lx', 'ly', 'rx', 'ry'):
        values = config['axes'][axis]
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError(f'{axis} requires min, center, max')
        low, center, high = [number(v, -10, 10, axis) for v in values]
        if not low < center < high:
            raise ValueError(f'{axis} requires min < center < max')
        for suffix, value in zip(('min', 'center', 'max'), (low, center, high)):
            cmd.extend([f'--{axis}-{suffix}', str(value)])
    if type(r['native_button_map']) is not bool:
        raise ValueError('native_button_map must be a YAML boolean')
    if r['native_button_map']:
        cmd.append('--native-button-map')
    if real:
        cmd.append('--enable-robot-command')
    if special:
        cmd.append('--enable-special-actions')
    return cmd


def acquire_lock(path, owner):
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        os.ftruncate(fd, 0)
        os.write(fd, f'{owner} pid={os.getpid()}\n'.encode())
        return fd
    except BaseException:
        os.close(fd)
        raise


def owns_udp_port(pid, port):
    """Readiness means this child owns the socket, not an unrelated listener."""
    sockets = set()
    try:
        for fd in os.scandir(f'/proc/{pid}/fd'):
            try:
                link = os.readlink(fd.path)
                if link.startswith('socket:['):
                    sockets.add(link[8:-1])
            except FileNotFoundError:
                continue
        with open(f'/proc/{pid}/net/udp') as stream:
            for line in stream.readlines()[1:]:
                fields = line.split()
                if int(fields[1].split(':')[1], 16) == port and fields[9] in sockets:
                    return True
    except FileNotFoundError:
        pass
    return False
