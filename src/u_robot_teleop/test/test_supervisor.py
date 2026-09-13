"""Installed receiver smoke checks: only dry-run, loopback and invalid packets."""
from pathlib import Path
import json
import os
import select
import socket
import subprocess
import tempfile
import time
import unittest
import yaml

WORKSPACE = Path(__file__).resolve().parents[3]
INSTALLED = WORKSPACE / 'install/u_robot_teleop/lib/u_robot_teleop/teleop_receiver.py'


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        if not INSTALLED.exists():
            self.skipTest('Run in the Docker workspace after installing u_robot_teleop')
        self.temp = tempfile.TemporaryDirectory()
        cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/teleop.yaml').read_text())
        self.port = self.free_port()
        self.ack_port = self.free_port()
        while self.port == self.ack_port:
            self.ack_port = self.free_port()
        cfg['receiver']['port'] = self.port
        cfg['receiver']['ack_port'] = self.ack_port
        cfg['receiver']['listen'] = '127.0.0.1'
        self.config = Path(self.temp.name) / 'teleop.yaml'
        self.config.write_text(yaml.safe_dump(cfg))
        self.children = []

    def tearDown(self):
        for child in self.children:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=9)
            for stream in (child.stdin, child.stdout):
                if stream and not stream.closed:
                    stream.close()
        self.temp.cleanup()

    @staticmethod
    def free_port():
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('127.0.0.1', 0))
            return sock.getsockname()[1]

    def start(self, extra=()):
        child = subprocess.Popen([str(INSTALLED), '--pc1-ip', '127.0.0.1', '--config', str(self.config),
                                  '--duration', '3', *extra], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.children.append(child)
        return child

    def ready(self, child):
        output = b''
        end = time.monotonic() + 6
        while time.monotonic() < end and child.poll() is None:
            if select.select([child.stdout], [], [], 0.1)[0]:
                output += os.read(child.stdout.fileno(), 4096)
                for line in output.splitlines():
                    if line.startswith(b'TELEOP_READY '):
                        data = json.loads(line[len(b'TELEOP_READY '):])
                        self.assertFalse(data['real_control'])
                        return data
        self.fail('Receiver not ready: ' + output.decode(errors='replace'))

    def test_dry_receiver_binds_and_rejects_malformed_packet(self):
        child = self.start()
        self.ready(child)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ack:
            ack.bind(('127.0.0.1', self.ack_port))
            ack.settimeout(0.2)
            ack.sendto(b'invalid-not-A2JP', ('127.0.0.1', self.port))
            with self.assertRaises(socket.timeout):
                ack.recvfrom(4096)
        self.assertEqual(child.wait(timeout=6), 0)

    def test_ssh_eof_stops_receiver_and_releases_udp_port(self):
        child = self.start(['--watch-stdin'])
        info = self.ready(child)
        child.stdin.close()
        self.assertEqual(child.wait(timeout=8), 0)
        self.assertFalse(Path(f"/proc/{info['pid']}").exists())
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('127.0.0.1', self.port))

    def test_duplicate_receiver_is_rejected(self):
        first = self.start()
        self.ready(first)
        second = self.start()
        self.assertEqual(second.wait(timeout=2), 2)
        self.assertIsNone(first.poll())


if __name__ == '__main__':
    unittest.main()
