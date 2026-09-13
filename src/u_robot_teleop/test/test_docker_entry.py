"""Host entry argv checks only: fake Docker and process guard, no receiver."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


WORKSPACE = Path(__file__).resolve().parents[3]


class DockerEntryTests(unittest.TestCase):
    def invoke(self, arguments=(), overrides=None):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            checkout = root / 'different host checkout' / 'scripts'
            checkout.mkdir(parents=True)
            entry = checkout / 'run_teleop_docker.sh'
            shutil.copyfile(WORKSPACE / 'scripts/run_teleop_docker.sh', entry)
            fake_bin = root / 'fake-bin'
            fake_bin.mkdir()
            capture = root / 'docker-argv'
            # Replace side-effect boundaries, leaving the real shell entry's
            # path resolution, argument forwarding and validation intact.
            for name, body in {
                'docker': '#!/bin/sh\nprintf "%s\\n" "$@" > "$DOCKER_ARGV_FILE"\n',
                'python3': '#!/bin/sh\nexit 0\n',
            }.items():
                executable = fake_bin / name
                executable.write_text(body)
                executable.chmod(0o755)
            env = os.environ.copy()
            env.pop('TELEOP_CONTAINER', None)
            env.pop('TELEOP_CONTAINER_WORKSPACE', None)
            env.update({
                'PATH': str(fake_bin) + os.pathsep + env['PATH'],
                'DOCKER_ARGV_FILE': str(capture),
            })
            env.update(overrides or {})
            subprocess.run(['bash', str(entry), *arguments], env=env,
                           check=True, capture_output=True, text=True, timeout=5)
            return capture.read_text().splitlines()

    def test_default_container_path_is_independent_of_host_checkout(self):
        self.assertEqual(self.invoke(['--pc1-ip', '192.0.2.10']), [
            'exec', '-i', 'unitree-review',
            '/home/unitree/unitree_robot_development/u_robot_move/install/'
            'u_robot_teleop/lib/u_robot_teleop/teleop_receiver.py',
            '--pc1-ip', '192.0.2.10',
        ])

    def test_explicit_container_and_workspace_override_are_forwarded(self):
        self.assertEqual(self.invoke(
            ['--container', 'custom-review', '--pc1-ip', '192.0.2.10'],
            {'TELEOP_CONTAINER': 'ignored-default',
             'TELEOP_CONTAINER_WORKSPACE': '/opt/custom audio workspace/move'},
        ), [
            'exec', '-i', 'custom-review',
            '/opt/custom audio workspace/move/install/u_robot_teleop/lib/'
            'u_robot_teleop/teleop_receiver.py', '--pc1-ip', '192.0.2.10',
        ])


if __name__ == '__main__':
    unittest.main()
