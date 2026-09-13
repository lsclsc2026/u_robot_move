"""SSH arguments must preserve sender/receiver separation and explicit peer IP."""
import importlib.util
from pathlib import Path
import shlex
import unittest


class PC1EntryTests(unittest.TestCase):
    def test_build_remote_command_without_shell_injection(self):
        path = Path(__file__).resolve().parents[3] / 'scripts/pc1_ble_docker.py'
        self.assertTrue(path.is_file(), 'PC1 Docker entry is not implemented')
        spec = importlib.util.spec_from_file_location('pc1_entry', path)
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        command = entry.ssh_command('unitree-robot', '192.168.124.66', False, 'unitree-dev')
        self.assertEqual(command[0], 'ssh')
        remote = shlex.split(command[-1])
        self.assertIn('--watch-stdin', remote)
        self.assertNotIn('--real-control', remote)
        self.assertEqual(remote[remote.index('--pc1-ip') + 1], '192.168.124.66')
        with self.assertRaises(ValueError):
            entry.ssh_command('-oProxyCommand=evil', '192.168.124.66', True, 'unitree-dev')
        with self.assertRaises(ValueError):
            entry.ssh_command('unitree-robot', '192.168.124.66;evil', True, 'unitree-dev')


if __name__ == '__main__':
    unittest.main()
