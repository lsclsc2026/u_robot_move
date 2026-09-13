"""Catch accidental real-control flags, malformed calibration and lock bypass."""
import copy
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'scripts' / 'teleop_runtime.py'


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(MODULE.is_file(), 'teleop runtime has not been implemented')
        spec = importlib.util.spec_from_file_location('teleop_runtime', MODULE)
        self.runtime = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.runtime)
        import yaml
        self.config = yaml.safe_load((MODULE.parents[1] / 'config' / 'teleop.yaml').read_text())

    def test_dry_run_preserves_receiver_mapping_without_enabling_sdk(self):
        cmd = self.runtime.receiver_command(self.config, '/bridge', '192.168.124.66', False, False, 2)
        self.assertNotIn('--enable-robot-command', cmd)
        self.assertNotIn('--enable-special-actions', cmd)
        self.assertEqual(cmd[cmd.index('--allowed-source') + 1], '192.168.124.66')
        self.assertEqual(cmd[cmd.index('--lx-max') + 1], '0.941')
        self.assertEqual(cmd[cmd.index('--hard-timeout-ms') + 1], '500')
        self.assertEqual(cmd[cmd.index('--duration') + 1], '2.0')

    def test_real_control_requires_explicit_selection(self):
        cmd = self.runtime.receiver_command(self.config, '/bridge', '192.168.124.66', True, False, 0)
        self.assertIn('--enable-robot-command', cmd)
        self.assertNotIn('--enable-special-actions', cmd)

    def test_special_action_requires_real_control(self):
        with self.assertRaises(ValueError):
            self.runtime.receiver_command(self.config, '/bridge', '192.168.124.66', False, True, 0)

    def test_rejects_missing_or_non_host_peer(self):
        for peer in ['', '0.0.0.0', '255.255.255.255', '::1', 'host; touch /tmp/oops']:
            with self.subTest(peer=peer), self.assertRaises(ValueError):
                self.runtime.receiver_command(self.config, '/bridge', peer, False, False, 0)

    def test_rejects_calibration_and_timeout_errors(self):
        for section, key, value in [('axes', 'lx', [-1, 1, 0]), ('receiver', 'hard_timeout_ms', 100),
                                    ('receiver', 'port', 70000), ('receiver', 'command_hz', 101),
                                    ('receiver', 'axis_deadzone', float('nan'))]:
            cfg = copy.deepcopy(self.config)
            cfg[section][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.runtime.receiver_command(cfg, '/bridge', '127.0.0.1', False, False, 1)

    def test_cross_process_lock_and_release(self):
        with tempfile.TemporaryDirectory() as temp:
            path = str(Path(temp) / 'control.lock')
            fd = self.runtime.acquire_lock(path, 'unit-test')
            probe = 'import fcntl,sys; f=open(sys.argv[1],"a+"); fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)'
            self.assertNotEqual(subprocess.run([sys.executable, '-c', probe, path], capture_output=True).returncode, 0)
            import os
            os.close(fd)
            self.assertEqual(subprocess.run([sys.executable, '-c', probe, path], capture_output=True).returncode, 0)


if __name__ == '__main__':
    unittest.main()
