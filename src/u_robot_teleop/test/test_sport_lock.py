"""Verify the SDK backend uses a lock compatible with the Python supervisor."""
from pathlib import Path
import fcntl
import subprocess
import tempfile
import unittest


class SportLockTests(unittest.TestCase):
    def test_native_lock_contends_with_python_and_releases(self):
        include = Path(__file__).resolve().parents[2] / 'u_robot_a2_driver' / 'include'
        self.assertTrue((include / 'u_robot_a2_driver/sport_control_lock.hpp').is_file(),
                        'SDK backend shared control lock is not implemented')
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'probe.cpp'
            executable = Path(temp) / 'probe'
            source.write_text('''#include "u_robot_a2_driver/sport_control_lock.hpp"
int main(int argc, char **argv) {
  try { u_robot_a2_driver::SportControlLock lock(argv[1], "probe"); return 0; }
  catch (...) { return 1; }
}
''')
            subprocess.run(['g++', '-std=c++17', '-I', str(include), str(source), '-o', str(executable)], check=True)
            path = Path(temp) / 'shared.lock'
            with path.open('w') as stream:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.assertEqual(subprocess.run([str(executable), str(path)]).returncode, 1)
            self.assertEqual(subprocess.run([str(executable), str(path)]).returncode, 0)
            with path.open('w') as stream:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)


if __name__ == '__main__':
    unittest.main()
