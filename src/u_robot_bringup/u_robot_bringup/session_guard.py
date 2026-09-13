"""Fail-fast, process-held lock for mutually exclusive operator launches."""

import fcntl
import os
from typing import List


_HELD_LOCK_FILES: List[int] = []


def acquire_operator_session(
    _context: object,
    *,
    session: str,
    lock_file: str = "/tmp/u_robot_operator_session.lock",
) -> list:
    """Acquire and retain the shared operator lock for this launch process."""
    lock_fd = os.open(lock_file, os.O_RDWR | os.O_CREAT, 0o600)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        holder = os.read(lock_fd, 256).decode("utf-8", errors="replace").strip()
        os.close(lock_fd)
        if not holder:
            holder = "unknown top-level launch"
        raise RuntimeError(
            f"Refusing to start '{session}': operator session lock is held by "
            f"'{holder}'. Stop that launch with Ctrl+C first."
        ) from error

    os.ftruncate(lock_fd, 0)
    os.write(lock_fd, f"{session} (pid {os.getpid()})\n".encode("utf-8"))
    os.fsync(lock_fd)
    _HELD_LOCK_FILES.append(lock_fd)
    print(f"Operator session lock acquired for '{session}' at {lock_file}", flush=True)
    return []
