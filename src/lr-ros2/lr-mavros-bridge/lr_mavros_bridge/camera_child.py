"""Tie the camera launcher's lifetime to its supervising process on Linux."""

import ctypes
import os
import signal
import sys


def main():
    parent = int(sys.argv[1])
    libc = ctypes.CDLL(None, use_errno=True)
    libc.prctl.argtypes = [ctypes.c_int] + [ctypes.c_ulong] * 4
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), 'Cannot bind camera lifetime to its supervisor')
    if os.getppid() != parent:
        return  # The supervisor exited before the parent-death signal was installed.
    os.execvp(sys.argv[2], sys.argv[2:])


if __name__ == '__main__':
    main()
