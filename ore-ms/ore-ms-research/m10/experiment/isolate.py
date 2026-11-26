"""Run policy inference with a filesystem and TCP allowlist on Linux."""
import argparse
import ctypes
import os
from pathlib import Path
import platform

from model import inference, issue


def restrict(source, state, output):
    if platform.machine() not in ("x86_64","aarch64") or platform.system()!="Linux":
        raise RuntimeError("policy isolation requires Linux x86-64 or aarch64")
    libc = ctypes.CDLL(None,use_errno=True)

    def checked(value):
        if value < 0:
            error = ctypes.get_errno()
            raise OSError(error,os.strerror(error))
        return value

    class Ruleset(ctypes.Structure):
        _fields_ = [("fs",ctypes.c_uint64),("net",ctypes.c_uint64)]

    class PathRule(ctypes.Structure):
        _pack_ = 1
        _fields_ = [("access",ctypes.c_uint64),("fd",ctypes.c_int)]

    create, add, enforce = 444,445,446
    abi = checked(libc.syscall(create,None,0,1))
    if abi < 4:
        raise RuntimeError("policy isolation requires Landlock ABI 4 or newer")
    read_file = 1<<2
    write_file = (1<<1)|(1<<14)
    handled = Ruleset((1<<15)-1,3)
    ruleset = checked(libc.syscall(create,ctypes.byref(handled),ctypes.sizeof(handled),0))
    try:
        for path, rights in [(source,read_file|(1<<3)),(state,read_file),(output,write_file)]:
            desc = os.open(path,os.O_PATH|os.O_CLOEXEC)
            try:
                rule = PathRule(rights,desc)
                checked(libc.syscall(add,ruleset,1,ctypes.byref(rule),0))
            finally:
                os.close(desc)
        checked(libc.prctl(38,1,0,0,0))
        checked(libc.syscall(enforce,ruleset,0))
    finally:
        os.close(ruleset)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source",type=Path)
    parser.add_argument("state",type=Path)
    parser.add_argument("output",type=Path)
    parser.add_argument("--deny-read",type=Path)
    parser.add_argument("--issue",action="store_true")
    args = parser.parse_args()
    # Initialize lazy formatting imports before restricting code and filesystem access.
    import encodings.ascii
    import encodings.utf_8
    import mmap
    import numpy.lib.format
    args.output.touch()
    restrict(args.source,args.state,args.output)
    if args.deny_read:
        try:
            args.deny_read.read_bytes()
        except PermissionError:
            pass
        else:
            raise AssertionError("privileged path was readable")
    (issue if args.issue else inference)(args.source,args.state,args.output)


if __name__=="__main__":
    main()
