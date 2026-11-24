"""Locate and load the native libccusb library and declare its prototypes."""
from __future__ import annotations

import ctypes
import ctypes.util
import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent


def _candidates():
    env = os.environ.get("CCUSB_LIBRARY")
    if env:
        yield Path(env)
    if sys.platform.startswith("win"):
        names = ["ccusb.dll", "libccusb.dll"]
    elif sys.platform == "darwin":
        names = ["libccusb.dylib", "libccusb.1.dylib"]
    else:
        names = ["libccusb.so", "libccusb.so.1"]
    # next to the package (pip/manual install), then a source-tree build
    roots = [_HERE, _HERE.parent.parent / "build", _HERE.parent.parent / "build" / "Release"]
    for root in roots:
        for n in names:
            yield root / n
    found = ctypes.util.find_library("ccusb")
    if found:
        yield Path(found)


def load() -> ctypes.CDLL:
    errors = []
    for path in _candidates():
        if path.is_absolute() and not path.exists():
            continue
        try:
            if sys.platform.startswith("win") and path.is_absolute():
                os.add_dll_directory(str(path.parent))
            lib = ctypes.CDLL(str(path))
        except OSError as exc:
            errors.append(f"{path}: {exc}")
            continue
        _declare(lib)
        return lib
    raise OSError(
        "libccusb not found. Build it with CMake (see README) and/or set "
        "CCUSB_LIBRARY to the path of libccusb.so / ccusb.dll.\n" + "\n".join(errors)
    )


class DeviceInfo(ctypes.Structure):
    _fields_ = [
        ("serial", ctypes.c_char * 32),
        ("vendor_id", ctypes.c_uint16),
        ("product_id", ctypes.c_uint16),
        ("bus", ctypes.c_uint8),
        ("address", ctypes.c_uint8),
        ("accessible", ctypes.c_int),
    ]


def _declare(lib: ctypes.CDLL) -> None:
    P = ctypes.c_void_p
    I = ctypes.c_int
    U32 = ctypes.c_uint32
    U16 = ctypes.c_uint16
    SZ = ctypes.c_size_t
    pI = ctypes.POINTER(I)
    pU32 = ctypes.POINTER(U32)
    pU16 = ctypes.POINTER(U16)
    pSZ = ctypes.POINTER(SZ)

    def sig(name, res, *args):
        fn = getattr(lib, name)
        fn.restype = res
        fn.argtypes = list(args)

    sig("ccusb_version", ctypes.c_char_p)
    sig("ccusb_strerror", ctypes.c_char_p, I)
    sig("ccusb_list", I, ctypes.POINTER(DeviceInfo), I)
    sig("ccusb_open", I, ctypes.POINTER(P), ctypes.c_char_p)
    sig("ccusb_close", None, P)
    sig("ccusb_serial", ctypes.c_char_p, P)
    sig("ccusb_is_simulator", I, P)
    sig("ccusb_set_timeout", None, P, ctypes.c_uint)
    sig("ccusb_set_trace", None, P, I)
    sig("ccusb_read", I, P, I, I, I, pU32, pI, pI)
    sig("ccusb_write", I, P, I, I, I, U32, pI, pI)
    sig("ccusb_control", I, P, I, I, I, pI, pI)
    sig("ccusb_z", I, P)
    sig("ccusb_c", I, P)
    sig("ccusb_inhibit", I, P, I)
    sig("ccusb_is_inhibited", I, P, pI)
    sig("ccusb_reg_read", I, P, I, pU32)
    sig("ccusb_reg_write", I, P, I, U32)
    sig("ccusb_read_lams", I, P, pU32)
    sig("ccusb_read_scalers", I, P, pU32, pU32)
    sig("ccusb_write_action", I, P, U16)
    sig("ccusb_execute", I, P, pU16, SZ, pU16, SZ, pSZ)
    sig("ccusb_stack_load", I, P, I, pU16, SZ)
    sig("ccusb_stack_read", I, P, I, pU16, SZ, pSZ)
    sig("ccusb_daq_start", I, P)
    sig("ccusb_daq_stop", I, P)
    sig("ccusb_daq_running", I, P)
    sig("ccusb_read_buffer", I, P, pU16, SZ, pSZ, ctypes.c_uint)
    sig("ccusb_flush", I, P)
