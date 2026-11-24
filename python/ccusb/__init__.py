"""Python interface to the W-IE-NE-R CC-USB CAMAC crate controller.

    >>> import ccusb
    >>> with ccusb.open("SIM") as cc:          # "SIM" = built-in simulator
    ...     cc.write(3, 0, 16, 0x1234)
    ...     cc.read(3, 0, 0).data
    4660

All hardware access goes through the native ``libccusb`` library
(libusb-1.0), the same code used by the C API and the ``ccusb`` tool.
"""
from __future__ import annotations

import ctypes
import threading
from dataclasses import dataclass
from typing import Iterable, List, NamedTuple, Optional, Sequence, Union

from . import _native

__all__ = [
    "CCUSB", "CCUSBError", "Cycle", "DeviceInfo", "Event", "Stack",
    "REGISTERS", "list_devices", "open", "parse_buffer", "library_version",
]

_lib = None
_lib_lock = threading.Lock()


def _L():
    global _lib
    with _lib_lock:
        if _lib is None:
            _lib = _native.load()
    return _lib


# Controller registers (N=25, A=index)
REGISTERS = {
    "firmware": 0, "global_mode": 1, "delays": 2, "scaler_ctrl": 3,
    "led_src": 4, "output_src": 5, "device_src": 6, "dgg_a": 7, "dgg_b": 8,
    "lam_mask": 9, "lam": 10, "scaler_a": 11, "scaler_b": 12, "dgg_ext": 13,
    "usb_setup": 14,
}
READ_ONLY_REGISTERS = {0, 10, 11, 12}

ACTION_START_DAQ = 0x0001
ACTION_USB_TRIGGER = 0x0002
ACTION_CLEAR = 0x0004
ACTION_SCALER_DUMP = 0x0010

GMODE_DOUBLE_HEADER = 0x0100

ERR_TIMEOUT = -7
ERR_BUSY = -6


class CCUSBError(RuntimeError):
    """A libccusb call failed; ``code`` holds the CCUSB_ERR_* value."""

    def __init__(self, code: int, where: str = ""):
        self.code = code
        msg = _L().ccusb_strerror(code).decode()
        super().__init__(f"{where}: {msg}" if where else msg)


def _check(rc: int, where: str) -> None:
    if rc != 0:
        raise CCUSBError(rc, where)


def library_version() -> str:
    return _L().ccusb_version().decode()


class Cycle(NamedTuple):
    """Result of one CAMAC cycle."""
    data: int
    q: bool
    x: bool


@dataclass
class DeviceInfo:
    serial: str
    bus: int
    address: int
    accessible: bool


def list_devices() -> List[DeviceInfo]:
    """CC-USB controllers attached to this computer."""
    lib = _L()
    arr = (_native.DeviceInfo * 32)()
    n = lib.ccusb_list(arr, 32)
    if n < 0:
        raise CCUSBError(n, "list")
    return [
        DeviceInfo(arr[i].serial.decode(errors="replace"), arr[i].bus,
                   arr[i].address, bool(arr[i].accessible))
        for i in range(min(n, 32))
    ]


# ---------------------------------------------------------------------------
# Stacks
# ---------------------------------------------------------------------------

NAF_LONG = 0x4000
NAF_CONT = 0x8000
MODE_QSTOP = 0x0010
MODE_QSCAN = 0x0020
MODE_REPEAT = 0x0040
MODE_LAMWAIT = 0x0080


def _valid(n: int, a: int, f: int) -> None:
    if not (0 <= n <= 31 and 0 <= a <= 15 and 0 <= f <= 31):
        raise ValueError(f"invalid NAF N{n} A{a} F{f}")


def _is_write(f: int) -> bool:
    return 16 <= f <= 23


def _is_control(f: int) -> bool:
    return 8 <= f <= 15 or f >= 24


def naf_word(n: int, a: int, f: int, long: bool = False) -> int:
    _valid(n, a, f)
    return (n << 9) | (a << 5) | f | (NAF_LONG if long else 0)


class Stack:
    """Builder for CC-USB command stacks (readout lists).

    Methods return ``self`` so calls can be chained::

        Stack().read(7, 0, 0).read(7, 1, 0).marker(0xEEEE)
    """

    def __init__(self, words: Iterable[int] = ()):
        self.words: List[int] = [int(w) & 0xFFFF for w in words]

    def __len__(self) -> int:
        return len(self.words)

    def __repr__(self) -> str:
        return f"Stack({[hex(w) for w in self.words]})"

    def read(self, n, a, f, long=False, lam=False) -> "Stack":
        if _is_write(f):
            raise ValueError("F16-F23 are write functions")
        w = naf_word(n, a, f, long)
        if lam:
            self.words += [w | NAF_CONT, MODE_LAMWAIT]
        else:
            self.words.append(w)
        return self

    def write(self, n, a, f, data, long=False) -> "Stack":
        if not _is_write(f):
            raise ValueError("writes need F16-F23")
        self.words += [naf_word(n, a, f, long), data & 0xFFFF]
        if long:
            self.words.append((data >> 16) & (0xFFFF if n == 25 else 0xFF))
        return self

    def control(self, n, a, f) -> "Stack":
        if not _is_control(f):
            raise ValueError("controls need F8-F15 or F24-F31")
        self.words.append(naf_word(n, a, f))
        return self

    def _qop(self, mode, n, a, f, count, long, lam) -> "Stack":
        if _is_write(f) or _is_control(f):
            raise ValueError("Q-stop/Q-scan/repeat need a read function")
        self.words += [naf_word(n, a, f, long) | NAF_CONT,
                       mode | NAF_CONT | (MODE_LAMWAIT if lam else 0),
                       count & 0xFFFF]
        return self

    def qstop(self, n, a, f, max_count, long=False, lam=False) -> "Stack":
        return self._qop(MODE_QSTOP, n, a, f, max_count, long, lam)

    def qscan(self, n, a, f, max_count, long=False, lam=False) -> "Stack":
        return self._qop(MODE_QSCAN, n, a, f, max_count, long, lam)

    def repeat(self, n, a, f, count, long=False, lam=False) -> "Stack":
        return self._qop(MODE_REPEAT, n, a, f, count, long, lam)

    def marker(self, value) -> "Stack":
        self.words += [naf_word(0, 0, 16), value & 0xFFFF]
        return self

    @classmethod
    def parse(cls, text: str) -> "Stack":
        """Parse the stack file syntax used by the ``ccusb`` tool."""
        s = cls()
        for lineno, raw in enumerate(text.splitlines(), 1):
            line = raw.split("#", 1)[0].split()
            if not line:
                continue
            op, rest = line[0].lower(), line[1:]
            flags = {t.lower() for t in rest if not t[0].isdigit()}
            nums = [int(t, 0) for t in rest if t[0].isdigit()]
            long, lam = "long" in flags, "lam" in flags
            try:
                if op == "read":
                    s.read(*nums[:3], long=long, lam=lam)
                elif op == "write":
                    s.write(*nums[:4], long=long)
                elif op == "control":
                    s.control(*nums[:3])
                elif op in ("qstop", "qscan", "repeat"):
                    getattr(s, op)(*nums[:4], long=long, lam=lam)
                elif op == "marker":
                    s.marker(nums[0])
                elif op == "word":
                    s.words.append(nums[0] & 0xFFFF)
                else:
                    raise ValueError(f"unknown command '{op}'")
            except (TypeError, IndexError, ValueError) as exc:
                raise ValueError(f"line {lineno}: {raw.strip()!r}: {exc}") from None
        return s


# ---------------------------------------------------------------------------
# Buffers
# ---------------------------------------------------------------------------

@dataclass
class Event:
    words: List[int]
    stack_id: int = 0      # 0 = data stack, 1 = scaler stack
    continued: bool = False


def parse_buffer(words: Sequence[int], double_header: bool = False) -> List[Event]:
    """Split a CC-USB data buffer into events."""
    if not words:
        return []
    nevents = words[0] & 0x0FFF
    pos = 2 if double_header else 1
    out: List[Event] = []
    while len(out) < nevents and pos < len(words):
        hdr = words[pos]
        if hdr == 0xFFFF:
            break
        n = hdr & 0x0FFF
        body = list(words[pos + 1: pos + 1 + n])
        if len(body) < n:
            raise ValueError("truncated event in buffer")
        out.append(Event(body, 1 if hdr & 0x2000 else 0, bool(hdr & 0x1000)))
        pos += 1 + n
    return out


# ---------------------------------------------------------------------------
# Device
# ---------------------------------------------------------------------------

class CCUSB:
    """An open CC-USB controller (or the simulator when serial == "SIM")."""

    def __init__(self, serial: Optional[str] = None, timeout_ms: int = 2000):
        lib = _L()
        self._lib = lib
        self._h = ctypes.c_void_p()
        _check(lib.ccusb_open(ctypes.byref(self._h),
                              serial.encode() if serial else None), "open")
        lib.ccusb_set_timeout(self._h, timeout_ms)

    # -- lifetime --------------------------------------------------------
    def close(self) -> None:
        if self._h:
            self._lib.ccusb_close(self._h)
            self._h = ctypes.c_void_p()

    def __enter__(self) -> "CCUSB":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    @property
    def serial(self) -> str:
        return self._lib.ccusb_serial(self._h).decode()

    @property
    def is_simulator(self) -> bool:
        return bool(self._lib.ccusb_is_simulator(self._h))

    def set_trace(self, on: bool = True) -> None:
        self._lib.ccusb_set_trace(self._h, int(on))

    def set_timeout(self, ms: int) -> None:
        self._lib.ccusb_set_timeout(self._h, ms)

    # -- CAMAC cycles ----------------------------------------------------
    def read(self, n: int, a: int, f: int = 0) -> Cycle:
        d, q, x = ctypes.c_uint32(), ctypes.c_int(), ctypes.c_int()
        _check(self._lib.ccusb_read(self._h, n, a, f, ctypes.byref(d),
                                    ctypes.byref(q), ctypes.byref(x)),
               f"read N{n} A{a} F{f}")
        return Cycle(d.value, bool(q.value), bool(x.value))

    def write(self, n: int, a: int, f: int, data: int) -> Cycle:
        q, x = ctypes.c_int(), ctypes.c_int()
        _check(self._lib.ccusb_write(self._h, n, a, f, data & 0xFFFFFFFF,
                                     ctypes.byref(q), ctypes.byref(x)),
               f"write N{n} A{a} F{f}")
        return Cycle(data, bool(q.value), bool(x.value))

    def control(self, n: int, a: int, f: int) -> Cycle:
        q, x = ctypes.c_int(), ctypes.c_int()
        _check(self._lib.ccusb_control(self._h, n, a, f, ctypes.byref(q),
                                       ctypes.byref(x)),
               f"control N{n} A{a} F{f}")
        return Cycle(0, bool(q.value), bool(x.value))

    def naf(self, n: int, a: int, f: int, data: int = 0) -> Cycle:
        """Any NAF: read, write or control depending on F."""
        if _is_write(f):
            return self.write(n, a, f, data)
        if _is_control(f):
            return self.control(n, a, f)
        return self.read(n, a, f)

    def z(self) -> None:
        _check(self._lib.ccusb_z(self._h), "Z")

    def c(self) -> None:
        _check(self._lib.ccusb_c(self._h), "C")

    def inhibit(self, on: bool = True) -> None:
        _check(self._lib.ccusb_inhibit(self._h, int(on)), "inhibit")

    @property
    def inhibited(self) -> bool:
        v = ctypes.c_int()
        _check(self._lib.ccusb_is_inhibited(self._h, ctypes.byref(v)), "inhibit")
        return bool(v.value)

    # -- registers --------------------------------------------------------
    @staticmethod
    def _reg(reg: Union[int, str]) -> int:
        if isinstance(reg, str):
            if reg not in REGISTERS:
                raise KeyError(f"unknown register {reg!r}")
            return REGISTERS[reg]
        return int(reg)

    def reg_read(self, reg: Union[int, str]) -> int:
        v = ctypes.c_uint32()
        _check(self._lib.ccusb_reg_read(self._h, self._reg(reg), ctypes.byref(v)),
               f"read register {reg}")
        return v.value

    def reg_write(self, reg: Union[int, str], value: int) -> None:
        _check(self._lib.ccusb_reg_write(self._h, self._reg(reg), value & 0xFFFFFFFF),
               f"write register {reg}")

    def registers(self) -> dict:
        return {name: self.reg_read(idx) for name, idx in REGISTERS.items()}

    @property
    def firmware(self) -> int:
        return self.reg_read(0)

    @property
    def lams(self) -> int:
        v = ctypes.c_uint32()
        _check(self._lib.ccusb_read_lams(self._h, ctypes.byref(v)), "LAM")
        return v.value

    @property
    def scalers(self):
        a, b = ctypes.c_uint32(), ctypes.c_uint32()
        _check(self._lib.ccusb_read_scalers(self._h, ctypes.byref(a), ctypes.byref(b)),
               "scalers")
        return a.value, b.value

    def action(self, value: int) -> None:
        _check(self._lib.ccusb_write_action(self._h, value), "action register")

    # -- stacks -------------------------------------------------------------
    @staticmethod
    def _words(stack: Union[Stack, Sequence[int]]):
        words = stack.words if isinstance(stack, Stack) else list(stack)
        return (ctypes.c_uint16 * max(1, len(words)))(*words), len(words)

    def execute(self, stack: Union[Stack, Sequence[int]], max_reply: int = 16384) -> List[int]:
        """Execute a stack immediately; returns the reply words."""
        arr, n = self._words(stack)
        reply = (ctypes.c_uint16 * max_reply)()
        got = ctypes.c_size_t()
        _check(self._lib.ccusb_execute(self._h, arr, n, reply, max_reply,
                                       ctypes.byref(got)), "execute")
        return list(reply[: got.value])

    def load_stack(self, stack: Union[Stack, Sequence[int]], which: str = "data") -> None:
        arr, n = self._words(stack)
        _check(self._lib.ccusb_stack_load(self._h, 1 if which == "scaler" else 0, arr, n),
               f"load {which} stack")

    def read_stack(self, which: str = "data") -> List[int]:
        buf = (ctypes.c_uint16 * 1024)()
        got = ctypes.c_size_t()
        _check(self._lib.ccusb_stack_read(self._h, 1 if which == "scaler" else 0,
                                          buf, 1024, ctypes.byref(got)),
               f"read {which} stack")
        return list(buf[: got.value])

    # -- data acquisition --------------------------------------------------
    def daq_start(self) -> None:
        _check(self._lib.ccusb_daq_start(self._h), "start DAQ")

    def daq_stop(self) -> None:
        _check(self._lib.ccusb_daq_stop(self._h), "stop DAQ")

    @property
    def daq_running(self) -> bool:
        return bool(self._lib.ccusb_daq_running(self._h))

    def read_buffer(self, timeout_ms: int = 1000) -> Optional[List[int]]:
        """Next data buffer as a list of 16-bit words, or None on timeout."""
        buf = (ctypes.c_uint16 * 32768)()
        got = ctypes.c_size_t()
        rc = self._lib.ccusb_read_buffer(self._h, buf, 32768, ctypes.byref(got), timeout_ms)
        if rc == ERR_TIMEOUT:
            return None
        _check(rc, "read buffer")
        return list(buf[: got.value])

    def flush(self) -> int:
        return self._lib.ccusb_flush(self._h)


def open(serial: Optional[str] = None, timeout_ms: int = 2000) -> CCUSB:  # noqa: A001
    """Open a CC-USB by serial number (None = first one, "SIM" = simulator)."""
    return CCUSB(serial, timeout_ms)
