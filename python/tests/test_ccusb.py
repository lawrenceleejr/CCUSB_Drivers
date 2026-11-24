"""Tests of the Python binding against the simulator (no hardware needed).

Run from the repository root after building:
    CCUSB_LIBRARY=build/libccusb.so python -m pytest python/tests
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest  # noqa: E402

import ccusb  # noqa: E402


@pytest.fixture
def cc():
    with ccusb.open("SIM") as dev:
        yield dev


def test_read_write(cc):
    r = cc.write(3, 1, 16, 0xABCDEF)
    assert r.q and r.x
    r = cc.read(3, 1, 0)
    assert r == ccusb.Cycle(0xABCDEF, True, True)
    assert cc.naf(3, 1, 0).data == 0xABCDEF
    assert cc.read(12, 0, 0) == ccusb.Cycle(0, False, False)


def test_function_class_checked(cc):
    with pytest.raises(ccusb.CCUSBError):
        cc.write(3, 0, 0, 1)
    with pytest.raises(ccusb.CCUSBError):
        cc.read(3, 0, 16)


def test_registers_and_dataway(cc):
    assert cc.firmware == 0x8E000601
    cc.reg_write("dgg_b", 0x00200010)
    assert cc.reg_read("dgg_b") == 0x00200010
    assert set(cc.registers()) == set(ccusb.REGISTERS)
    cc.inhibit(True)
    assert cc.inhibited
    cc.inhibit(False)
    assert not cc.inhibited
    cc.write(3, 0, 16, 5)
    cc.c()
    assert cc.read(3, 0, 0).data == 0


def test_stack_encoding():
    s = ccusb.Stack().read(7, 0, 0).qstop(7, 0, 2, 8, long=True).marker(0xCAFE)
    assert s.words == [7 << 9, 0xC000 | 7 << 9 | 2, 0x8010, 8, 16, 0xCAFE]
    p = ccusb.Stack.parse("read 7 0 0\nqstop 7 0 2 8 long # c\n\nmarker 0xCAFE\n")
    assert p.words == s.words
    with pytest.raises(ValueError):
        ccusb.Stack.parse("frobnicate 1 2 3")
    with pytest.raises(ValueError):
        ccusb.Stack().write(1, 0, 0, 1)


def test_execute_and_stack_memory(cc):
    cc.control(7, 0, 25)                      # software conversion
    reply = cc.execute(ccusb.Stack().qstop(7, 0, 0, 4).marker(0x55))
    assert len(reply) == 5 and reply[-1] == 0x55
    st = ccusb.Stack().read(7, 0, 0).read(7, 7, 2)
    cc.load_stack(st, "data")
    assert cc.read_stack("data") == [2] + st.words


def test_parse_buffer():
    buf = [0x8002, 2, 0x11, 0x22, 0x2001, 0x33, 0xFFFF]
    ev = ccusb.parse_buffer(buf)
    assert [e.words for e in ev] == [[0x11, 0x22], [0x33]]
    assert ev[1].stack_id == 1


def test_daq(cc):
    cc.load_stack(ccusb.Stack().read(7, 0, 0).read(7, 7, 2).marker(0xEEEE))
    cc.daq_start()
    with pytest.raises(ccusb.CCUSBError):
        cc.read(3, 0, 0)                      # busy while taking data
    events = []
    for _ in range(4):
        buf = cc.read_buffer(500)
        if buf:
            events += ccusb.parse_buffer(buf)
    cc.daq_stop()
    while (buf := cc.read_buffer(200)) is not None:
        events += ccusb.parse_buffer(buf)
    assert len(events) > 10
    assert all(e.words[-1] == 0xEEEE and len(e.words) == 3 for e in events)
    assert all(e.words[0] < 4096 for e in events)
