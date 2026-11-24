"""Minimal use of the Python binding.

    python examples/example.py          # first CC-USB found
    python examples/example.py SIM      # simulator
"""
import sys

import ccusb

with ccusb.open(sys.argv[1] if len(sys.argv) > 1 else None) as cc:
    print(f"connected to {cc.serial}, firmware 0x{cc.firmware:08x}")
    cc.z()

    # single cycles
    cc.write(3, 0, 16, 0x1234)
    r = cc.read(3, 0, 0)
    print(f"N3 A0 F0 -> 0x{r.data:06x} Q={r.q:d} X={r.x:d}")

    # which stations answer?
    print("stations with X:", [n for n in range(1, 24) if cc.read(n, 0, 0).x])

    # take a little data with a readout stack
    stack = ccusb.Stack().read(7, 0, 0).read(7, 7, 2).marker(0xEEEE)
    cc.load_stack(stack)
    cc.daq_start()
    events = []
    for _ in range(5):
        buf = cc.read_buffer(timeout_ms=500)
        if buf:
            events += ccusb.parse_buffer(buf)
    cc.daq_stop()
    while (buf := cc.read_buffer(200)) is not None:   # drain
        events += ccusb.parse_buffer(buf)
    print(f"{len(events)} events, first: {[hex(w) for w in events[0].words]}")
