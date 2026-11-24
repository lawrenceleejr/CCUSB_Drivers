"""ccusb-gui: a small Tk application for the CC-USB CAMAC controller.

    python -m ccusb.gui            # pick a controller (or the simulator)
    python -m ccusb.gui --sim      # connect to the simulator right away

Tabs
  Crate      single NAF cycles, Z / C / Inhibit, a station map, history
  Monitor    poll channels (e.g. scalers) and show them as sparklines
  Registers  view and edit the controller registers (N25)
  DAQ        load a stack, take data, live event rate and histogram

Graphics follow Tufte: no chart frames or grid, data drawn in ink, labels
placed directly on the data, sparklines with min / max / last marked.
"""
from __future__ import annotations

import argparse
import collections
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter import font as tkfont

import ccusb

PAPER = "#fffff8"
INK = "#111111"
MUTED = "#7a7a72"
FAINT = "#dcdcd2"
ACCENT = "#a3201b"        # used sparingly: last value, errors

REG_DOC = {
    "firmware": "firmware id (read only)",
    "global_mode": "buffer length [3:0], mixed 0x20, double header 0x100",
    "delays": "trigger delay [7:0] us, LAM timeout [15:8] us",
    "scaler_ctrl": "events between scaler reads [15:0], period [23:16] x 0.5 s",
    "led_src": "LED sources: red [7:0], green [15:8], yellow [23:16]",
    "output_src": "NIM outputs: O1 [7:0], O2 [15:8], O3 [23:16]",
    "device_src": "scaler A [7:0], scaler B [15:8], DGG A [23:16], DGG B [31:24]",
    "dgg_a": "gate & delay A: width [31:16], delay [15:0]",
    "dgg_b": "gate & delay B: width [31:16], delay [15:0]",
    "lam_mask": "stations whose LAM triggers the data stack",
    "lam": "LAM lines (read only), inhibit = bit 24",
    "scaler_a": "internal scaler A (read only)",
    "scaler_b": "internal scaler B (read only)",
    "dgg_ext": "coarse delays: A [15:0], B [31:16]",
    "usb_setup": "USB bulk transfer setup",
}

DEFAULT_STACK = """# Data stack: executed by the CC-USB on every trigger.
# read N A F [long] [lam]   write N A F DATA   control N A F
# qstop|qscan|repeat N A F MAX   marker VALUE
#
# The simulator has an 8-channel ADC in station 7:
read 7 0 0
read 7 1 0
read 7 2 0
read 7 3 0
read 7 4 0
read 7 5 0
read 7 6 0
read 7 7 2      # F2 on the last channel clears the ADC
marker 0xEEEE
"""


def parse_int(text: str) -> int:
    return int(text.strip().replace("_", ""), 0)


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

def draw_sparkline(cv: tk.Canvas, values, width: int, height: int, fmt="{:.0f}"):
    """Word-sized line chart: thin ink line, min/max dots, red last value."""
    cv.delete("all")
    pts = [v for v in values if v is not None]
    if len(pts) < 2:
        cv.create_text(2, height / 2, text="collecting...", anchor="w",
                       fill=MUTED, font=("TkDefaultFont", 8))
        return
    lo, hi = min(pts), max(pts)
    span = (hi - lo) or 1.0
    pad = 3
    n = len(pts)

    def xy(i, v):
        x = pad + (width - 2 * pad) * i / (n - 1)
        y = height - pad - (height - 2 * pad) * (v - lo) / span
        return x, y

    coords = []
    for i, v in enumerate(pts):
        coords.extend(xy(i, v))
    cv.create_line(*coords, fill=INK, width=1)
    imin, imax = pts.index(lo), pts.index(hi)
    for i, v in ((imin, lo), (imax, hi)):
        x, y = xy(i, v)
        cv.create_oval(x - 1.5, y - 1.5, x + 1.5, y + 1.5, fill=MUTED, outline="")
    x, y = xy(n - 1, pts[-1])
    cv.create_oval(x - 2, y - 2, x + 2, y + 2, fill=ACCENT, outline="")


def draw_histogram(cv: tk.Canvas, counts, lo: float, hi: float, label: str):
    """Histogram with a range-frame x axis and no box, grid or y axis line.

    Bars are drawn as narrow ink columns; the y scale is given by direct
    labels on the tallest bar only.
    """
    cv.delete("all")
    w = int(cv.winfo_width()) or 480
    h = int(cv.winfo_height()) or 220
    left, right, top, bottom = 12, 12, 22, 34
    pw, ph = w - left - right, h - top - bottom
    total = sum(counts)
    small = ("TkDefaultFont", 8)
    if total == 0:
        cv.create_text(w / 2, h / 2, text="no events yet", fill=MUTED)
        return
    peak = max(counts)
    nb = len(counts)
    bw = pw / nb
    for i, c in enumerate(counts):
        if not c:
            continue
        x0 = left + i * bw
        y0 = top + ph * (1 - c / peak)
        cv.create_rectangle(x0 + bw * 0.12, y0, x0 + bw * 0.88, top + ph,
                            fill=INK, outline="")
    # direct label on the tallest bar
    ip = counts.index(peak)
    cv.create_text(left + (ip + 0.5) * bw, top + ph * 0 - 4, text=f"{peak}",
                   anchor="s", fill=MUTED, font=small)
    # range frame: axis spans only the occupied bins
    first = next(i for i, c in enumerate(counts) if c)
    last = nb - 1 - next(i for i, c in enumerate(reversed(counts)) if c)
    x_first = left + first * bw
    x_last = left + (last + 1) * bw
    ybase = top + ph + 4
    cv.create_line(x_first, ybase, x_last, ybase, fill=MUTED)
    step = (hi - lo) / nb
    for x, v in ((x_first, lo + first * step), (x_last, lo + (last + 1) * step)):
        cv.create_line(x, ybase, x, ybase + 3, fill=MUTED)
        cv.create_text(x, ybase + 5, text=f"{v:.0f}", anchor="n", fill=MUTED, font=small)
    cv.create_text(w - right, h - 2, text=label, anchor="se", fill=MUTED, font=small)
    cv.create_text(left, 4, text=f"{total} events", anchor="nw", fill=MUTED, font=small)


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------

class App:
    def __init__(self, root: tk.Tk, auto_serial=None):
        self.root = root
        self.cc: ccusb.CCUSB | None = None
        self.daq_thread: threading.Thread | None = None
        self.daq_queue: queue.Queue = queue.Queue()
        self.daq_stop_flag = threading.Event()
        self.monitor_rows = []
        self._monitor_job = None

        root.title("CC-USB")
        root.configure(bg=PAPER)
        root.minsize(860, 600)
        self._style()
        self._build_header()
        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=True, padx=12, pady=(4, 0))
        self.nb = nb
        self._build_crate(nb)
        self._build_monitor(nb)
        self._build_registers(nb)
        self._build_daq(nb)
        self.status = tk.StringVar(value="Not connected.")
        self.status_label = tk.Label(root, textvariable=self.status, anchor="w",
                                     bg=PAPER, fg=MUTED, padx=14, pady=6)
        self.status_label.pack(fill="x")
        self.refresh_devices()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        if auto_serial:
            self.device_var.set(auto_serial)
            root.after(50, self.connect)

    # -- appearance -------------------------------------------------------
    def _style(self):
        st = ttk.Style(self.root)
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        for el in ("TFrame", "TLabel", "TCheckbutton", "TRadiobutton", "TLabelframe",
                   "TLabelframe.Label", "TNotebook"):
            st.configure(el, background=PAPER, foreground=INK)
        st.configure("TNotebook", borderwidth=0)
        st.configure("TNotebook.Tab", background=PAPER, foreground=MUTED,
                     padding=(14, 5), borderwidth=0)
        st.map("TNotebook.Tab", background=[("selected", PAPER)],
               foreground=[("selected", INK)])
        st.configure("TButton", background=PAPER, foreground=INK, borderwidth=1,
                     relief="flat", padding=(10, 3))
        st.map("TButton", background=[("active", FAINT)])
        st.configure("Muted.TLabel", foreground=MUTED)
        st.configure("Head.TLabel", font=("TkDefaultFont", 10, "bold"))
        st.configure("TLabelframe", bordercolor=FAINT, relief="flat")
        self.mono = tkfont.nametofont("TkFixedFont")

    def _build_header(self):
        f = ttk.Frame(self.root)
        f.pack(fill="x", padx=14, pady=(10, 4))
        ttk.Label(f, text="Controller").pack(side="left")
        self.device_var = tk.StringVar()
        self.device_box = ttk.Combobox(f, textvariable=self.device_var, width=14)
        self.device_box.pack(side="left", padx=6)
        ttk.Button(f, text="Refresh", command=self.refresh_devices).pack(side="left")
        self.connect_btn = ttk.Button(f, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=6)
        self.info_var = tk.StringVar(value="")
        ttk.Label(f, textvariable=self.info_var, style="Muted.TLabel").pack(side="left", padx=10)

    # -- crate tab ----------------------------------------------------------
    def _build_crate(self, nb):
        tab = ttk.Frame(nb, padding=12)
        nb.add(tab, text="Crate")
        left = ttk.Frame(tab)
        left.pack(side="left", fill="y")

        ttk.Label(left, text="Single cycle", style="Head.TLabel").grid(row=0, column=0, columnspan=8, sticky="w")
        self.n_var, self.a_var, self.f_var = tk.StringVar(value="3"), tk.StringVar(value="0"), tk.StringVar(value="0")
        self.data_var = tk.StringVar(value="0x0")
        for col, (lbl, var, hi) in enumerate((("N", self.n_var, 31), ("A", self.a_var, 15), ("F", self.f_var, 31))):
            ttk.Label(left, text=lbl).grid(row=1, column=2 * col, sticky="e", padx=(0, 3), pady=6)
            sp = ttk.Spinbox(left, from_=0, to=hi, textvariable=var, width=4)
            sp.grid(row=1, column=2 * col + 1, sticky="w", padx=(0, 10))
            sp.bind("<Return>", lambda e: self.do_naf())
        ttk.Label(left, text="Data").grid(row=2, column=0, sticky="e", padx=(0, 3))
        de = ttk.Entry(left, textvariable=self.data_var, width=12, font=self.mono)
        de.grid(row=2, column=1, columnspan=3, sticky="w")
        de.bind("<Return>", lambda e: self.do_naf())
        ttk.Button(left, text="Execute", command=self.do_naf).grid(row=2, column=4, columnspan=2, sticky="w")
        ttk.Label(left, text="F0-7 read · F8-15 control · F16-23 write · F24-31 control",
                  style="Muted.TLabel").grid(row=3, column=0, columnspan=8, sticky="w", pady=(4, 0))
        self.result_var = tk.StringVar(value="")
        tk.Label(left, textvariable=self.result_var, bg=PAPER, fg=INK, font=self.mono,
                 anchor="w", justify="left").grid(row=4, column=0, columnspan=8, sticky="w", pady=(8, 14))

        ttk.Label(left, text="Dataway", style="Head.TLabel").grid(row=5, column=0, columnspan=8, sticky="w")
        dw = ttk.Frame(left)
        dw.grid(row=6, column=0, columnspan=8, sticky="w", pady=6)
        ttk.Button(dw, text="Z  initialise", command=lambda: self.dataway("z")).pack(side="left")
        ttk.Button(dw, text="C  clear", command=lambda: self.dataway("c")).pack(side="left", padx=6)
        self.inhibit_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(dw, text="Inhibit", variable=self.inhibit_var,
                        command=self.set_inhibit).pack(side="left", padx=6)

        ttk.Label(left, text="Stations", style="Head.TLabel").grid(row=7, column=0, columnspan=8, sticky="w", pady=(14, 0))
        self.crate_cv = tk.Canvas(left, width=23 * 17 + 20, height=74, bg=PAPER, highlightthickness=0)
        self.crate_cv.grid(row=8, column=0, columnspan=8, sticky="w", pady=4)
        ttk.Button(left, text="Scan", command=self.scan).grid(row=9, column=0, columnspan=2, sticky="w")
        ttk.Label(left, text="bar = module answers X to A0 F0 · red dot = LAM",
                  style="Muted.TLabel").grid(row=9, column=2, columnspan=6, sticky="w")

        right = ttk.Frame(tab)
        right.pack(side="left", fill="both", expand=True, padx=(24, 0))
        ttk.Label(right, text="History", style="Head.TLabel").pack(anchor="w")
        self.history = tk.Text(right, height=20, width=48, bg=PAPER, fg=INK, font=self.mono,
                               relief="flat", highlightthickness=0, wrap="none")
        self.history.pack(fill="both", expand=True, pady=4)
        self.history.tag_configure("muted", foreground=MUTED)
        self.history.tag_configure("err", foreground=ACCENT)
        self.history.configure(state="disabled")
        self.draw_crate(None, 0)

    def log(self, text, tag=None):
        self.history.configure(state="normal")
        self.history.insert("1.0", time.strftime("%H:%M:%S  "), ("muted",))
        self.history.insert("1.9", text + "\n", (tag,) if tag else ())
        if int(self.history.index("end-1c").split(".")[0]) > 500:
            self.history.delete("500.0", "end")
        self.history.configure(state="disabled")

    def draw_crate(self, x_mask, lam_mask):
        cv = self.crate_cv
        cv.delete("all")
        for n in range(1, 24):
            x = 10 + (n - 1) * 17 + 8
            has = x_mask is not None and (x_mask >> (n - 1)) & 1
            if x_mask is None:
                cv.create_line(x, 40, x, 42, fill=FAINT)
            elif has:
                cv.create_rectangle(x - 3, 16, x + 3, 44, fill=INK, outline="")
            else:
                cv.create_line(x, 42, x, 44, fill=MUTED)
            if (lam_mask >> (n - 1)) & 1:
                cv.create_oval(x - 2.5, 6, x + 2.5, 11, fill=ACCENT, outline="")
            if n == 1 or n % 5 == 0 or has:
                cv.create_text(x, 50, text=str(n), anchor="n",
                               fill=INK if has else MUTED, font=("TkDefaultFont", 8))

    # -- monitor tab -------------------------------------------------------
    def _build_monitor(self, nb):
        tab = ttk.Frame(nb, padding=12)
        nb.add(tab, text="Monitor")
        top = ttk.Frame(tab)
        top.pack(fill="x")
        ttk.Label(top, text="Add channel  N").pack(side="left")
        self.mn, self.ma, self.mf = tk.StringVar(value="5"), tk.StringVar(value="0"), tk.StringVar(value="0")
        for var, lbl in ((self.mn, "A"), (self.ma, "F"), (self.mf, None)):
            ttk.Entry(top, textvariable=var, width=4).pack(side="left", padx=(3, 6))
            if lbl:
                ttk.Label(top, text=lbl).pack(side="left")
        self.mode_var = tk.StringVar(value="rate")
        ttk.Radiobutton(top, text="rate /s", value="rate", variable=self.mode_var).pack(side="left", padx=(8, 0))
        ttk.Radiobutton(top, text="value", value="value", variable=self.mode_var).pack(side="left")
        ttk.Button(top, text="Add", command=self.monitor_add).pack(side="left", padx=8)
        ttk.Label(top, text="every").pack(side="left", padx=(16, 3))
        self.period_var = tk.StringVar(value="500")
        ttk.Spinbox(top, from_=100, to=10000, increment=100, textvariable=self.period_var, width=6).pack(side="left")
        ttk.Label(top, text="ms").pack(side="left", padx=3)
        self.monitor_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="polling", variable=self.monitor_on).pack(side="left", padx=12)

        self.mon_table = ttk.Frame(tab)
        self.mon_table.pack(fill="both", expand=True, pady=12)
        hdr = ("channel", "last 2 minutes", "now", "min", "max", "")
        for c, t in enumerate(hdr):
            ttk.Label(self.mon_table, text=t, style="Muted.TLabel").grid(row=0, column=c, sticky="w", padx=6)
        ttk.Label(tab, text="red dot = latest reading, grey dots = minimum and maximum",
                  style="Muted.TLabel").pack(anchor="w")

    def monitor_add(self, n=None, a=None, f=None, mode=None):
        try:
            n = parse_int(self.mn.get()) if n is None else n
            a = parse_int(self.ma.get()) if a is None else a
            f = parse_int(self.mf.get()) if f is None else f
        except ValueError:
            self.error("N, A and F must be numbers")
            return
        mode = mode or self.mode_var.get()
        row = len(self.monitor_rows) + 1
        r = {"naf": (n, a, f), "mode": mode, "hist": collections.deque(maxlen=240),
             "last": None, "t": None}
        unit = "/s" if mode == "rate" else ""
        r["label"] = ttk.Label(self.mon_table, text=f"N{n} A{a} F{f}{'  rate' if mode == 'rate' else ''}",
                               font=self.mono)
        r["cv"] = tk.Canvas(self.mon_table, width=260, height=26, bg=PAPER, highlightthickness=0)
        r["now"] = tk.Label(self.mon_table, text="", fg=ACCENT, bg=PAPER, font=self.mono, width=11, anchor="e")
        r["min"] = tk.Label(self.mon_table, text="", fg=MUTED, bg=PAPER, font=self.mono, width=10, anchor="e")
        r["max"] = tk.Label(self.mon_table, text="", fg=MUTED, bg=PAPER, font=self.mono, width=10, anchor="e")
        r["unit"] = unit
        r["rm"] = ttk.Button(self.mon_table, text="remove", command=lambda rr=r: self.monitor_remove(rr))
        for c, key in enumerate(("label", "cv", "now", "min", "max", "rm")):
            r[key].grid(row=row, column=c, sticky="w", padx=6, pady=2)
        self.monitor_rows.append(r)
        self._schedule_monitor()

    def monitor_remove(self, r):
        for key in ("label", "cv", "now", "min", "max", "rm"):
            r[key].destroy()
        self.monitor_rows.remove(r)

    def _schedule_monitor(self):
        if self._monitor_job is None:
            self._monitor_job = self.root.after(200, self._monitor_tick)

    def _monitor_tick(self):
        self._monitor_job = None
        if not self.monitor_rows:
            return
        try:
            period = max(100, parse_int(self.period_var.get()))
        except ValueError:
            period = 500
        if self.cc and self.monitor_on.get() and not self.cc.daq_running:
            now = time.monotonic()
            for r in self.monitor_rows:
                try:
                    v = self.cc.naf(*r["naf"]).data
                except ccusb.CCUSBError as exc:
                    self.error(str(exc))
                    break
                if r["mode"] == "rate":
                    if r["last"] is not None and now > r["t"]:
                        delta = (v - r["last"]) % (1 << 24)
                        r["hist"].append(delta / (now - r["t"]))
                    r["last"], r["t"] = v, now
                else:
                    r["hist"].append(float(v))
                self._draw_row(r)
        self._monitor_job = self.root.after(period, self._monitor_tick)

    def _draw_row(self, r):
        h = list(r["hist"])
        draw_sparkline(r["cv"], h, 260, 26)
        if h:
            r["now"].configure(text=f"{h[-1]:,.0f}{r['unit']}")
            r["min"].configure(text=f"{min(h):,.0f}")
            r["max"].configure(text=f"{max(h):,.0f}")

    # -- registers tab -----------------------------------------------------
    def _build_registers(self, nb):
        tab = ttk.Frame(nb, padding=12)
        nb.add(tab, text="Registers")
        bar = ttk.Frame(tab)
        bar.pack(fill="x")
        ttk.Button(bar, text="Read all", command=self.regs_read).pack(side="left")
        ttk.Button(bar, text="Write changed", command=self.regs_write).pack(side="left", padx=6)
        ttk.Label(bar, text="N25 · F0 read, F16 write · values in hex", style="Muted.TLabel").pack(side="left", padx=10)
        grid = ttk.Frame(tab)
        grid.pack(fill="both", expand=True, pady=10)
        for c, t in enumerate(("A", "register", "value", "decimal", "meaning")):
            ttk.Label(grid, text=t, style="Muted.TLabel").grid(row=0, column=c, sticky="w", padx=6, pady=(0, 4))
        self.reg_vars, self.reg_dec, self.reg_loaded = {}, {}, {}
        for i, (name, idx) in enumerate(ccusb.REGISTERS.items(), 1):
            ttk.Label(grid, text=str(idx)).grid(row=i, column=0, sticky="e", padx=6)
            ttk.Label(grid, text=name, font=self.mono).grid(row=i, column=1, sticky="w", padx=6)
            var = tk.StringVar(value="")
            if idx in ccusb.READ_ONLY_REGISTERS:
                w = tk.Label(grid, textvariable=var, bg=PAPER, fg=INK, font=self.mono, width=12, anchor="w")
            else:
                w = ttk.Entry(grid, textvariable=var, width=12, font=self.mono)
            w.grid(row=i, column=2, sticky="w", padx=6, pady=1)
            dec = tk.StringVar(value="")
            tk.Label(grid, textvariable=dec, bg=PAPER, fg=MUTED, font=self.mono, width=11, anchor="e").grid(
                row=i, column=3, sticky="e", padx=6)
            ttk.Label(grid, text=REG_DOC.get(name, ""), style="Muted.TLabel").grid(row=i, column=4, sticky="w", padx=6)
            self.reg_vars[idx], self.reg_dec[idx] = var, dec

    def regs_read(self):
        if not self.require():
            return
        try:
            for idx, var in self.reg_vars.items():
                v = self.cc.reg_read(idx)
                var.set(f"0x{v:08x}")
                self.reg_dec[idx].set(f"{v}")
                self.reg_loaded[idx] = v
            self.status.set("Registers read.")
        except ccusb.CCUSBError as exc:
            self.error(str(exc))

    def regs_write(self):
        if not self.require():
            return
        written = []
        try:
            for idx, var in self.reg_vars.items():
                if idx in ccusb.READ_ONLY_REGISTERS or not var.get().strip():
                    continue
                v = parse_int(var.get())
                if self.reg_loaded.get(idx) != v:
                    self.cc.reg_write(idx, v)
                    written.append(idx)
        except ValueError as exc:
            self.error(f"bad register value: {exc}")
            return
        except ccusb.CCUSBError as exc:
            self.error(str(exc))
            return
        self.regs_read()
        self.status.set(f"Wrote {len(written)} register(s)." if written else "Nothing changed.")

    # -- DAQ tab ------------------------------------------------------------
    def _build_daq(self, nb):
        tab = ttk.Frame(nb, padding=12)
        nb.add(tab, text="DAQ")
        left = ttk.Frame(tab)
        left.pack(side="left", fill="y")
        ttk.Label(left, text="Data stack", style="Head.TLabel").pack(anchor="w")
        self.stack_text = tk.Text(left, width=44, height=22, bg=PAPER, fg=INK, font=self.mono,
                                  relief="solid", borderwidth=1, highlightthickness=0,
                                  insertbackground=INK)
        self.stack_text.configure(highlightbackground=FAINT)
        self.stack_text.insert("1.0", DEFAULT_STACK)
        self.stack_text.pack(pady=4)
        b = ttk.Frame(left)
        b.pack(fill="x")
        ttk.Button(b, text="Open...", command=self.stack_open).pack(side="left")
        ttk.Button(b, text="Save...", command=self.stack_save).pack(side="left", padx=4)
        ttk.Button(b, text="Run once", command=self.stack_run_once).pack(side="left", padx=4)
        self.save_raw = tk.StringVar(value="")
        r = ttk.Frame(left)
        r.pack(fill="x", pady=(10, 0))
        ttk.Button(r, text="Raw output...", command=self.choose_raw).pack(side="left")
        ttk.Label(r, textvariable=self.save_raw, style="Muted.TLabel").pack(side="left", padx=6)

        right = ttk.Frame(tab)
        right.pack(side="left", fill="both", expand=True, padx=(24, 0))
        ctl = ttk.Frame(right)
        ctl.pack(fill="x")
        self.daq_btn = ttk.Button(ctl, text="Start", command=self.daq_toggle)
        self.daq_btn.pack(side="left")
        ttk.Label(ctl, text="histogram word").pack(side="left", padx=(16, 3))
        self.hist_word = tk.StringVar(value="0")
        ttk.Spinbox(ctl, from_=0, to=255, textvariable=self.hist_word, width=4,
                    command=self.hist_reset).pack(side="left")
        ttk.Label(ctl, text="range 0 –").pack(side="left", padx=(10, 3))
        self.hist_max = tk.StringVar(value="4096")
        ttk.Entry(ctl, textvariable=self.hist_max, width=7).pack(side="left")
        ttk.Button(ctl, text="Clear", command=self.hist_reset).pack(side="left", padx=8)

        self.daq_stats = tk.StringVar(value="")
        tk.Label(right, textvariable=self.daq_stats, bg=PAPER, fg=INK, font=self.mono,
                 anchor="w", justify="left").pack(fill="x", pady=(10, 0))
        rate_row = ttk.Frame(right)
        rate_row.pack(fill="x", pady=2)
        ttk.Label(rate_row, text="events/s, last minute", style="Muted.TLabel").pack(side="left")
        self.rate_cv = tk.Canvas(rate_row, width=260, height=26, bg=PAPER, highlightthickness=0)
        self.rate_cv.pack(side="left", padx=8)
        self.rate_now = tk.Label(rate_row, text="", bg=PAPER, fg=ACCENT, font=self.mono)
        self.rate_now.pack(side="left")
        self.hist_cv = tk.Canvas(right, height=240, bg=PAPER, highlightthickness=0)
        self.hist_cv.pack(fill="both", expand=True, pady=(10, 0))
        self.hist_cv.bind("<Configure>", lambda e: self.hist_draw())
        self.last_event = tk.StringVar(value="")
        tk.Label(right, textvariable=self.last_event, bg=PAPER, fg=MUTED, font=self.mono,
                 anchor="w", justify="left").pack(fill="x")
        self.hist_bins = 128
        self.hist_reset()
        self.rate_hist = collections.deque(maxlen=60)

    def stack_open(self):
        p = filedialog.askopenfilename(filetypes=[("Stack files", "*.stk *.txt"), ("All", "*")])
        if p:
            with open(p) as fh:
                self.stack_text.delete("1.0", "end")
                self.stack_text.insert("1.0", fh.read())

    def stack_save(self):
        p = filedialog.asksaveasfilename(defaultextension=".stk")
        if p:
            with open(p, "w") as fh:
                fh.write(self.stack_text.get("1.0", "end-1c"))

    def choose_raw(self):
        p = filedialog.asksaveasfilename(defaultextension=".bin",
                                         filetypes=[("Raw buffers", "*.bin")])
        self.save_raw.set(p or "")

    def _parse_stack(self):
        try:
            return ccusb.Stack.parse(self.stack_text.get("1.0", "end"))
        except ValueError as exc:
            self.error(f"stack: {exc}")
            return None

    def stack_run_once(self):
        if not self.require():
            return
        st = self._parse_stack()
        if st is None:
            return
        try:
            reply = self.cc.execute(st)
        except ccusb.CCUSBError as exc:
            self.error(str(exc))
            return
        self.last_event.set("reply: " + " ".join(f"{w:04x}" for w in reply[:24]) +
                            (" ..." if len(reply) > 24 else ""))
        self.log(f"stack executed, {len(reply)} words")

    def hist_reset(self):
        self.hist_counts = [0] * self.hist_bins
        self.daq_events = 0
        self.hist_draw()

    def hist_draw(self):
        try:
            hi = float(parse_int(self.hist_max.get()))
            word = parse_int(self.hist_word.get())
        except ValueError:
            hi, word = 4096.0, 0
        draw_histogram(self.hist_cv, self.hist_counts, 0, hi, f"event word {word}")

    def daq_toggle(self):
        if self.daq_thread and self.daq_thread.is_alive():
            self.daq_stop_flag.set()
            self.daq_btn.configure(text="Stopping...")
            return
        if not self.require():
            return
        st = self._parse_stack()
        if st is None:
            return
        try:
            self.cc.load_stack(st, "data")
            gm = self.cc.reg_read("global_mode")
            self.cc.daq_start()
        except ccusb.CCUSBError as exc:
            self.error(str(exc))
            return
        raw = None
        if self.save_raw.get():
            try:
                raw = open(self.save_raw.get(), "wb")
            except OSError as exc:
                self.error(str(exc))
        self.double_header = bool(gm & ccusb.GMODE_DOUBLE_HEADER)
        self.daq_stop_flag.clear()
        self.daq_buffers = 0
        self.daq_t0 = time.monotonic()
        self.rate_hist.clear()
        self._rate_mark = (time.monotonic(), 0)
        self.daq_thread = threading.Thread(target=self._daq_worker, args=(raw,), daemon=True)
        self.daq_thread.start()
        self.daq_btn.configure(text="Stop")
        self.status.set("Taking data.")
        self.log("DAQ started")
        self.root.after(100, self._daq_poll)

    def _daq_worker(self, raw):
        cc = self.cc
        try:
            while not self.daq_stop_flag.is_set():
                buf = cc.read_buffer(200)
                if buf:
                    self.daq_queue.put(buf)
                    if raw:
                        raw.write(len(buf).to_bytes(4, "little"))
                        raw.write(b"".join(w.to_bytes(2, "little") for w in buf))
            cc.daq_stop()
            while True:          # drain the last buffers
                buf = cc.read_buffer(300)
                if buf is None:
                    break
                self.daq_queue.put(buf)
        except ccusb.CCUSBError as exc:
            self.daq_queue.put(exc)
            try:
                cc.daq_stop()
            except ccusb.CCUSBError:
                pass
        finally:
            if raw:
                raw.close()
            self.daq_queue.put(None)

    def _daq_poll(self):
        done = False
        try:
            word = parse_int(self.hist_word.get())
            hi = max(1, parse_int(self.hist_max.get()))
        except ValueError:
            word, hi = 0, 4096
        last = None
        while True:
            try:
                item = self.daq_queue.get_nowait()
            except queue.Empty:
                break
            if item is None:
                done = True
                break
            if isinstance(item, Exception):
                self.error(str(item))
                continue
            self.daq_buffers += 1
            try:
                events = ccusb.parse_buffer(item, self.double_header)
            except ValueError:
                continue
            for ev in events:
                if ev.stack_id:
                    continue
                self.daq_events += 1
                last = ev
                if word < len(ev.words):
                    b = int(ev.words[word] * self.hist_bins / hi)
                    if 0 <= b < self.hist_bins:
                        self.hist_counts[b] += 1
        now = time.monotonic()
        t_mark, n_mark = self._rate_mark
        if now - t_mark >= 1.0:
            self.rate_hist.append((self.daq_events - n_mark) / (now - t_mark))
            self._rate_mark = (now, self.daq_events)
            draw_sparkline(self.rate_cv, list(self.rate_hist), 260, 26)
            if self.rate_hist:
                self.rate_now.configure(text=f"{self.rate_hist[-1]:,.0f}/s")
        elapsed = now - self.daq_t0
        self.daq_stats.set(f"{self.daq_events:,} events   {self.daq_buffers:,} buffers   {elapsed:,.0f} s")
        if last is not None:
            self.last_event.set("last event: " + " ".join(f"{w:04x}" for w in last.words[:16]))
        self.hist_draw()
        if done:
            self.daq_btn.configure(text="Start")
            self.status.set("DAQ stopped.")
            self.log(f"DAQ stopped: {self.daq_events} events")
        else:
            self.root.after(150, self._daq_poll)

    # -- device actions -------------------------------------------------------
    def refresh_devices(self):
        values = []
        try:
            values = [d.serial or "?" for d in ccusb.list_devices()]
        except (ccusb.CCUSBError, OSError) as exc:
            self.status.set(f"Cannot list devices: {exc}")
        values.append("SIM")
        self.device_box.configure(values=values)
        if not self.device_var.get():
            self.device_var.set(values[0])

    def toggle_connect(self):
        if self.cc:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        serial = self.device_var.get().strip() or None
        try:
            self.cc = ccusb.open(serial)
            fw = self.cc.firmware
        except (ccusb.CCUSBError, OSError) as exc:
            self.cc = None
            self.error(f"Cannot open {serial or 'CC-USB'}: {exc}")
            return
        sim = "  simulator" if self.cc.is_simulator else ""
        self.info_var.set(f"{self.cc.serial}   firmware 0x{fw:08x}{sim}")
        self.connect_btn.configure(text="Disconnect")
        self.status.set("Connected.")
        self.log(f"connected to {self.cc.serial}")
        try:
            self.inhibit_var.set(self.cc.inhibited)
        except ccusb.CCUSBError:
            pass
        self.scan()
        self.regs_read()
        if self.cc.is_simulator and not self.monitor_rows:
            for a in range(4):
                self.monitor_add(5, a, 0, "rate")

    def disconnect(self):
        if self.daq_thread and self.daq_thread.is_alive():
            self.daq_stop_flag.set()
            self.daq_thread.join(timeout=3)
        if self.cc:
            self.cc.close()
        self.cc = None
        self.info_var.set("")
        self.connect_btn.configure(text="Connect")
        self.status.set("Disconnected.")

    def require(self) -> bool:
        if not self.cc:
            self.error("Not connected.")
            return False
        if self.cc.daq_running:
            self.error("Stop the DAQ first.")
            return False
        return True

    def error(self, msg):
        self.status.set(msg)
        self.status_label.configure(fg=ACCENT)
        self.root.after(4000, lambda: self.status_label.configure(fg=MUTED))
        self.log(msg, "err")

    def do_naf(self):
        if not self.require():
            return
        try:
            n, a, f = parse_int(self.n_var.get()), parse_int(self.a_var.get()), parse_int(self.f_var.get())
            data = parse_int(self.data_var.get() or "0")
        except ValueError:
            self.error("N, A, F and data must be numbers (decimal or 0x hex)")
            return
        try:
            r = self.cc.naf(n, a, f, data)
        except (ccusb.CCUSBError, ValueError) as exc:
            self.error(str(exc))
            return
        q = "Q" if r.q else "·"
        x = "X" if r.x else "·"
        if 16 <= f <= 23:
            text = f"N{n} A{a} F{f}  wrote 0x{data:06x}   {q} {x}"
        elif f < 8:
            text = f"N{n} A{a} F{f}  0x{r.data:06x}  {r.data:>8}   {q} {x}"
        else:
            text = f"N{n} A{a} F{f}  control   {q} {x}"
        self.result_var.set(text)
        self.log(text)

    def dataway(self, which):
        if not self.require():
            return
        try:
            getattr(self.cc, which)()
            self.log(f"dataway {which.upper()}")
            self.scan()
        except ccusb.CCUSBError as exc:
            self.error(str(exc))

    def set_inhibit(self):
        if not self.require():
            self.inhibit_var.set(not self.inhibit_var.get())
            return
        try:
            self.cc.inhibit(self.inhibit_var.get())
            self.log(f"inhibit {'on' if self.inhibit_var.get() else 'off'}")
        except ccusb.CCUSBError as exc:
            self.error(str(exc))

    def scan(self):
        if not self.require():
            return
        mask = 0
        try:
            for n in range(1, 24):
                if self.cc.read(n, 0, 0).x:
                    mask |= 1 << (n - 1)
            lams = self.cc.lams
        except ccusb.CCUSBError as exc:
            self.error(str(exc))
            return
        self.draw_crate(mask, lams)
        occupied = [str(n) for n in range(1, 24) if (mask >> (n - 1)) & 1]
        self.log("scan: X from N" + (", ".join(occupied) if occupied else " none"))

    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main(argv=None):
    p = argparse.ArgumentParser(description="CC-USB CAMAC controller GUI")
    p.add_argument("-s", "--serial", help="connect to this controller at start")
    p.add_argument("--sim", action="store_true", help="connect to the simulator")
    args = p.parse_args(argv)
    root = tk.Tk()
    App(root, "SIM" if args.sim else args.serial)
    root.mainloop()


if __name__ == "__main__":
    main()
