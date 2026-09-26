#!/usr/bin/env python3
"""
Automatic SOLT calibration of a LibreVNA using a LibreCAL electronic cal kit.

Flow:
  1. connect to LibreVNA-GUI (SCPI/TCP) and LibreCAL (SCPI/USB serial)
  2. wait for the LibreCAL temperature to stabilise
  3. download the characterised standards (coefficient set, default FACTORY)
     from the LibreCAL and store them as Touchstone files
  4. back up the current GUI cal kit, then build a new kit from those files
  5. create the OPEN/SHORT/LOAD/THROUGH measurements, switch the LibreCAL
     through all states and let the GUI measure them
  6. activate the SOLT calibration and save it to a .cal file
  7. optional: verify by re-measuring LibreCAL states with the new calibration

The LibreVNA-GUI must be running with the SCPI server enabled and the VNA
connected. Close the LibreCAL port in LibreCAL-GUI / LibreVNA-GUI eCal dialog
before running (only one program can hold the serial port).

Every output file carries the run timestamp (YYYYMMDD_HHMMSS), so runs never
overwrite each other and all files of one run share the same stamp. In
addition, latest.cal is overwritten with the newest calibration so that
changes between runs can be inspected with `git diff`.
"""
import argparse
import datetime
import math
import os
import shutil
import socket
import sys
import time

import serial
import serial.tools.list_ports

LIBRECAL_VID_PID = (0x1209, 0x4122)


# ---------------------------------------------------------------- LibreVNA --
class LibreVNA:
    def __init__(self, host="localhost", port=19542, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.f = self.sock.makefile("rw", newline="\n")
        self.query("*ESR?")  # clear stale error flags

    def _send(self, cmd):
        self.f.write(cmd + "\n")
        self.f.flush()

    def query(self, cmd, timeout=None):
        if timeout is not None:
            self.sock.settimeout(timeout)
        try:
            self._send(cmd)
            return self.f.readline().strip()
        except socket.timeout:
            raise RuntimeError(f"LibreVNA: no answer to '{cmd}' (invalid query?)")
        finally:
            if timeout is not None:
                self.sock.settimeout(5.0)

    def cmd(self, cmd):
        self._send(cmd)
        esr = int(self.query("*ESR?"))
        if esr & 0x3C:  # command / execution / device / query error bits
            raise RuntimeError(f"LibreVNA: command failed (ESR={esr}): {cmd}")


# ---------------------------------------------------------------- LibreCAL --
class LibreCAL:
    def __init__(self, device=None):
        if device is None:
            for p in serial.tools.list_ports.comports():
                if (p.vid, p.pid) == LIBRECAL_VID_PID:
                    device = p.device
                    break
            else:
                raise RuntimeError("LibreCAL not found on USB")
        self.ser = serial.Serial(device, 115200, timeout=2)
        self.ser.reset_input_buffer()

    def query(self, cmd, retries=3):
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\r\n").encode())
            ans = self.ser.readline().decode(errors="replace").strip()
            if ans:
                return ans
        raise RuntimeError(f"LibreCAL: no answer to '{cmd}'")

    def cmd(self, cmd):
        # LibreCAL answers events with an empty line (or ERROR)
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r\n").encode())
        ans = self.ser.readline().decode(errors="replace").strip()
        if ans.upper().startswith("ERR"):
            raise RuntimeError(f"LibreCAL: command failed: {cmd} -> {ans}")

    def set_port(self, port, standard, dest=None):
        self.cmd(f":PORT {port} {standard}" + (f" {dest}" if dest else ""))
        expected = f"THROUGH {dest}" if standard == "THROUGH" else standard
        actual = self.query(f":PORT? {port}")
        if actual != expected:
            raise RuntimeError(f"LibreCAL port {port}: expected {expected}, got {actual}")

    def all_ports(self, standard):
        for p in range(1, self.ports + 1):
            self.cmd(f":PORT {p} {standard}")

    @property
    def ports(self):
        return int(self.query(":PORTS?"))

    def coefficient(self, cset, name):
        """Returns list of (freq_Hz, [complex S...])"""
        n = int(self.query(f":COEFF:NUM? {cset} {name}"))
        data = []
        for i in range(n):
            v = [float(x) for x in self.query(f":COEFF:GET? {cset} {name} {i}").split(",")]
            s = [complex(v[k], v[k + 1]) for k in range(1, len(v), 2)]
            data.append((v[0] * 1e9, s))
        return data


# ----------------------------------------------------------------- helpers --
def write_touchstone(path, data, comment):
    # LibreCAL returns S11,S21,S12,S22 for throughs, which is also the Touchstone 2-port order
    with open(path, "w") as f:
        f.write(f"! {comment}\n# GHz S RI R 50\n")
        for freq, s in data:
            vals = " ".join(f"{x.real:.9g} {x.imag:.9g}" for x in s)
            f.write(f"{freq / 1e9:.9f} {vals}\n")


def swap_ports(data):
    # S11,S21,S12,S22 -> S22,S12,S21,S11
    return [(f, [s[3], s[2], s[1], s[0]]) for f, s in data]


def interpolate(data, freq):
    lo, hi = 0, len(data) - 1
    if not data[lo][0] <= freq <= data[hi][0]:
        return None
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if data[mid][0] <= freq:
            lo = mid
        else:
            hi = mid
    (f0, s0), (f1, s1) = data[lo], data[hi]
    a = 0 if f1 == f0 else (freq - f0) / (f1 - f0)
    return [x0 + a * (x1 - x0) for x0, x1 in zip(s0, s1)]


def parse_map(text):
    # "1:1,2:2" -> {vna_port: librecal_port}
    m = {}
    for item in text.split(","):
        v, c = item.split(":")
        m[int(v)] = int(c)
    return m


def wait_cal_done(vna, timeout=120):
    time.sleep(0.5)
    t0 = time.time()
    while vna.query("VNA:CAL:BUSY?") == "TRUE":
        if time.time() - t0 > timeout:
            raise RuntimeError("calibration measurement timed out")
        time.sleep(0.2)


def fresh_sweep(vna, timeout=120):
    """Trigger one complete sweep and wait for it to finish."""
    vna.cmd("VNA:ACQ:SINGLE TRUE")
    vna.cmd("VNA:ACQ:RUN")
    time.sleep(0.3)
    t0 = time.time()
    while vna.query("VNA:ACQ:FIN?") != "TRUE":
        if time.time() - t0 > timeout:
            raise RuntimeError("sweep timed out")
        time.sleep(0.1)


def read_trace(vna, name):
    raw = vna.query(f"VNA:TRACE:DATA? {name}", timeout=20)
    out = []
    for tup in raw.strip("[]").split("],["):
        f, re_, im = (float(x) for x in tup.split(","))
        out.append((f, complex(re_, im)))
    return out


def max_error(measured, expected_data, idx):
    worst = 0.0
    for f, s in measured:
        e = interpolate(expected_data, f)
        if e is not None:
            worst = max(worst, abs(s - e[idx]))
    return worst


# -------------------------------------------------------------------- main --
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default="1:1,2:2",
                    help="VNA port -> LibreCAL port mapping, e.g. '1:1,2:2' or '1:3,2:4' (default 1:1,2:2)")
    ap.add_argument("--set", default="FACTORY", help="LibreCAL coefficient set (default FACTORY)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "librecal_out"),
                    help="output directory (default: librecal_out next to this script)")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--scpi-port", type=int, default=19542)
    ap.add_argument("--serial", help="LibreCAL serial device (default: auto-detect)")
    ap.add_argument("--temp-timeout", type=float, default=900, help="max seconds to wait for LibreCAL temperature")
    ap.add_argument("--no-verify", action="store_true", help="skip verification after calibration")
    args = ap.parse_args()

    pmap = parse_map(args.map)
    vna_ports = sorted(pmap)
    out = os.path.abspath(args.out)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(out, exist_ok=True)

    # 1. connect
    vna = LibreVNA(args.host, args.scpi_port)
    idn = vna.query("*IDN?")
    dev = vna.query("DEV:CONN?")
    if not dev or dev.lower() == "not connected":
        sys.exit("LibreVNA-GUI is not connected to a device")
    print(f"VNA:      {idn} (device {dev})")
    cal = LibreCAL(args.serial)
    cal_idn = cal.query("*IDN?")
    cal_serial = cal_idn.split(",")[2]
    print(f"LibreCAL: {cal_idn}")
    for v, c in pmap.items():
        if not 1 <= c <= cal.ports:
            sys.exit(f"LibreCAL has no port {c}")
    if args.set not in cal.query(":COEFF:LIST?").split(","):
        sys.exit(f"coefficient set {args.set} not found on LibreCAL")
    vna.cmd("DEV:MODE VNA")

    # 2. temperature
    t0 = time.time()
    while cal.query(":TEMP:STABLE?") != "TRUE":
        if time.time() - t0 > args.temp_timeout:
            sys.exit("LibreCAL temperature did not stabilise")
        print(f"\rwaiting for LibreCAL temperature: {float(cal.query(':TEMP?')):.2f} °C", end="", flush=True)
        time.sleep(2)
    print(f"LibreCAL temperature stable at {float(cal.query(':TEMP?')):.2f} °C")

    # 3. download coefficients -> touchstone files
    tsdir = os.path.join(out, f"coeffs_{cal_serial}_{args.set}_{stamp}")
    os.makedirs(tsdir, exist_ok=True)
    coeffs = {}  # our standard name -> (touchstone path, data, kit type)
    for v in vna_ports:
        c = pmap[v]
        for std, kit_type in (("OPEN", "Open"), ("SHORT", "Short"), ("LOAD", "Load")):
            name = f"LC_P{c}_{std}"
            print(f"reading {args.set}/P{c}_{std} ...")
            data = cal.coefficient(args.set, f"P{c}_{std}")
            path = os.path.join(tsdir, f"P{c}_{std}.s1p")
            write_touchstone(path, data, f"LibreCAL {cal_serial} set {args.set} P{c}_{std}")
            coeffs[name] = (path, data, kit_type)
    pairs = [(a, b) for i, a in enumerate(vna_ports) for b in vna_ports[i + 1:]]
    for a, b in pairs:
        ca, cb = pmap[a], pmap[b]
        lo, hi = min(ca, cb), max(ca, cb)
        print(f"reading {args.set}/P{lo}{hi}_THROUGH ...")
        data = cal.coefficient(args.set, f"P{lo}{hi}_THROUGH")
        if ca > cb:  # VNA port order is reversed relative to LibreCAL port order
            data = swap_ports(data)
        name = f"LC_P{ca}{cb}_THROUGH"
        path = os.path.join(tsdir, f"P{ca}{cb}_THROUGH.s2p")
        write_touchstone(path, data, f"LibreCAL {cal_serial} set {args.set} P{lo}{hi}_THROUGH (ports ordered {ca},{cb})")
        coeffs[name] = (path, data, "Through")

    fmin = max(d[0][0] for _, d, _ in coeffs.values())
    fmax = min(d[-1][0] for _, d, _ in coeffs.values())
    start, stop = float(vna.query("VNA:FREQ:START?")), float(vna.query("VNA:FREQ:STOP?"))
    if start < fmin or stop > fmax:
        sys.exit(f"VNA sweep {start/1e6:g}-{stop/1e6:g} MHz exceeds LibreCAL coefficient range "
                 f"{fmin/1e6:g}-{fmax/1e6:g} MHz")

    # 4. calibration kit
    backup = os.path.join(out, f"calkit_backup_{stamp}.calkit")
    vna.cmd(f"VNA:CAL:KIT:SAVE {backup}")
    print(f"previous cal kit backed up to {backup}")
    while int(vna.query("VNA:CAL:KIT:STA:NUM?")) > 0:
        vna.cmd("VNA:CAL:KIT:STA:DEL 1")
    for name, (path, _, kit_type) in coeffs.items():
        vna.cmd(f"VNA:CAL:KIT:STA:NEW {kit_type} {name}")
        idx = int(vna.query("VNA:CAL:KIT:STA:NUM?"))
        if kit_type == "Through":
            vna.cmd(f"VNA:CAL:KIT:STA:{idx}:FILE {path} 1 2")
        else:
            vna.cmd(f"VNA:CAL:KIT:STA:{idx}:FILE {path} 1")
    vna.cmd("VNA:CAL:KIT:MAN LibreCAL")
    vna.cmd(f"VNA:CAL:KIT:SER {cal_serial}")
    vna.cmd(f"VNA:CAL:KIT:DESC LibreCAL_{args.set}_{stamp}")
    vna.cmd(f"VNA:CAL:KIT:SAVE {os.path.join(out, f'librecal_{cal_serial}_{args.set}_{stamp}.calkit')}")

    # 5. measurements
    vna.cmd("VNA:CAL:RESET")
    meas = {}  # (std, vna_port or pair) -> measurement index
    for std in ("OPEN", "SHORT", "LOAD"):
        for v in vna_ports:
            vna.cmd(f"VNA:CAL:ADD {std} LC_P{pmap[v]}_{std}")
            i = int(vna.query("VNA:CAL:NUM?")) - 1
            vna.cmd(f"VNA:CAL:PORT {i} {v}")
            meas[(std, v)] = i
    for a, b in pairs:
        vna.cmd(f"VNA:CAL:ADD THROUGH LC_P{pmap[a]}{pmap[b]}_THROUGH")
        i = int(vna.query("VNA:CAL:NUM?")) - 1
        vna.cmd(f"VNA:CAL:PORT {i} {a} {b}")
        meas[("THROUGH", (a, b))] = i

    vna.cmd("VNA:ACQ:SINGLE FALSE")
    vna.cmd("VNA:ACQ:RUN")
    try:
        for std in ("OPEN", "SHORT", "LOAD"):
            cal.all_ports("NONE")
            for v in vna_ports:
                cal.set_port(pmap[v], std)
            time.sleep(0.2)
            ids = [meas[(std, v)] for v in vna_ports]
            print(f"measuring {std} on VNA ports {vna_ports} ...")
            vna.cmd("VNA:CAL:MEAS " + " ".join(map(str, ids)))
            wait_cal_done(vna)
        for a, b in pairs:
            cal.all_ports("NONE")
            cal.set_port(pmap[a], "THROUGH", pmap[b])
            time.sleep(0.2)
            print(f"measuring THROUGH VNA ports {a}-{b} ...")
            vna.cmd(f"VNA:CAL:MEAS {meas[('THROUGH', (a, b))]}")
            wait_cal_done(vna)
    finally:
        cal.all_ports("NONE")

    # 6. activate + save
    available = [t for t in vna.query("VNA:CAL:ACT?").split(",") if t]
    port_str = "".join(map(str, vna_ports))
    candidates = [t for t in available if t.upper().startswith("SOLT") and port_str in t] or \
                 [t for t in available if t.upper().startswith("SOLT")]
    if not candidates:
        sys.exit(f"no SOLT calibration available (available: {available})")
    caltype = max(candidates, key=len)
    vna.cmd(f"VNA:CAL:ACT {caltype}")
    print(f"activated calibration: {vna.query('VNA:CAL:ACTIVE?')}")
    calfile = os.path.join(out, f"librecal_{stamp}.cal")
    vna.cmd(f"VNA:CAL:SAVE {calfile}")
    print(f"calibration saved to {calfile}")
    # stable-named copy so `git diff` shows what changed between runs
    latest = os.path.join(out, "latest.cal")
    shutil.copyfile(calfile, latest)
    print(f"copied to {latest}")

    # 7. verification: corrected measurement of LibreCAL states vs. their coefficients
    if not args.no_verify:
        print("\nverification (max vector error |S_meas - S_coeff| over the sweep):")
        try:
            for std in ("OPEN", "SHORT", "LOAD"):
                cal.all_ports("NONE")
                for v in vna_ports:
                    cal.set_port(pmap[v], std)
                time.sleep(0.2)
                fresh_sweep(vna)
                for v in vna_ports:
                    err = max_error(read_trace(vna, f"S{v}{v}"), coeffs[f"LC_P{pmap[v]}_{std}"][1], 0)
                    print(f"  {std:5s} S{v}{v}: {err:.4f}")
            for a, b in pairs:
                cal.all_ports("NONE")
                cal.set_port(pmap[a], "THROUGH", pmap[b])
                time.sleep(0.2)
                fresh_sweep(vna)
                data = coeffs[f"LC_P{pmap[a]}{pmap[b]}_THROUGH"][1]
                for trace, idx in ((f"S{b}{a}", 1), (f"S{a}{b}", 2)):
                    err = max_error(read_trace(vna, trace), data, idx)
                    db = 20 * math.log10(1 + err) if err < 1 else float("inf")
                    print(f"  THRU  {trace}: {err:.4f} (≈ ±{db:.3f} dB)")
        finally:
            cal.all_ports("NONE")
            vna.cmd("VNA:ACQ:SINGLE FALSE")
            vna.cmd("VNA:ACQ:RUN")

    print("\ndone.")


if __name__ == "__main__":
    main()
