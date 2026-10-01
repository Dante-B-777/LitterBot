"""
Motor Characterization Script — LitterBot
==========================================
Sweeps motor commands and records measured RPM from encoder telemetry.
Outputs a CSV table and plots for all 4 wheels.

Requirements:
    pip install pyserial matplotlib numpy

Usage:
    python characterize_motors.py --port COM3
    python characterize_motors.py --port COM3 --commands 100 300 500 700 1000

The STM32 must be in DEV mode (sent automatically).
The firmware must be streaming ENC telemetry (UartCmd_TelemetryTask running).
"""

import argparse
import csv
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial

# ── Constants ────────────────────────────────────────────────────────────────

TICKS_PER_REV  = 537.7   # goBILDA 5203-2402-0019: 28 CPR × 19.2 gear ratio
BAUD           = 115200
STABILIZE_S    = 2.0     # wait this long for speed to stabilize before sampling
SAMPLE_S       = 1.5     # sampling window duration
COAST_S        = 0.5     # coast time between steps
WATCHDOG_MS    = 0.180   # resend motor command every 180ms (watchdog = 250ms)

WHEEL_NAMES    = ["FL", "FR", "RL", "RR"]
WHEEL_COLORS   = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

DEFAULT_COMMANDS = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]

# ── Serial reader thread ──────────────────────────────────────────────────────

class SerialReader:
    """Reads lines from serial in a background thread, parses ENC telemetry."""

    def __init__(self, ser: serial.Serial):
        self._ser    = ser
        self._lock   = threading.Lock()
        self._latest = None   # (timestamp, [fl, fr, rl, rr]) or None
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if line.startswith("ENC "):
                    parts = line.split()
                    if len(parts) == 5:
                        ticks = [int(x) for x in parts[1:5]]
                        with self._lock:
                            self._latest = (time.monotonic(), ticks)
            except Exception:
                pass

    def get(self):
        """Return latest (timestamp, ticks) or None."""
        with self._lock:
            return self._latest

    def stop(self):
        self._stop.set()


# ── Core measurement ──────────────────────────────────────────────────────────

def send(ser: serial.Serial, cmd: str):
    ser.write((cmd + "\n").encode())


def measure_step(ser: serial.Serial, reader: SerialReader,
                 cmd_value: int, direction: int = 1) -> list:
    """
    Apply motor command, wait to stabilize, then sample encoder RPM.

    Returns list of 4 floats: [fl_rpm, fr_rpm, rl_rpm, rr_rpm]
    Positive = forward rotation for that wheel.
    """
    motor_cmd = f"M {direction * cmd_value} {direction * cmd_value} " \
                f"{direction * cmd_value} {direction * cmd_value}"

    print(f"  Sending: {motor_cmd}  (stabilizing {STABILIZE_S}s...)", end="", flush=True)

    # ── Stabilize phase ───────────────────────────────────────────────────────
    t0 = time.monotonic()
    while time.monotonic() - t0 < STABILIZE_S:
        send(ser, motor_cmd)
        time.sleep(WATCHDOG_MS)

    # ── Sample phase ──────────────────────────────────────────────────────────
    print(f"  sampling {SAMPLE_S}s...", end="", flush=True)
    samples = []       # list of RPM-quads
    prev = reader.get()

    t1 = time.monotonic()
    while time.monotonic() - t1 < SAMPLE_S:
        send(ser, motor_cmd)
        time.sleep(WATCHDOG_MS)

        cur = reader.get()
        if cur is None or prev is None:
            prev = cur
            continue
        if cur[0] == prev[0]:
            continue   # same sample, no new ENC line yet

        dt = cur[0] - prev[0]
        if dt < 0.01:
            continue   # guard against near-zero dt

        delta_ticks = [cur[1][i] - prev[1][i] for i in range(4)]
        rpm         = [(d / TICKS_PER_REV / dt) * 60.0 for d in delta_ticks]
        samples.append(rpm)
        prev = cur

    # ── Stop ──────────────────────────────────────────────────────────────────
    send(ser, "X")
    time.sleep(COAST_S)

    if not samples:
        print("  WARNING: no encoder samples received")
        return [0.0, 0.0, 0.0, 0.0]

    avg = [float(np.mean([s[i] for s in samples])) for i in range(4)]
    std = [float(np.std([s[i]  for s in samples])) for i in range(4)]
    print(f"  FL={avg[0]:+.1f} FR={avg[1]:+.1f} RL={avg[2]:+.1f} RR={avg[3]:+.1f} RPM "
          f"(n={len(samples)})")
    return avg, std


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="LitterBot motor characterization")
    parser.add_argument("--port",     required=True, help="Serial port, e.g. COM3")
    parser.add_argument("--commands", nargs="+", type=int,
                        default=DEFAULT_COMMANDS,
                        help="Command values to sweep (default: 100–1000 in steps of 100)")
    parser.add_argument("--both-dirs", action="store_true",
                        help="Also sweep negative commands (reverse direction)")
    parser.add_argument("--out-dir", default=".",
                        help="Directory for CSV and plot output (default: current dir)")
    args = parser.parse_args()

    commands  = sorted(set(args.commands))
    out_dir   = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # ── Open serial ───────────────────────────────────────────────────────────
    print(f"Opening {args.port} at {BAUD} baud...")
    ser = serial.Serial(args.port, BAUD, timeout=0.5)
    time.sleep(1.0)   # let STM32 settle

    reader = SerialReader(ser)

    # Enable dev mode so motor commands are accepted
    print("Enabling DEV mode...")
    send(ser, "DEV ON")
    time.sleep(0.3)

    # ── Sweep ─────────────────────────────────────────────────────────────────
    directions = [1]
    if args.both_dirs:
        directions = [1, -1]

    # results[cmd] = { 1: ([avg_rpm×4], [std×4]), -1: ... }
    results = {}

    for direction in directions:
        dir_label = "FWD" if direction == 1 else "REV"
        print(f"\n{'='*60}")
        print(f"  Direction: {dir_label}")
        print(f"{'='*60}")

        for cmd in commands:
            print(f"\nCommand {cmd} ({dir_label}):")
            avg, std = measure_step(ser, reader, cmd, direction)
            key = direction * cmd
            results[key] = {"avg": avg, "std": std, "cmd": cmd, "dir": direction}

    # Disable dev mode and stop
    send(ser, "DEV OFF")
    send(ser, "X")
    reader.stop()
    ser.close()
    print("\nSerial closed.")

    # ── Build data arrays ─────────────────────────────────────────────────────
    # Separate forward and reverse
    fwd_cmds = sorted([k for k in results if k > 0])
    rev_cmds = sorted([k for k in results if k < 0], reverse=True)

    def rpm_array(keys):
        """keys → (cmd_vals, avg_rpm 4×N, std_rpm 4×N)"""
        cmd_vals = [results[k]["cmd"] * results[k]["dir"] for k in keys]
        avg = np.array([results[k]["avg"] for k in keys])   # shape (N, 4)
        std = np.array([results[k]["std"] for k in keys])
        return cmd_vals, avg, std

    # ── Save CSV ──────────────────────────────────────────────────────────────
    csv_path = out_dir / f"motor_characterization_{timestamp}.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["command", "direction",
                         "fl_rpm", "fr_rpm", "rl_rpm", "rr_rpm",
                         "fl_std", "fr_std", "rl_std", "rr_std"])
        for key in sorted(results.keys()):
            r = results[key]
            writer.writerow([r["cmd"], "FWD" if r["dir"] == 1 else "REV"] +
                            [f"{v:.2f}" for v in r["avg"]] +
                            [f"{v:.2f}" for v in r["std"]])

    print(f"\nCSV saved: {csv_path}")

    # ── Print table ───────────────────────────────────────────────────────────
    print("\n" + "="*70)
    print(f"{'CMD':>6}  {'DIR':>4}  {'FL RPM':>8}  {'FR RPM':>8}  "
          f"{'RL RPM':>8}  {'RR RPM':>8}")
    print("-"*70)
    for key in sorted(results.keys()):
        r = results[key]
        avg = r["avg"]
        dir_s = "FWD" if r["dir"] == 1 else "REV"
        print(f"{r['cmd']:>6}  {dir_s:>4}  "
              f"{avg[0]:>8.1f}  {avg[1]:>8.1f}  {avg[2]:>8.1f}  {avg[3]:>8.1f}")
    print("="*70)

    # ── Plots ─────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=False)
    fig.suptitle("Motor Characterization — Command vs Measured RPM\n"
                 "(no-load, on stand)", fontsize=13, fontweight="bold")
    axes = axes.flatten()

    for i, (wheel, ax, color) in enumerate(zip(WHEEL_NAMES, axes, WHEEL_COLORS)):
        if fwd_cmds:
            cmd_vals, avg, std = rpm_array(fwd_cmds)
            rpm_vals = [avg[j][i] for j in range(len(fwd_cmds))]
            std_vals = [std[j][i] for j in range(len(fwd_cmds))]
            ax.errorbar(cmd_vals, rpm_vals, yerr=std_vals,
                        fmt="o-", color=color, label="Forward",
                        capsize=4, linewidth=2, markersize=6)

        if rev_cmds:
            cmd_vals_r, avg_r, std_r = rpm_array(rev_cmds)
            rpm_vals_r = [avg_r[j][i] for j in range(len(rev_cmds))]
            std_vals_r = [std_r[j][i] for j in range(len(rev_cmds))]
            # Plot as positive command, negative RPM for clarity
            ax.errorbar([abs(c) for c in cmd_vals_r], rpm_vals_r, yerr=std_vals_r,
                        fmt="s--", color=color, label="Reverse",
                        capsize=4, linewidth=2, markersize=6)

        # Theoretical max line (312 RPM at cmd=1000)
        theory_x = np.array([0, 1000])
        theory_y = theory_x * 312.0 / 1000.0
        ax.plot(theory_x, theory_y, "k:", linewidth=1, alpha=0.5, label="Theoretical max")

        ax.set_title(f"{wheel} Wheel", fontweight="bold")
        ax.set_xlabel("Command Value")
        ax.set_ylabel("Measured RPM")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 1050)

    plt.tight_layout()
    plot_path = out_dir / f"motor_characterization_{timestamp}.png"
    plt.savefig(plot_path, dpi=150)
    print(f"Plot saved:  {plot_path}")

    # ── Combined comparison plot ───────────────────────────────────────────────
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    fig2.suptitle("All Wheels Comparison — Command vs Measured RPM",
                  fontsize=13, fontweight="bold")

    if fwd_cmds:
        cmd_vals, avg, std = rpm_array(fwd_cmds)
        for i, (wheel, color) in enumerate(zip(WHEEL_NAMES, WHEEL_COLORS)):
            rpm_vals = [avg[j][i] for j in range(len(fwd_cmds))]
            std_vals = [std[j][i] for j in range(len(fwd_cmds))]
            ax2.errorbar(cmd_vals, rpm_vals, yerr=std_vals,
                         fmt="o-", color=color, label=wheel,
                         capsize=4, linewidth=2, markersize=6)

    theory_x = np.array([0, 1000])
    theory_y = theory_x * 312.0 / 1000.0
    ax2.plot(theory_x, theory_y, "k:", linewidth=1.5, alpha=0.6, label="Theoretical (no-load)")

    ax2.set_xlabel("Command Value")
    ax2.set_ylabel("Measured RPM")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, 1050)

    plt.tight_layout()
    compare_path = out_dir / f"motor_comparison_{timestamp}.png"
    plt.savefig(compare_path, dpi=150)
    print(f"Comparison:  {compare_path}")

    plt.show()


if __name__ == "__main__":
    main()
