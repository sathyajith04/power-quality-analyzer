"""
pq_receiver_v2.py  —  Python receiver for F28027 Power Quality Monitor
=======================================================================
Receives the 6-line UART frame sent by send_csv_full() in pq_normal.c
and plots all panels live.

UART Frame format (115200 8N1):
  # CSV,Vrms,Irms,THD_V%,THD_I%,P,PF,DF,DPF,EVENT   ← comment/header
  CSV,<Vrms>,<Irms>,<THD_V>,<THD_I>,<P>,<PF>,<DF>,<DPF>,<EVENT>
  VWAVE,<s0>,<s1>,...,<s1999>
  IWAVE,<s0>,<s1>,...,<s1999>
  VCYC,<c0>,<c1>,...,<c49>
  DWT,<e0>,<e1>,...,<e49>
  FFT,<m0>,<m1>,...,<m20>

NOTE: Q (Reactive Power) and S (Apparent Power) are NOT transmitted by the
      firmware's send_csv_full() even though they exist in pq_result struct.
      They are derived here as:
          S = Vrms × Irms          [VA]
          Q = sqrt(max(S²  − P², 0))  [VAR]  (clamped to avoid sqrt of negative)

Usage:
  pip install pyserial matplotlib
  python pq_receiver_v2.py --port COM4 --wait 15

Arguments:
  --port   Serial port  (default: COM4  on Windows, /dev/ttyUSB0 on Linux)
  --baud   Baud rate    (default: 115200)
  --wait   Seconds to wait for a full frame before timing out (default: 15)
  --loop   Keep receiving and refreshing plots indefinitely
"""

import argparse
import sys
import time
import serial
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# ── Configuration ────────────────────────────────────────────────────────────
FS          = 2000          # Sampling frequency (Hz) — must match CCS #define
F0          = 50            # Fundamental frequency (Hz)
SPC         = FS // F0      # Samples per cycle = 40
NUM_SAMPLES = 2000
NUM_CYCLES  = NUM_SAMPLES // SPC   # 50 cycles
FFT_BINS    = SPC // 2 + 1         # 21 bins
VNOM        = 230.0         # ← FIX: match C #define VNOM (was 150.0, now 230.0)

# ── Argument Parsing ─────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="F28027 PQ Monitor UART Receiver")
    p.add_argument("--port",  default="COM4",    help="Serial port (e.g. COM4 or /dev/ttyUSB0)")
    p.add_argument("--baud",  default=115200, type=int, help="Baud rate (default 115200)")
    p.add_argument("--wait",  default=15,    type=int, help="Frame timeout in seconds (default 15)")
    p.add_argument("--loop",  action="store_true",     help="Loop: keep re-receiving and refreshing")
    return p.parse_args()

# ── Serial Reader ─────────────────────────────────────────────────────────────
def read_frame(ser, timeout_s):
    """
    Read lines from serial until all 6 data lines are received or timeout.
    Returns a dict with keys: csv, vwave, iwave, vcyc, dwt, fft
    or None on timeout.
    """
    frame = {}
    deadline = time.time() + timeout_s

    print(f"[INFO] Waiting for frame (timeout={timeout_s}s) ...")

    while time.time() < deadline:
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            print(f"[ERROR] Serial read error: {e}")
            return None

        if not raw:
            continue

        try:
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            continue

        if not line:
            continue

        # ✅ DEBUG PRINT
        print("[RAW LINE]", line)

        # Skip comment lines
        if line.startswith("#"):
            print(f"[COMMENT] {line}")
            continue

        tag = line.split(",", 1)[0].upper()

        if tag == "CSV":
            parts = line.split(",")
            # CSV,Vrms,Irms,THD_V,THD_I,P,PF,DF,DPF,EVENT
            if len(parts) >= 10:
                vrms  = float(parts[1])
                irms  = float(parts[2])
                p_val = float(parts[5])

                # ── Derive S and Q (not transmitted by firmware) ──────────
                s_val = vrms * irms                                   # VA
                q_val = float(np.sqrt(max(s_val**2 - p_val**2, 0.0))) # VAR

                dpf_val = float(parts[8])

                # ── Derive displacement angle φ₁ = arccos(DPF) ───────────
                # DPF = cos(φ₁)  where φ₁ is the phase angle between the
                # V and I fundamentals (same formula as the C firmware).
                # arccos is only valid for DPF ∈ [-1, 1]; clamp for safety.
                dpf_clamped = max(-1.0, min(1.0, dpf_val))
                phi1_rad    = float(np.arccos(dpf_clamped))   # radians
                phi1_deg    = float(np.degrees(phi1_rad))     # degrees

                frame["csv"] = {
                    "Vrms":  vrms,
                    "Irms":  irms,
                    "THD_V": float(parts[3]),
                    "THD_I": float(parts[4]),
                    "P":     p_val,
                    "Q":     q_val,
                    "S":     s_val,
                    "PF":    float(parts[6]),
                    "DF":    float(parts[7]),
                    "DPF":   dpf_val,
                    "phi1_deg": phi1_deg,   # displacement angle in degrees
                    "phi1_rad": phi1_rad,   # displacement angle in radians
                    "EVENT": parts[9].strip(),
                }
                print(f"[CSV]  Vrms={frame['csv']['Vrms']:.2f}V  "
                      f"Irms={frame['csv']['Irms']:.4f}A  "
                      f"P={frame['csv']['P']:.2f}W  "
                      f"Q={frame['csv']['Q']:.2f}VAR  "
                      f"S={frame['csv']['S']:.2f}VA  "
                      f"PF={frame['csv']['PF']:.4f}  "
                      f"DPF={frame['csv']['DPF']:.4f}  "
                      f"φ₁={frame['csv']['phi1_deg']:.2f}°  "
                      f"EVENT={frame['csv']['EVENT']}")

        elif tag == "VWAVE":
            vals = [float(x) for x in line.split(",")[1:]]
            frame["vwave"] = np.array(vals, dtype=np.float32)
            print(f"[VWAVE] {len(vals)} samples received")

        elif tag == "IWAVE":
            vals = [float(x) for x in line.split(",")[1:]]
            frame["iwave"] = np.array(vals, dtype=np.float32)
            print(f"[IWAVE] {len(vals)} samples received")

        elif tag == "VCYC":
            vals = [float(x) for x in line.split(",")[1:]]
            frame["vcyc"] = np.array(vals, dtype=np.float32)
            print(f"[VCYC]  {len(vals)} cycle Vrms values")

        elif tag == "DWT":
            vals = [float(x) for x in line.split(",")[1:]]
            frame["dwt"] = np.array(vals, dtype=np.float32)
            print(f"[DWT]   {len(vals)} detail energies")

        elif tag == "FFT":
            vals = [float(x) for x in line.split(",")[1:]]
            frame["fft"] = np.array(vals, dtype=np.float32)
            print(f"[FFT]   {len(vals)} magnitude bins")

        # Check if all 6 lines received
        if all(k in frame for k in ("csv", "vwave", "iwave", "vcyc", "dwt", "fft")):
            print("[INFO] Full frame received.")
            return frame

    print("[WARN] Timeout — partial frame received.")
    return frame if frame else None

# ── Plotting ─────────────────────────────────────────────────────────────────
def plot_frame(frame):
    csv   = frame.get("csv",   {})
    vwave = frame.get("vwave", np.array([]))
    iwave = frame.get("iwave", np.array([]))
    vcyc  = frame.get("vcyc",  np.array([]))
    dwt   = frame.get("dwt",   np.array([]))
    fft   = frame.get("fft",   np.array([]))

    t_wave  = np.arange(len(vwave)) / FS          # time axis in seconds
    t_cyc   = np.arange(len(vcyc))  / F0          # cycle axis in seconds
    f_bins  = np.arange(len(fft))   * (FS / SPC)  # frequency axis in Hz

    event   = csv.get("EVENT", "?")
    color_ev = {"SAG": "red", "SWELL": "orange", "NORMAL": "green"}.get(event, "gray")

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        f"F28027 Power Quality Monitor  |  "
        f"Vrms={csv.get('Vrms','?')} V   Irms={csv.get('Irms','?')} A   "
        f"P={csv.get('P','?')} W   PF={csv.get('PF','?')}   "
        f"THD_V={csv.get('THD_V','?')}%   THD_I={csv.get('THD_I','?')}%   "
        f"EVENT=",
        fontsize=11, fontweight="bold"
    )
    # Add coloured event text separately
    fig.text(0.91, 0.965, event, fontsize=11, fontweight="bold",
             color=color_ev, ha="left", va="center")

    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # ── Panel 1: Voltage Waveform ─────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    if vwave.size:
        ax1.plot(t_wave, vwave, color="royalblue", lw=0.8)
    ax1.set_title("Voltage Waveform")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Voltage (V)")
    ax1.grid(True, alpha=0.3)

    # ── Panel 2: Current Waveform ─────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    if iwave.size:
        ax2.plot(t_wave, iwave, color="darkorange", lw=0.8)
    zoom_factor = 3
    ax2.set_xlim(t_wave[0], t_wave[0] + (t_wave[-1] - t_wave[0]) / zoom_factor)
    ax2.autoscale(enable=False, axis='x')
    ax2.set_title("Current Waveform")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Current (A)")
    ax2.grid(True, alpha=0.3)

    # ── Panel 3: Per-Cycle Vrms ───────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    if vcyc.size:
        ax3.plot(t_cyc, vcyc, marker="o", ms=3, color="steelblue", lw=1.2)
        # ← FIX: use VNOM = 230.0 to match C firmware (was hardcoded 150.0)
        ax3.axhline(VNOM * 1.10, color="orange", ls="--", lw=0.9, label="+10% (Swell)")
        ax3.axhline(VNOM * 0.90, color="red",    ls="--", lw=0.9, label="-10% (Sag)")
        ax3.axhline(VNOM,        color="green",  ls=":",  lw=0.9, label="Vnom")
        ax3.legend(fontsize=7, loc="lower right")
    ax3.set_title("Per-Cycle Vrms")
    ax3.set_xlabel("Cycle time (s)")
    ax3.set_ylabel("Vrms (V)")
    ax3.grid(True, alpha=0.3)

    # ── Panel 4: DWT Detail Energy ────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    if dwt.size:
        ax4.bar(np.arange(len(dwt)), dwt, color="mediumpurple", width=0.7)
    ax4.set_title("DWT Detail Energy per Cycle")
    ax4.set_xlabel("Cycle #")
    ax4.set_ylabel("Energy")
    ax4.grid(True, alpha=0.3, axis="y")

    # ── Panel 5: FFT Spectrum (Current) ──────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 0])
    if fft.size:
        ax5.bar(f_bins, fft, width=f_bins[1] * 0.7 if len(f_bins) > 1 else 10,
                color="tomato", align="center")
        ax5.set_xticks(f_bins)
        ax5.set_xticklabels([f"{int(f)}" for f in f_bins], rotation=45, fontsize=7)
    ax5.set_title("FFT Magnitude Spectrum (Current)")
    ax5.set_xlabel("Frequency (Hz)")
    ax5.set_ylabel("|X(f)|")
    ax5.grid(True, alpha=0.3, axis="y")

    # ── Panel 6: PQ Metrics Table ─────────────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.axis("off")

    # ── Derive formatted values ───────────────────────────────────────────
    p_val = csv.get('P', '—')
    q_val = csv.get('Q', '—')
    s_val = csv.get('S', '—')
    p_str = f"{p_val:.2f}" if isinstance(p_val, float) else str(p_val)
    q_str = f"{q_val:.2f}" if isinstance(q_val, float) else str(q_val)
    s_str = f"{s_val:.2f}" if isinstance(s_val, float) else str(s_val)

    # ── Format displacement angle ─────────────────────────────────────────
    phi1_deg = csv.get('phi1_deg', '—')
    phi1_rad = csv.get('phi1_rad', '—')
    phi1_deg_str = f"{phi1_deg:.2f}" if isinstance(phi1_deg, float) else str(phi1_deg)
    phi1_rad_str = f"{phi1_rad:.4f}" if isinstance(phi1_rad, float) else str(phi1_rad)

    metrics = [
        ["Parameter",               "Value",                       "Unit"],
        ["Vrms",                    f"{csv.get('Vrms','—')}",       "V"],
        ["Irms",                    f"{csv.get('Irms','—')}",       "A"],
        ["Active Power (P)",        p_str,                          "W"],
        ["Reactive Power (Q)",      q_str,                          "VAR"],
        ["Apparent Power (S)",      s_str,                          "VA"],
        ["Power Factor",            f"{csv.get('PF','—')}",         ""],
        ["Distortion Factor (DF)",  f"{csv.get('DF','—')}",         ""],
        ["Displacement Factor (DPF)", f"{csv.get('DPF','—')}",      "cos(φ₁)"],
        # ── NEW: displacement angle derived as φ₁ = arccos(DPF) ──────────
        ["Displacement Angle (φ₁)", phi1_deg_str,                   "°"],
        ["Displacement Angle (φ₁)", phi1_rad_str,                   "rad"],
        ["THD_V",                   f"{csv.get('THD_V','—')}",      "%"],
        ["THD_I",                   f"{csv.get('THD_I','—')}",      "%"],
        ["Event",                   event,                          ""],
    ]
    tbl = ax6.table(cellText=metrics[1:], colLabels=metrics[0],
                    loc="center", cellLoc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.0)
    tbl.scale(1, 1.18)

    # Highlight P / Q / S rows in pale yellow
    pqs_rows = {3: "#fffde7", 4: "#fff9c4", 5: "#fff3e0"}
    for row_idx, bg in pqs_rows.items():
        for col in range(3):
            tbl[row_idx, col].set_facecolor(bg)

    # ── Highlight Displacement Factor + Angle rows in pale cyan ──────────
    for dpf_row in (8, 9, 10):   # DPF, φ₁ degrees, φ₁ radians
        for col in range(3):
            tbl[dpf_row, col].set_facecolor("#e0f7fa")

    # Colour the Event row (last data row = len(metrics) - 1)
    for col in range(3):
        tbl[len(metrics) - 1, col].set_facecolor(
            {"SAG": "#ffcccc", "SWELL": "#ffe0b2", "NORMAL": "#ccffcc"}.get(event, "#eeeeee")
        )

    # ── Print displacement angle proof to console ─────────────────────────
    dpf_v  = csv.get('DPF', None)
    phi_v  = csv.get('phi1_deg', None)
    phi_r  = csv.get('phi1_rad', None)
    if dpf_v is not None:
        print("\n══════════════════════════════════════════════════════")
        print("  DISPLACEMENT FACTOR PROOF")
        print("  Formula (IEEE 1459): DPF = cos(φ₁)")
        print(f"  DPF received from firmware : {dpf_v:.6f}")
        print(f"  φ₁ = arccos(DPF)           : {phi_r:.6f} rad  =  {phi_v:.4f}°")
        print(f"  Verification — cos(φ₁)     : {float(np.cos(phi_r)):.6f}  (should equal DPF)")
        roundtrip_ok = abs(float(np.cos(phi_r)) - dpf_v) < 1e-5
        print(f"  Round-trip match           : {'✓ PASS' if roundtrip_ok else '✗ FAIL'}")
        print("══════════════════════════════════════════════════════\n")
   

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    print(f"[INFO] Opening {args.port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=2.0)
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open port: {e}")
        sys.exit(1)

    time.sleep(0.5)   # Let the DSP settle after port open

    matplotlib.use("TkAgg")   # Change to "Qt5Agg" if TkAgg not available

    try:
        while True:
            frame = read_frame(ser, args.wait)
            if frame:
                plot_frame(frame)
            else:
                print("[WARN] No data received. Check board power and COM port.")

            if not args.loop:
                break

            print("\n[INFO] --loop active: waiting for next frame ...\n")

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        ser.close()
        print("[INFO] Serial port closed.")

if __name__ == "__main__":
    main()