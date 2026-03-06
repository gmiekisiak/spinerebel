"""
SPINE PDR v5 — Nadir-Based Zero-Cycle
======================================

Uses the NADIR (closest approach to Neutral Zone hub) within each
heel-strike interval as the Zero-Cycle boundary.

Nadir vs hub crossings:
  Hub crossings require the spine to actually reach the hub each step.
  On ramps the NZ shifts by several degrees and the spine never crosses
  the global hub → steps are lost. Nadirs always exist.

  Hub crossings: 2005 cycles, 1.47% closure
  Nadir-based:   3782 cycles, 0.71% closure  (89% more cycles recovered)

Method:
  1. Detect heel strikes from thorax accelerometer (impact propagation)
  2. Within each HS-to-HS interval, find the nadir:
     point closest to the global hub in pitch-roll space
  3. Zero-Cycle heading extraction at nadir boundaries
  4. Stride from differential accelerometry (T4-S1 "35 cm ruler")
  5. NZ pitch shift from hub → terrain classification (free byproduct)

Author: Grzegorz Miekisiak, MD PhD / SpineRebel Technology
(c) 2026 SpineRebel Technology — Patent Protected
Academic and non-commercial use only. See LICENSE.
"""

import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from scipy.signal import find_peaks, butter, filtfilt
from scipy.ndimage import gaussian_filter1d, gaussian_filter
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import os


# =============================================================================
# DATA LOADING
# =============================================================================

def load_nimbrace_bin(filepath, log_callback=None):
    def log(msg):
        if log_callback:
            log_callback(msg)
    
    with open(filepath, 'rb') as f:
        data = f.read()
    
    # Detect format
    header_size = 0
    if len(data) >= 86 and data[:7] == b'NIMDATA':
        header_size = 86
    else:
        valid = sum(1 for i in range(min(20, len(data)//92))
                    if 0.99 < np.sqrt(sum(f**2 for f in 
                    struct.unpack('<22f', data[i*92+4:i*92+92])[0:4])) < 1.01)
        if valid < 15:
            header_size = 86
    
    data_section = data[header_size:]
    num_packets = len(data_section) // 92
    
    log(f"{'NIMDATA' if header_size else 'Raw'} | {num_packets} packets")
    
    timestamps = []
    pelvis = {'quats': [], 'accels': [], 'gyros': []}
    thorax = {'quats': [], 'accels': [], 'gyros': []}
    
    for i in range(num_packets):
        chunk = data_section[i*92:(i+1)*92]
        if len(chunk) < 92:
            break
        ts = struct.unpack('<I', chunk[0:4])[0]
        f = struct.unpack('<22f', chunk[4:92])
        timestamps.append(ts)
        pelvis['quats'].append(f[0:4])
        pelvis['accels'].append(f[4:7])
        pelvis['gyros'].append(f[7:10])
        thorax['quats'].append(f[10:14])
        thorax['accels'].append(f[14:17])
        thorax['gyros'].append(f[17:20])
    
    result = {
        'timestamps': np.array(timestamps),
        'pelvis': {k: np.array(v) for k, v in pelvis.items()},
        'thorax': {k: np.array(v) for k, v in thorax.items()},
    }
    
    dt = np.median(np.diff(result['timestamps'])) / 1000.0
    duration = (result['timestamps'][-1] - result['timestamps'][0]) / 1000.0
    log(f"Duration: {duration:.0f}s | Rate: {1/dt:.0f} Hz")
    
    return result


# =============================================================================
# QUATERNION UTILITIES
# =============================================================================

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_to_yaw(q):
    w, x, y, z = q
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def quat_to_euler(q):
    w, x, y, z = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


# =============================================================================
# HEEL STRIKE DETECTION FROM THORAX
# =============================================================================

def detect_heel_strikes(thorax_accels, sample_rate, log_callback=None):
    """Detect heel strikes from thorax accelerometer impacts.
    
    Impact from foot contact propagates through the spine and is
    detectable at T4 as a high-frequency acceleration spike.
    """
    log = log_callback or (lambda m: None)
    
    amag = np.linalg.norm(thorax_accels, axis=1)
    b, a = butter(2, 2.0 / (sample_rate / 2), btype='high')
    amag_hp = filtfilt(b, a, amag)
    
    pos = amag_hp[amag_hp > 0]
    thresh = np.median(pos) * 0.5 if len(pos) > 0 else 0.5
    
    hs_idx, _ = find_peaks(amag_hp, height=thresh, distance=int(0.3 * sample_rate))
    
    log(f"Heel strikes: {len(hs_idx)}")
    if len(hs_idx) > 2:
        intervals = np.diff(hs_idx) / sample_rate
        log(f"  Interval: {np.mean(intervals):.3f} +/- {np.std(intervals):.3f}s")
        log(f"  Cadence: {60/np.mean(intervals):.0f} steps/min")
    
    return hs_idx


# =============================================================================
# HUB FINDER (Panjabi Neutral Zone center)
# =============================================================================

def find_hub(pitch, roll, n_bins=50):
    """Find the NZ center from 2D orientation histogram."""
    hist, pe, re = np.histogram2d(pitch, roll, bins=n_bins)
    hist_smooth = gaussian_filter(hist, sigma=2)
    
    pc = (pe[:-1] + pe[1:]) / 2
    rc = (re[:-1] + re[1:]) / 2
    pg, rg = np.meshgrid(pc, rc, indexing='ij')
    
    threshold = np.percentile(hist_smooth, 95)
    mask = hist_smooth > threshold
    
    if np.sum(mask) > 0:
        weights = hist_smooth[mask]
        hub_pitch = np.average(pg[mask], weights=weights)
        hub_roll = np.average(rg[mask], weights=weights)
    else:
        mi = np.unravel_index(np.argmax(hist_smooth), hist_smooth.shape)
        hub_pitch = (pe[mi[0]] + pe[mi[0]+1]) / 2
        hub_roll = (re[mi[1]] + re[mi[1]+1]) / 2
    
    return hub_pitch, hub_roll, hist_smooth, pe, re


# =============================================================================
# NADIR DETECTION
# =============================================================================

def detect_nadirs(hs_idx, pel_pitch, pel_roll, hub_pitch, hub_roll, 
                  log_callback=None):
    """Within each HS-to-HS interval, find the nadir: the point closest
    to the global hub in pitch-roll space. This is where the spine
    "parks" during that step.
    
    On level ground: nadir ~ hub (NZ shift ~ 0)
    On ramps: nadir shifts away from hub (NZ shift = terrain signal)
    """
    log = log_callback or (lambda m: None)
    
    pitch_smooth = gaussian_filter1d(pel_pitch, sigma=3)
    
    nadir_indices = []
    nadir_pitches = []
    nadir_rolls = []
    nadir_distances = []
    
    for i in range(len(hs_idx) - 1):
        s, e = hs_idx[i], hs_idx[i + 1]
        if e - s < 10 or e - s > 300:
            continue
        
        d2hub = np.sqrt((pitch_smooth[s:e] - hub_pitch)**2 + 
                        (pel_roll[s:e] - hub_roll)**2)
        
        min_idx = np.argmin(d2hub)
        abs_idx = s + min_idx
        
        nadir_indices.append(abs_idx)
        nadir_pitches.append(pitch_smooth[abs_idx])
        nadir_rolls.append(pel_roll[abs_idx])
        nadir_distances.append(d2hub[min_idx])
    
    nadir_indices = np.array(nadir_indices)
    nadir_pitches = np.array(nadir_pitches)
    nadir_rolls = np.array(nadir_rolls)
    nadir_distances = np.array(nadir_distances)
    
    log(f"Nadirs: {len(nadir_indices)}")
    log(f"  NZ pitch: {np.mean(nadir_pitches):.1f} +/- {np.std(nadir_pitches):.1f} deg")
    log(f"  Dist to hub: {np.mean(nadir_distances):.2f} +/- {np.std(nadir_distances):.2f} deg")
    
    return nadir_indices, nadir_pitches, nadir_rolls, nadir_distances


# =============================================================================
# TERRAIN CLASSIFICATION FROM NZ SHIFT
# =============================================================================

def classify_terrain_nz(nadir_pitches, hub_pitch, sigma_smooth=10, 
                        threshold=2.0, log_callback=None):
    """Classify terrain from NZ pitch offset.
    
    On slopes, gravity redefines neutral -> NZ shifts.
    Uphill: pelvis tilts backward -> lower pitch -> negative shift
    Downhill: pelvis tilts forward -> higher pitch -> positive shift
    """
    log = log_callback or (lambda m: None)
    
    nz_shift = nadir_pitches - hub_pitch
    nz_shift_smooth = gaussian_filter1d(nz_shift, sigma=sigma_smooth)
    
    terrain = np.full(len(nadir_pitches), 'level', dtype='U10')
    for i in range(len(nz_shift_smooth)):
        if nz_shift_smooth[i] < -threshold:
            terrain[i] = 'uphill'
        elif nz_shift_smooth[i] > threshold:
            terrain[i] = 'downhill'
    
    # Minimum segment length (5 steps)
    i = 0
    while i < len(terrain):
        j = i
        while j < len(terrain) and terrain[j] == terrain[i]:
            j += 1
        if j - i < 5 and terrain[i] != 'level':
            terrain[i:j] = 'level'
        i = j
    
    counts = {t: np.sum(terrain == t) for t in ['level', 'uphill', 'downhill']}
    for t, c in counts.items():
        log(f"  {t:10s}: {c:4d} steps ({100*c/len(terrain):.0f}%)")
    
    return terrain, nz_shift, nz_shift_smooth


# =============================================================================
# STRIDE ESTIMATION
# =============================================================================

def compute_diff_accel_mag(pelvis_accels, thorax_accels, boundaries, sample_rate):
    """Differential acceleration magnitude stride proxy.
    
    Integrates ||a_thorax - a_pelvis|| over each cycle.
    The 35 cm T4-S1 spacing acts as a biomechanical ruler
    measuring total spine deformation per step.
    """
    dt = 1.0 / sample_rate
    excursions = []
    
    for i in range(len(boundaries) - 1):
        start = boundaries[i]
        end = boundaries[i + 1]
        
        a_diff = thorax_accels[start:end] - pelvis_accels[start:end]
        a_diff_mag = np.linalg.norm(a_diff, axis=1)
        
        excursions.append(np.sum(a_diff_mag) * dt)
    
    return np.array(excursions)


def compute_weinberg_thorax(thorax_accels, boundaries):
    """Weinberg stride model: stride ~ (a_max - a_min)^0.25"""
    strides = []
    for i in range(len(boundaries) - 1):
        start = boundaries[i]
        end = boundaries[i + 1]
        a_mag = np.linalg.norm(thorax_accels[start:end], axis=1)
        if len(a_mag) > 5:
            strides.append((np.max(a_mag) - np.min(a_mag))**0.25)
        else:
            strides.append(0)
    return np.array(strides)


# =============================================================================
# ZERO-CYCLE PDR
# =============================================================================

def zero_cycle_pdr(pelvis_quats, boundaries, excursions, 
                   known_distance=None, user_height=None):
    """Zero-Cycle PDR with three stride modes:
      1. Calibrated: known_distance -> K = dist/sum(exc)
      2. Height-based: K = height * 0.375 / mean(exc)
      3. Default: population-average height (1.70 m)
    
    Each nadir boundary = 1 step (not 1 full gait cycle).
    """
    if len(boundaries) < 2:
        return None
    
    n_cycles = len(boundaries) - 1
    
    # Zero-Cycle heading extraction
    headings = []
    for i in range(n_cycles):
        Q_start = pelvis_quats[boundaries[i]]
        Q_end = pelvis_quats[boundaries[i + 1]]
        Q_delta = quat_multiply(Q_end, quat_conjugate(Q_start))
        delta = quat_to_yaw(Q_delta)
        
        while delta > np.pi:
            delta -= 2 * np.pi
        while delta < -np.pi:
            delta += 2 * np.pi
        
        headings.append(delta)
    
    headings = np.array(headings)
    
    # Stride computation
    if known_distance is not None and known_distance > 0:
        K = known_distance / np.sum(excursions)
        strides = K * excursions
        method = f"Calibrated (K={K:.4f})"
    elif user_height is not None and user_height > 0:
        step_est = user_height * 0.375
        mean_exc = np.mean(excursions)
        K = step_est / mean_exc
        strides = K * excursions
        method = f"Height-based (h={user_height:.2f}m, K={K:.4f})"
    else:
        default_height = 1.70
        step_est = default_height * 0.375
        mean_exc = np.mean(excursions)
        K = step_est / mean_exc
        strides = K * excursions
        method = f"Default (h=1.70m, K={K:.4f})"
    
    strides = np.clip(strides, 0.15, 3.0)
    
    # Build path
    min_len = min(len(headings), len(strides))
    headings = headings[:min_len]
    strides = strides[:min_len]
    
    cumulative = np.insert(np.cumsum(headings), 0, 0)
    
    x, y = [0.0], [0.0]
    for i in range(min_len):
        h = (cumulative[i] + cumulative[i + 1]) / 2
        x.append(x[-1] + strides[i] * np.cos(h))
        y.append(y[-1] + strides[i] * np.sin(h))
    
    x, y = np.array(x), np.array(y)
    total_distance = np.sum(strides)
    end_dist = np.sqrt(x[-1]**2 + y[-1]**2)
    
    return {
        'x': x, 'y': y,
        'headings': headings,
        'headings_deg': np.degrees(headings),
        'strides': strides,
        'excursions': excursions[:min_len],
        'cumulative_heading': cumulative,
        'distance': total_distance,
        'end_dist': end_dist,
        'closure_pct': 100 * end_dist / total_distance if total_distance > 0 else 0,
        'total_heading_deg': np.degrees(cumulative[-1]),
        'method': method,
        'n_cycles': min_len,
        'n_steps': min_len,
    }


def analyze_panjabi(headings_deg):
    """Heading statistics and random walk verification."""
    std_heading = np.std(headings_deg)
    residuals = headings_deg - np.mean(headings_deg)
    
    autocorr = np.correlate(residuals, residuals, mode='full')
    autocorr = autocorr[len(autocorr)//2:]
    if autocorr[0] != 0:
        autocorr = autocorr / autocorr[0]
    
    cumsum = np.cumsum(residuals)
    sqrt_n_bound = std_heading * np.sqrt(len(residuals))
    
    return {
        'std_heading': std_heading,
        'autocorr_lag1': autocorr[1] if len(autocorr) > 1 else 0,
        'cumsum_final': cumsum[-1] if len(cumsum) > 0 else 0,
        'sqrt_n_bound': sqrt_n_bound,
        'cumsum': cumsum
    }


# =============================================================================
# GUI APPLICATION
# =============================================================================

TERRAIN_COLORS = {'level': '#4682B4', 'uphill': '#DC143C', 'downhill': '#1E90FF'}

class SpinePDRApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Spine PDR v5 — Nadir-Based Zero-Cycle")
        self.root.geometry("1500x950")
        self.root.minsize(1300, 850)
        
        style = ttk.Style()
        style.configure('Title.TLabel', font=('Helvetica', 14, 'bold'))
        
        self.results = None
        self.create_widgets()
        
    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_frame = ttk.Frame(main_frame)
        title_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(title_frame,
                  text="SPINE PDR v5 — NADIR-BASED ZERO-CYCLE",
                  style='Title.TLabel').pack(side=tk.LEFT)
        
        # Controls
        ctrl_frame = ttk.LabelFrame(main_frame, text="Controls", padding="8")
        ctrl_frame.pack(fill=tk.X, pady=(0, 10))
        
        row = ttk.Frame(ctrl_frame)
        row.pack(fill=tk.X)
        
        ttk.Label(row, text="File:").pack(side=tk.LEFT)
        self.file_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.file_var, width=50).pack(
            side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        ttk.Button(row, text="Browse", command=self.browse).pack(
            side=tk.LEFT, padx=(0, 10))
        
        ttk.Label(row, text="Known dist (m):").pack(side=tk.LEFT)
        self.dist_var = tk.StringVar(value="")
        ttk.Entry(row, textvariable=self.dist_var, width=8).pack(
            side=tk.LEFT, padx=(3, 10))
        
        ttk.Label(row, text="Height (m):").pack(side=tk.LEFT)
        self.height_var = tk.StringVar(value="")
        ttk.Entry(row, textvariable=self.height_var, width=6).pack(
            side=tk.LEFT, padx=(3, 10))
        
        self.process_btn = ttk.Button(row, text="Process", command=self.process)
        self.process_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        self.save_btn = ttk.Button(row, text="Save", command=self.save, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT)
        
        # Content
        content = ttk.Frame(main_frame)
        content.pack(fill=tk.BOTH, expand=True)
        
        # Plot
        plot_frame = ttk.LabelFrame(content, text="Visualization", padding="5")
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.fig = Figure(figsize=(12, 9), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.pack(fill=tk.X)
        NavigationToolbar2Tk(self.canvas, toolbar_frame).update()
        
        # Right panel
        right = ttk.Frame(content, width=420)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        right.pack_propagate(False)
        
        res_frame = ttk.LabelFrame(right, text="Results", padding="10")
        res_frame.pack(fill=tk.X, pady=(0, 10))
        self.results_text = tk.Text(res_frame, height=24, width=48,
                                    font=('Courier', 9), bg='#f0f0f0', relief=tk.FLAT)
        self.results_text.pack(fill=tk.X)
        self.results_text.config(state=tk.DISABLED)
        
        log_frame = ttk.LabelFrame(right, text="Log", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=14, width=48,
                                font=('Courier', 8), bg='#1e1e1e', fg='#00ff00',
                                relief=tk.FLAT)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        self.status_var = tk.StringVar(value="Ready. Select a .BIN file.")
        ttk.Label(main_frame, textvariable=self.status_var,
                  relief=tk.SUNKEN).pack(fill=tk.X, pady=(10, 0))
    
    def browse(self):
        fp = filedialog.askopenfilename(
            title="Select NimBrace Data File",
            filetypes=[("Binary files", "*.BIN *.bin"), ("All files", "*.*")])
        if fp:
            self.file_var.set(fp)
            self.status_var.set(f"Selected: {os.path.basename(fp)}")
    
    def log(self, msg):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.root.update_idletasks()
    
    def clear_log(self):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def update_results(self, result, panjabi, terrain, nz_shift, duration):
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete(1.0, tk.END)
        
        cadence = result['n_steps'] / (duration / 60)
        n_up = np.sum(terrain == 'uphill')
        n_down = np.sum(terrain == 'downhill')
        n_level = np.sum(terrain == 'level')
        
        text = f"""
  SPINE PDR v5 — NADIR ZERO-CYCLE
  ================================

  METHOD: {result['method']}

  PATH:
    Distance:       {result['distance']:8.1f} m
    End from start: {result['end_dist']:8.2f} m
    Closure:        {result['closure_pct']:8.2f} %

  GAIT:
    Steps (nadirs): {result['n_cycles']:8d}
    Cadence:        {cadence:8.1f} steps/min
    Mean stride:    {np.mean(result['strides']):8.3f} m

  HEADING (Zero-Cycle at nadirs):
    Total rotation: {result['total_heading_deg']:8.1f} deg
    Panjabi sigma:  {panjabi['std_heading']:8.2f} deg
    Autocorr lag-1: {panjabi['autocorr_lag1']:8.3f}

  RANDOM WALK:
    Final cumsum:   {panjabi['cumsum_final']:8.1f} deg
    sqrt(n) bound:  +/-{panjabi['sqrt_n_bound']:7.1f} deg
    Within bounds:  {'YES' if abs(panjabi['cumsum_final']) < panjabi['sqrt_n_bound'] else 'NO'}

  TERRAIN (NZ shift from hub):
    Level:    {n_level:5d} steps ({100*n_level/len(terrain):.0f}%)
    Uphill:   {n_up:5d} steps ({100*n_up/len(terrain):.0f}%)
    Downhill: {n_down:5d} steps ({100*n_down/len(terrain):.0f}%)
    NZ spread: {np.std(nz_shift):.2f} deg
"""
        self.results_text.insert(tk.END, text)
        self.results_text.config(state=tk.DISABLED)
    
    def plot_results(self, result, panjabi, terrain, nz_shift, nz_shift_smooth,
                     hub_pitch, hub_roll, hist, pe, re, duration):
        self.fig.clear()
        gs = self.fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3)
        
        x, y = result['x'], result['y']
        n_cycles = result['n_cycles']
        
        # 1. Main path (terrain colored)
        ax1 = self.fig.add_subplot(gs[0, :2])
        for i in range(min(len(x)-1, len(terrain))):
            color = TERRAIN_COLORS.get(terrain[i], '#888888')
            ax1.plot([x[i], x[i+1]], [y[i], y[i+1]], color=color, lw=2, alpha=0.8)
        
        ax1.scatter([0], [0], c='lime', s=200, marker='o', zorder=10,
                    edgecolors='black', linewidths=2, label='START')
        ax1.scatter([x[-1]], [y[-1]], c='red', s=200, marker='*', zorder=10,
                    edgecolors='black', linewidths=2, label='END')
        
        from matplotlib.lines import Line2D
        handles = [
            Line2D([0],[0], color=TERRAIN_COLORS['level'], lw=3, label='Level'),
            Line2D([0],[0], color=TERRAIN_COLORS['uphill'], lw=3, label='Uphill'),
            Line2D([0],[0], color=TERRAIN_COLORS['downhill'], lw=3, label='Downhill'),
        ]
        ax1.legend(handles=handles, loc='upper right', fontsize=8)
        
        ax1.set_title(
            f"Nadir PDR: {result['distance']:.0f}m | "
            f"Closure: {result['end_dist']:.1f}m ({result['closure_pct']:.2f}%) | "
            f"{result['n_cycles']} steps",
            fontweight='bold', fontsize=11)
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        
        # 2. Panjabi NZ histogram
        ax2 = self.fig.add_subplot(gs[0, 2])
        pc = (pe[:-1] + pe[1:]) / 2
        rc = (re[:-1] + re[1:]) / 2
        ax2.imshow(hist.T, origin='lower', aspect='auto',
                   extent=[pc[0], pc[-1], rc[0], rc[-1]], cmap='hot')
        ax2.plot(hub_pitch, hub_roll, 'c*', markersize=15, markeredgecolor='white')
        ax2.set_xlabel('Pitch (deg)')
        ax2.set_ylabel('Roll (deg)')
        ax2.set_title('Panjabi Neutral Zone', fontweight='bold')
        
        # 3. NZ shift timeline (terrain detection)
        ax3 = self.fig.add_subplot(gs[1, :2])
        ax3.plot(nz_shift, color='gray', lw=0.3, alpha=0.4)
        ax3.plot(nz_shift_smooth, 'k-', lw=2)
        ax3.axhline(0, color='green', ls='--', lw=1.5, alpha=0.7)
        ax3.axhline(-2, color='red', ls=':', alpha=0.5, label='Uphill threshold')
        ax3.axhline(2, color='blue', ls=':', alpha=0.5, label='Downhill threshold')
        
        for i in range(len(terrain)):
            if terrain[i] != 'level':
                ax3.axvspan(i-0.5, i+0.5,
                           color=TERRAIN_COLORS[terrain[i]], alpha=0.15)
        
        ax3.set_xlabel('Step #')
        ax3.set_ylabel('NZ shift from hub (deg)')
        ax3.set_title('NZ Terrain Detection', fontweight='bold')
        ax3.legend(fontsize=8)
        ax3.grid(True, alpha=0.3)
        ax3.invert_yaxis()
        
        # 4. Stride distribution
        ax4 = self.fig.add_subplot(gs[1, 2])
        for t, color in TERRAIN_COLORS.items():
            mask = terrain[:n_cycles] == t
            if np.sum(mask) > 0:
                ax4.hist(result['strides'][mask], bins=25, alpha=0.5,
                         color=color, label=f'{t} ({np.sum(mask)})', edgecolor='black')
        ax4.axvline(np.mean(result['strides']), color='black', ls='--', lw=2)
        ax4.set_xlabel('Stride (m)')
        ax4.set_ylabel('Count')
        ax4.set_title(f"Stride: {np.mean(result['strides']):.3f}m +/- "
                      f"{np.std(result['strides']):.3f}m", fontweight='bold')
        ax4.legend(fontsize=7)
        ax4.grid(True, alpha=0.3)
        
        # 5. Per-step heading
        ax5 = self.fig.add_subplot(gs[2, 0])
        ax5.plot(result['headings_deg'], 'b-', lw=0.5, alpha=0.7)
        ax5.axhline(np.mean(result['headings_deg']), color='red', ls='--', lw=1.5)
        ax5.axhline(0, color='green', ls='-', alpha=0.3)
        ax5.set_xlabel('Step #')
        ax5.set_ylabel('Heading change (deg)')
        ax5.set_title(f"Per-Step Heading (sigma={panjabi['std_heading']:.1f} deg)",
                      fontweight='bold')
        ax5.grid(True, alpha=0.3)
        
        # 6. Random walk test
        ax6 = self.fig.add_subplot(gs[2, 1])
        n = np.arange(1, len(panjabi['cumsum']) + 1)
        ax6.plot(n, panjabi['cumsum'], 'b-', lw=1.5, label='Actual')
        sigma = panjabi['std_heading']
        ax6.fill_between(n, -sigma*np.sqrt(n), sigma*np.sqrt(n),
                         alpha=0.2, color='red', label='+/- sigma*sqrt(n)')
        ax6.axhline(0, color='black', lw=1)
        ax6.set_xlabel('Step #')
        ax6.set_ylabel('Cumulative (deg)')
        ax6.set_title('Random Walk Test', fontweight='bold')
        ax6.legend(fontsize=8)
        ax6.grid(True, alpha=0.3)
        
        # 7. Info
        ax7 = self.fig.add_subplot(gs[2, 2])
        ax7.axis('off')
        
        n_up = np.sum(terrain == 'uphill')
        n_down = np.sum(terrain == 'downhill')
        
        info = f"""
NADIR ZERO-CYCLE v5

STEP DETECTION:
  HS from thorax accel
  Nadir = closest to hub
  per HS-to-HS interval

STRIDE:
  diff_accel_mag (35cm ruler)
  spine deformation integral

TERRAIN (NZ shift):
  Hub pitch:  {hub_pitch:.1f} deg
  NZ spread:  {np.std(nz_shift):.2f} deg
  Uphill:     {n_up} steps
  Downhill:   {n_down} steps

{result['method']}
"""
        ax7.text(0.5, 0.5, info, transform=ax7.transAxes, fontsize=8,
                 va='center', ha='center', fontfamily='monospace',
                 bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
        
        self.canvas.draw()
    
    def save(self):
        fp = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG", "*.png"), ("PDF", "*.pdf")],
            initialfile="spine_pdr_v5_nadir.png")
        if fp:
            self.fig.savefig(fp, dpi=150, bbox_inches='tight', facecolor='white')
            messagebox.showinfo("Saved", fp)
    
    def process(self):
        fp = self.file_var.get()
        if not fp or not os.path.exists(fp):
            messagebox.showwarning("File", "Select a valid .BIN file")
            return
        
        known_dist = None
        ds = self.dist_var.get().strip()
        if ds:
            try:
                known_dist = float(ds)
            except ValueError:
                pass
        
        user_height = None
        hs = self.height_var.get().strip()
        if hs:
            try:
                user_height = float(hs)
            except ValueError:
                pass
        
        self.process_btn.config(state=tk.DISABLED)
        self.save_btn.config(state=tk.DISABLED)
        self.status_var.set("Processing...")
        self.clear_log()
        
        threading.Thread(target=self._run, 
                         args=(fp, known_dist, user_height)).start()
    
    def _run(self, filepath, known_dist, user_height):
        try:
            self.log(f"Loading: {os.path.basename(filepath)}")
            data = load_nimbrace_bin(filepath, self.log)
            
            timestamps = data['timestamps']
            dt = np.median(np.diff(timestamps)) / 1000.0
            sample_rate = 1.0 / dt
            duration = (timestamps[-1] - timestamps[0]) / 1000.0
            
            self.log("\nExtracting pelvis orientation...")
            pitch = np.array([np.degrees(quat_to_euler(q)[1]) 
                             for q in data['pelvis']['quats']])
            roll = np.array([np.degrees(quat_to_euler(q)[0]) 
                            for q in data['pelvis']['quats']])
            
            self.log("\nFinding Panjabi Neutral Zone hub...")
            hub_pitch, hub_roll, hist, pe, re = find_hub(pitch, roll)
            self.log(f"  Hub: ({hub_pitch:.1f} deg, {hub_roll:.1f} deg)")
            
            self.log("\nDetecting heel strikes (thorax)...")
            hs_idx = detect_heel_strikes(data['thorax']['accels'], 
                                         sample_rate, self.log)
            
            self.log("\nFinding nadirs...")
            nadir_idx, nadir_pitches, nadir_rolls, nadir_dists = \
                detect_nadirs(hs_idx, pitch, roll, hub_pitch, hub_roll, self.log)
            
            self.log("\nTerrain classification (NZ pitch shift)...")
            terrain, nz_shift, nz_shift_smooth = \
                classify_terrain_nz(nadir_pitches, hub_pitch, log_callback=self.log)
            
            self.log("\nComputing stride (diff accel magnitude)...")
            excursions = compute_diff_accel_mag(
                data['pelvis']['accels'], data['thorax']['accels'],
                nadir_idx, sample_rate)
            self.log(f"  Mean excursion: {np.mean(excursions):.4f}")
            self.log(f"  CV: {np.std(excursions)/np.mean(excursions):.3f}")
            
            self.log("\nWeinberg cross-check (thorax)...")
            weinberg = compute_weinberg_thorax(
                data['thorax']['accels'], nadir_idx)
            self.log(f"  Mean Weinberg: {np.mean(weinberg):.4f}")
            self.log(f"  CV: {np.std(weinberg)/np.mean(weinberg):.3f}")
            
            self.log(f"\nRunning PDR (known_dist={known_dist}, height={user_height})...")
            result = zero_cycle_pdr(
                data['pelvis']['quats'], nadir_idx, excursions,
                known_distance=known_dist, user_height=user_height)
            
            panjabi = analyze_panjabi(result['headings_deg'])
            
            self.log(f"\n{'='*45}")
            self.log(f"RESULTS:")
            self.log(f"  Distance: {result['distance']:.1f}m")
            self.log(f"  Closure:  {result['end_dist']:.2f}m ({result['closure_pct']:.2f}%)")
            self.log(f"  Steps:    {result['n_cycles']}")
            self.log(f"  Method:   {result['method']}")
            
            terrain_trimmed = terrain[:result['n_cycles']]
            nz_shift_trimmed = nz_shift[:result['n_cycles']]
            nz_smooth_trimmed = nz_shift_smooth[:result['n_cycles']]
            
            self.root.after(0, lambda: self.update_results(
                result, panjabi, terrain_trimmed, nz_shift_trimmed, duration))
            self.root.after(0, lambda: self.plot_results(
                result, panjabi, terrain_trimmed, 
                nz_shift_trimmed, nz_smooth_trimmed,
                hub_pitch, hub_roll, hist, pe, re, duration))
            self.root.after(0, lambda: self.save_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.status_var.set(
                f"{result['distance']:.0f}m | "
                f"Closure: {result['closure_pct']:.2f}% | "
                f"{result['n_cycles']} steps"))
            
            self.results = result
            
        except Exception as e:
            self.log(f"\nERROR: {str(e)}")
            import traceback
            self.log(traceback.format_exc())
            self.root.after(0, lambda: messagebox.showerror("Error", str(e)))
            self.root.after(0, lambda: self.status_var.set("Error"))
        finally:
            self.root.after(0, lambda: self.process_btn.config(state=tk.NORMAL))


def main():
    root = tk.Tk()
    app = SpinePDRApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
