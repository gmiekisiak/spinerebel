"""
TARS PDR V17 - QUATERNION-CENTERED ZERO-CYCLE
==============================================
Pure Quaternion Heading - No Euler Angles

ZERO-CYCLE PRINCIPLE:
    At each heel strike, center the quaternion to identity.
    Measure relative rotation from that reference.
    No gimbal lock. No unwrap. No Euler hacks.

THE MATH:
    Q_relative = Q_current âŠ— Q_referenceâ»Â¹
    
    At heel strike N:
        Q_ref = Q[HS_N]
    
    At heel strike N+1:
        Q_rel = Q[HS_N+1] âŠ— Q_refâ»Â¹
        heading_change = extract_yaw(Q_rel)

WHY THIS IS BETTER:
    - Quaternion math is gimbal-lock free
    - Each cycle starts from identity (0,0,0,1)
    - Relative rotation is bounded to single cycle
    - No accumulated Euler artifacts

THE FORMULA:
    stride = 2 Ã— L_eff Ã— sin(Î¸)

NO USER INPUT. NO CALIBRATION. PURE PHYSICS.

Author: Grzegorz Miekisiak, MD PhD / SpineRebel Technology
(c) 2025-2026 SpineRebel Technology — Patent Protected
Academic and non-commercial use only. See LICENSE.
"""

import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import os


# =============================================================================
# DATA LOADING
# =============================================================================

def load_bin_file(filepath, log_callback=None):
    """Load BNO085 binary data file with dual IMU support."""
    def log(msg):
        if log_callback:
            log_callback(msg)
    
    log(f"Loading: {os.path.basename(filepath)}")
    
    with open(filepath, 'rb') as f:
        data = f.read()
    
    header_size = 86
    packet_size = 92
    data_section = data[header_size:]
    num_packets = len(data_section) // packet_size
    
    timestamps = []
    imu1 = {'quats': [], 'lin_accels': [], 'gyros': []}
    imu2 = {'quats': [], 'lin_accels': [], 'gyros': []}
    
    for i in range(num_packets):
        offset = i * packet_size
        chunk = data_section[offset:offset + packet_size]
        
        ts = struct.unpack('<I', chunk[0:4])[0]
        floats = struct.unpack('<22f', chunk[4:92])
        
        timestamps.append(ts)
        
        # IMU1: indices 0-9
        imu1['quats'].append(floats[0:4])
        imu1['lin_accels'].append(floats[4:7])
        imu1['gyros'].append(floats[7:10])
        
        # IMU2: indices 10-19
        imu2['quats'].append(floats[10:14])
        imu2['lin_accels'].append(floats[14:17])
        imu2['gyros'].append(floats[17:20])
    
    result = {
        'timestamps': np.array(timestamps),
        'imu1': {k: np.array(v) for k, v in imu1.items()},
        'imu2': {k: np.array(v) for k, v in imu2.items()}
    }
    
    duration = (timestamps[-1] - timestamps[0]) / 1000.0
    dt = np.median(np.diff(timestamps)) / 1000.0
    
    log(f"  Samples: {num_packets}")
    log(f"  Duration: {duration:.1f}s")
    log(f"  Sample Rate: {1/dt:.1f} Hz")
    log(f"  Dual IMU: Yes")
    
    return result


# =============================================================================
# QUATERNION UTILITIES - THE CORE
# =============================================================================

def quat_conjugate(q):
    """
    Quaternion conjugate (inverse for unit quaternion).
    q = [w, x, y, z]
    q* = [w, -x, -y, -z]
    """
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_multiply(q1, q2):
    """
    Quaternion multiplication: q1 âŠ— q2
    Hamilton convention: [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return np.array([w, x, y, z])


def quat_normalize(q):
    """Normalize quaternion to unit length."""
    norm = np.linalg.norm(q)
    if norm < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quat_to_yaw(q):
    """
    Extract yaw (heading) from quaternion.
    This is the rotation around world Z axis.
    """
    w, x, y, z = q
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def quat_relative_yaw(q_current, q_reference):
    """
    ZERO-CYCLE QUATERNION HEADING
    
    Compute heading change from reference to current.
    
    Q_relative = Q_current âŠ— Q_referenceâ»Â¹
    
    This gives rotation FROM reference TO current.
    Then extract yaw from relative quaternion.
    
    NO EULER ANGLES IN THE CHAIN.
    """
    q_ref_inv = quat_conjugate(q_reference)
    q_relative = quat_multiply(q_current, q_ref_inv)
    q_relative = quat_normalize(q_relative)
    
    # Extract yaw from relative quaternion
    # This yaw is the heading CHANGE, not absolute heading
    yaw = quat_to_yaw(q_relative)
    
    return yaw, q_relative


def quat_to_pitch(quats):
    """Extract pitch angles from quaternion array."""
    w = quats[:, 0]
    x = quats[:, 1]
    y = quats[:, 2]
    z = quats[:, 3]
    sinp = 2 * (w * y - z * x)
    return np.degrees(np.arcsin(np.clip(sinp, -1, 1)))


def get_gravity_vector(q):
    """
    Rotate global Z vector [0,0,1] by inverse quaternion 
    to get gravity vector in sensor frame.
    """
    w, x, y, z = q
    gx = 2 * (x * z - w * y)
    gy = 2 * (y * z + w * x)
    gz = 1 - 2 * (x * x + y * y)
    return np.array([gx, gy, gz])


# =============================================================================
# IMU VALIDATION
# =============================================================================

def validate_imu(imu_data, name, log_callback=None):
    """Check if IMU data is valid (not glitching)."""
    def log(msg):
        if log_callback:
            log_callback(msg)
    
    gyros = imu_data['gyros']
    accels = imu_data['lin_accels']
    
    nan_gyro = np.sum(np.isnan(gyros))
    nan_accel = np.sum(np.isnan(accels))
    gyro_var = np.var(gyros, axis=0)
    accel_var = np.var(accels, axis=0)
    gyro_mag = np.linalg.norm(gyros, axis=1)
    
    issues = []
    
    if nan_gyro > 0 or nan_accel > 0:
        issues.append(f"NaN values: gyro={nan_gyro}, accel={nan_accel}")
    if np.any(gyro_var < 0.01):
        issues.append("Gyro stuck (low variance)")
    if np.any(accel_var < 0.01):
        issues.append("Accel stuck (low variance)")
    if np.mean(gyro_mag) < 0.1:
        issues.append("Gyro magnitude too low")
    
    valid = len(issues) == 0
    
    if valid:
        log(f"  {name}: Valid âœ“")
    else:
        log(f"  {name}: ISSUES - {', '.join(issues)}")
    
    return valid, issues


# =============================================================================
# PCA SAGITTAL ALIGNMENT
# =============================================================================

def get_sagittal_axis(gyros, log_callback=None):
    """Find sagittal plane via PCA."""
    gyros_centered = gyros - np.mean(gyros, axis=0)
    cov = np.cov(gyros_centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    
    idx = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    sagittal_axis = eigenvectors[:, 0]
    variance_explained = eigenvalues[0] / np.sum(eigenvalues)
    
    return sagittal_axis, variance_explained


# =============================================================================
# GAIT EVENT DETECTION
# =============================================================================

def detect_gait_events(quats, log_callback=None):
    """Detect toe-off and heel-strike events from quaternion pitch."""
    pitch = quat_to_pitch(quats)
    pitch_smooth = gaussian_filter1d(pitch, sigma=5)
    
    toe_offs, _ = find_peaks(pitch_smooth, distance=40, prominence=2)
    heel_strikes, _ = find_peaks(-pitch_smooth, distance=40, prominence=2)
    
    return toe_offs, heel_strikes, pitch_smooth


# =============================================================================
# DUAL CENTRIPETAL EXTRACTION
# =============================================================================

def extract_dual_centripetal(sagittal_gyro, a_mag, pitch_smooth, toe_offs, heel_strikes, 
                              r_max_stance=1.0, r_max_swing=0.5, log_callback=None):
    """Extract r_midstance and r_midswing using PCA sagittal axis."""
    pitch_mean = np.mean(pitch_smooth)
    pitch_centered = pitch_smooth - pitch_mean
    
    all_events = [(to, 'TO') for to in toe_offs] + [(hs, 'HS') for hs in heel_strikes]
    all_events.sort(key=lambda x: x[0])
    
    r_stance_vals = []
    for i in range(len(all_events) - 1):
        if all_events[i][1] == 'HS' and all_events[i+1][1] == 'TO':
            hs, to = all_events[i][0], all_events[i+1][0]
            stance_pitch = pitch_centered[hs:to]
            for j in range(1, len(stance_pitch)):
                if stance_pitch[j-1] < 0 and stance_pitch[j] >= 0:
                    idx = hs + j
                    window = 3
                    idx_start = max(0, idx - window)
                    idx_end = min(len(sagittal_gyro), idx + window)
                    
                    w = np.abs(sagittal_gyro[idx_start:idx_end])
                    a = a_mag[idx_start:idx_end]
                    
                    valid = w > 0.3
                    if np.sum(valid) > 2:
                        r_est = a[valid] / (w[valid] ** 2)
                        r_clean = r_est[(r_est > 0.05) & (r_est < r_max_stance)]
                        if len(r_clean) > 0:
                            r_stance_vals.append(np.median(r_clean))
                    break
    
    r_swing_vals = []
    for i in range(len(all_events) - 1):
        if all_events[i][1] == 'TO' and all_events[i+1][1] == 'HS':
            to, hs = all_events[i][0], all_events[i+1][0]
            swing_pitch = pitch_centered[to:hs]
            for j in range(1, len(swing_pitch)):
                if swing_pitch[j-1] > 0 and swing_pitch[j] <= 0:
                    idx = to + j
                    window = 3
                    idx_start = max(0, idx - window)
                    idx_end = min(len(sagittal_gyro), idx + window)
                    
                    w = np.abs(sagittal_gyro[idx_start:idx_end])
                    a = a_mag[idx_start:idx_end]
                    
                    valid = w > 0.3
                    if np.sum(valid) > 2:
                        r_est = a[valid] / (w[valid] ** 2)
                        r_clean = r_est[(r_est > 0.05) & (r_est < r_max_swing)]
                        if len(r_clean) > 0:
                            r_swing_vals.append(np.median(r_clean))
                    break
    
    r_stance = np.median(r_stance_vals) if len(r_stance_vals) > 0 else None
    r_swing = np.median(r_swing_vals) if len(r_swing_vals) > 0 else None
    
    return r_stance, r_swing, len(r_stance_vals), len(r_swing_vals)


# =============================================================================
# TEARDROP EXTRACTION
# =============================================================================

def extract_teardrops(gyro_mag, heel_strikes, dt, log_callback=None):
    """Extract teardrop angles from gyro magnitude."""
    teardrops = []
    valid_cycles = []
    
    for i in range(len(heel_strikes) - 1):
        hs1 = heel_strikes[i]
        hs2 = heel_strikes[i + 1]
        
        if 20 < hs2 - hs1 < 200:
            theta = np.sum(gyro_mag[hs1:hs2]) * dt / 2.0
            teardrops.append(theta)
            valid_cycles.append(i)
    
    return np.array(teardrops), valid_cycles


# =============================================================================
# ZERO-CYCLE QUATERNION HEADING - THE NEW WAY
# =============================================================================

def extract_headings_quaternion_centered(quats, heel_strikes, log_callback=None):
    """
    ZERO-CYCLE QUATERNION HEADING
    =============================
    
    At each heel strike:
        1. Take Q_reference = Q[heel_strike_N]
        2. Take Q_current = Q[heel_strike_N+1]
        3. Compute Q_relative = Q_current âŠ— Q_referenceâ»Â¹
        4. Extract yaw from Q_relative
    
    This is PURE QUATERNION MATH.
    No Euler angles until the final yaw extraction.
    No gimbal lock in the rotation composition.
    
    The yaw extraction from Q_relative is safe because
    the relative rotation is small (~single gait cycle).
    """
    def log(msg):
        if log_callback:
            log_callback(msg)
    
    headings = []
    q_relatives = []
    
    for i in range(len(heel_strikes) - 1):
        hs1 = heel_strikes[i]
        hs2 = heel_strikes[i + 1]
        
        # Reference quaternion at start of cycle
        q_ref = quats[hs1]
        
        # Current quaternion at end of cycle  
        q_cur = quats[hs2]
        
        # Compute relative rotation
        yaw_change, q_rel = quat_relative_yaw(q_cur, q_ref)
        
        # Normalize to [-Ï€, +Ï€] - bounds drift to single cycle
        while yaw_change > np.pi:
            yaw_change -= 2 * np.pi
        while yaw_change < -np.pi:
            yaw_change += 2 * np.pi
        
        headings.append(yaw_change)
        q_relatives.append(q_rel)
    
    headings = np.array(headings)
    
    if log_callback:
        total = np.rad2deg(np.sum(headings))
        mean_per_cycle = np.rad2deg(np.mean(headings))
        log(f"  Quaternion-centered heading:")
        log(f"    Total rotation: {total:.1f}Â°")
        log(f"    Mean per cycle: {mean_per_cycle:.3f}Â°")
        log(f"    Cycles: {len(headings)}")
    
    return headings, q_relatives


# =============================================================================
# DRIFT MEASUREMENT (GRAVITY-PROJECTED - FOR REPORTING)
# =============================================================================

def measure_drift_gravity_projected(quats, gyros, toe_offs, heel_strikes, dt, log_callback=None):
    """
    DRIFT MEASUREMENT using gravity-projected angular velocity.
    Used for REPORTING only, not for PDR path.
    """
    def log(msg):
        if log_callback:
            log_callback(msg)
    
    # Calculate gravity-projected heading rate
    heading_rates = []
    for i in range(len(quats)):
        g_vec = get_gravity_vector(quats[i])
        w_vec = gyros[i]
        h_rate = np.dot(w_vec, g_vec)
        heading_rates.append(h_rate)
    
    heading_rates = np.array(heading_rates)
    yaw_grav = np.cumsum(heading_rates) * dt
    
    # Build event sequence
    all_events = [(to, 'TO') for to in toe_offs] + [(hs, 'HS') for hs in heel_strikes]
    all_events.sort(key=lambda x: x[0])
    
    stance_yaw = []
    swing_yaw = []
    stance_duration = []
    swing_duration = []
    
    for i in range(len(all_events) - 1):
        idx1, event1 = all_events[i]
        idx2, event2 = all_events[i + 1]
        
        delta = yaw_grav[idx2] - yaw_grav[idx1]
        duration = (idx2 - idx1) * dt
        
        if event1 == 'HS' and event2 == 'TO':
            stance_yaw.append(delta)
            stance_duration.append(duration)
        elif event1 == 'TO' and event2 == 'HS':
            swing_yaw.append(delta)
            swing_duration.append(duration)
    
    stance_yaw = np.array(stance_yaw)
    swing_yaw = np.array(swing_yaw)
    swing_duration = np.array(swing_duration)
    
    min_len = min(len(stance_yaw), len(swing_yaw))
    
    # Drift metrics from swing phase
    mean_swing_yaw = np.mean(swing_yaw[:min_len])
    mean_swing_duration = np.mean(swing_duration[:min_len])
    drift_rate = mean_swing_yaw / mean_swing_duration if mean_swing_duration > 0 else 0
    
    if log_callback:
        log(f"  Gravity-projected drift measurement:")
        log(f"    Mean swing yaw:  {np.rad2deg(mean_swing_yaw):.3f}Â°")
        log(f"    Mean swing dur:  {mean_swing_duration*1000:.0f} ms")
        log(f"    Drift rate:      {np.rad2deg(drift_rate):.3f}Â°/s")
    
    return {
        'stance_yaw': stance_yaw[:min_len],
        'swing_yaw': swing_yaw[:min_len],
        'swing_duration': swing_duration[:min_len],
        'mean_swing_yaw': mean_swing_yaw,
        'mean_swing_duration': mean_swing_duration,
        'drift_rate': drift_rate,
        'yaw_grav': yaw_grav
    }


# =============================================================================
# DUAL IMU FUSION ENGINE - V17 QUATERNION-CENTERED
# =============================================================================

def fuse_dual_imu(data, log_callback=None):
    """
    DUAL IMU FUSION ENGINE - V17
    
    QUATERNION-CENTERED ZERO-CYCLE HEADING
    
    No Euler angles in the heading chain.
    Pure quaternion math for rotation composition.
    """
    def log(msg):
        if log_callback:
            log_callback(msg)
    
    timestamps = data['timestamps']
    dt = np.median(np.diff(timestamps)) / 1000.0
    
    imu1 = data['imu1']
    imu2 = data['imu2']
    
    log("="*50)
    log("TARS PDR V17 - QUATERNION-CENTERED ZERO-CYCLE")
    log("="*50)
    
    # =========================================
    # 1. VALIDATE BOTH IMUs
    # =========================================
    log("\n[1] IMU VALIDATION")
    imu1_valid, _ = validate_imu(imu1, "IMU1", log)
    imu2_valid, _ = validate_imu(imu2, "IMU2", log)
    
    if not imu1_valid and not imu2_valid:
        raise ValueError("Both IMUs have issues!")
    
    # =========================================
    # 2. GAIT EVENTS
    # =========================================
    log("\n[2] GAIT EVENTS")
    primary_imu = imu1 if imu1_valid else imu2
    primary_name = "IMU1" if imu1_valid else "IMU2"
    
    toe_offs, heel_strikes, pitch_smooth = detect_gait_events(primary_imu['quats'], log)
    log(f"  Using {primary_name} for gait events")
    log(f"  Heel strikes: {len(heel_strikes)}")
    log(f"  Toe offs: {len(toe_offs)}")
    
    # =========================================
    # 3. DUAL CENTRIPETAL EXTRACTION
    # =========================================
    log("\n[3] DUAL CENTRIPETAL (r = a/Ï‰Â²)")
    
    results = {'imu1': {}, 'imu2': {}}
    
    for name, imu, valid in [('imu1', imu1, imu1_valid), ('imu2', imu2, imu2_valid)]:
        if not valid:
            results[name] = {'valid': False}
            continue
        
        sagittal_axis, var_exp = get_sagittal_axis(imu['gyros'])
        sagittal_gyro = np.dot(imu['gyros'], sagittal_axis)
        a_mag = np.linalg.norm(imu['lin_accels'], axis=1)
        
        r_stance, r_swing, n_stance, n_swing = extract_dual_centripetal(
            sagittal_gyro, a_mag, pitch_smooth, toe_offs, heel_strikes
        )
        
        if r_stance is not None and r_swing is not None:
            L_eff = r_stance + r_swing
            log(f"  {name.upper()}: r_stance={r_stance:.3f}m, r_swing={r_swing:.3f}m, L_eff={L_eff:.3f}m")
            results[name] = {
                'valid': True, 'r_stance': r_stance, 'r_swing': r_swing,
                'L_eff': L_eff, 'n_stance': n_stance, 'n_swing': n_swing
            }
        else:
            results[name] = {'valid': False}
    
    # =========================================
    # 4. CROSS-VALIDATE r VALUES
    # =========================================
    log("\n[4] CROSS-VALIDATION")
    
    if results['imu1']['valid'] and results['imu2']['valid']:
        r_diff = abs(results['imu1']['L_eff'] - results['imu2']['L_eff'])
        if r_diff < 0.10:
            L_eff = (results['imu1']['L_eff'] + results['imu2']['L_eff']) / 2
            r_stance = (results['imu1']['r_stance'] + results['imu2']['r_stance']) / 2
            r_swing = (results['imu1']['r_swing'] + results['imu2']['r_swing']) / 2
            fusion_method = "averaged"
        else:
            n1 = results['imu1']['n_stance'] + results['imu1']['n_swing']
            n2 = results['imu2']['n_stance'] + results['imu2']['n_swing']
            if n1 >= n2:
                L_eff, r_stance, r_swing = results['imu1']['L_eff'], results['imu1']['r_stance'], results['imu1']['r_swing']
                fusion_method = "imu1_preferred"
            else:
                L_eff, r_stance, r_swing = results['imu2']['L_eff'], results['imu2']['r_stance'], results['imu2']['r_swing']
                fusion_method = "imu2_preferred"
    elif results['imu1']['valid']:
        L_eff, r_stance, r_swing = results['imu1']['L_eff'], results['imu1']['r_stance'], results['imu1']['r_swing']
        fusion_method = "imu1_only"
    else:
        L_eff, r_stance, r_swing = results['imu2']['L_eff'], results['imu2']['r_stance'], results['imu2']['r_swing']
        fusion_method = "imu2_only"
    
    log(f"  â†’ Fused L_eff: {L_eff:.3f} m")
    
    # =========================================
    # 5. TEARDROP EXTRACTION
    # =========================================
    log("\n[5] TEARDROP ANGLES")
    
    teardrops_list = []
    for name, imu, valid in [('imu1', imu1, imu1_valid), ('imu2', imu2, imu2_valid)]:
        if not valid:
            continue
        gyro_mag = np.linalg.norm(imu['gyros'], axis=1)
        teardrops, _ = extract_teardrops(gyro_mag, heel_strikes, dt)
        teardrops_list.append(teardrops)
        log(f"  {name.upper()}: {np.rad2deg(np.mean(teardrops)):.1f}Â° mean")
    
    if len(teardrops_list) == 2:
        min_len = min(len(teardrops_list[0]), len(teardrops_list[1]))
        teardrops = (teardrops_list[0][:min_len] + teardrops_list[1][:min_len]) / 2
    else:
        teardrops = teardrops_list[0]
    
    # =========================================
    # 6. QUATERNION-CENTERED HEADING (THE NEW WAY)
    # =========================================
    log("\n[6] QUATERNION-CENTERED HEADING (Zero-Cycle)")
    
    headings_list = []
    closures = []
    
    for name, imu, valid in [('imu1', imu1, imu1_valid), ('imu2', imu2, imu2_valid)]:
        if not valid:
            continue
        
        log(f"  {name.upper()}:")
        headings, q_relatives = extract_headings_quaternion_centered(imu['quats'], heel_strikes, log)
        headings_list.append((name, headings, q_relatives))
        
        # Quick closure test
        min_len = min(len(headings), len(teardrops))
        h = headings[:min_len]
        s = 2 * L_eff * np.sin(teardrops[:min_len])
        
        cumulative = np.cumsum(h)
        cumulative = np.insert(cumulative, 0, 0)
        x, y = 0.0, 0.0
        for i in range(len(s)):
            angle = (cumulative[i] + cumulative[i+1]) / 2
            x += s[i] * np.cos(angle)
            y += s[i] * np.sin(angle)
        
        closure = np.sqrt(x**2 + y**2)
        closures.append((name, closure))
        log(f"    Closure: {closure:.2f}m")
    
    best_imu = min(closures, key=lambda x: x[1])[0]
    headings = [h for n, h, q in headings_list if n == best_imu][0]
    q_relatives = [q for n, h, q in headings_list if n == best_imu][0]
    log(f"  â†’ Using {best_imu.upper()} for PDR")
    
    # =========================================
    # 7. DRIFT MEASUREMENT (GRAVITY-PROJECTED)
    # =========================================
    log("\n[7] DRIFT MEASUREMENT (Gravity-projected)")
    
    best_imu_data = imu1 if best_imu == 'imu1' else imu2
    drift_info = measure_drift_gravity_projected(
        best_imu_data['quats'], best_imu_data['gyros'],
        toe_offs, heel_strikes, dt, log
    )
    
    # =========================================
    # 8. COMPUTE STRIDES & BUILD PATH
    # =========================================
    log("\n[8] PATH RECONSTRUCTION")
    
    strides = 2 * L_eff * np.sin(teardrops)
    
    min_len = min(len(headings), len(strides))
    headings = headings[:min_len]
    strides = strides[:min_len]
    
    cumulative_heading = np.cumsum(headings)
    cumulative_heading = np.insert(cumulative_heading, 0, 0)
    
    x, y = [0.0], [0.0]
    for i in range(len(strides)):
        h = (cumulative_heading[i] + cumulative_heading[i + 1]) / 2
        x.append(x[-1] + strides[i] * np.cos(h))
        y.append(y[-1] + strides[i] * np.sin(h))
    
    x, y = np.array(x), np.array(y)
    
    # =========================================
    # 9. RESULTS
    # =========================================
    total_distance = np.sum(strides)
    closure = np.sqrt(x[-1]**2 + y[-1]**2)
    closure_pct = 100 * closure / total_distance if total_distance > 0 else 0
    height = (r_stance + 0.40) / 0.55
    
    log("\n" + "="*50)
    log("RESULTS")
    log("="*50)
    log(f"  Distance:      {total_distance:.1f} m")
    log(f"  Closure:       {closure:.2f} m ({closure_pct:.2f}%)")
    log(f"  Height:        {height:.2f} m")
    log(f"  Drift rate:    {np.rad2deg(drift_info['drift_rate']):.3f}Â°/s")
    
    return {
        'x': x, 'y': y,
        'strides': strides,
        'headings': headings,
        'teardrops': teardrops,
        'L_eff': L_eff,
        'r_stance': r_stance,
        'r_swing': r_swing,
        'height': height,
        'leg_length': height * 0.53,
        'total_distance': total_distance,
        'closure': closure,
        'closure_pct': closure_pct,
        'n_cycles': len(strides),
        'mean_theta': np.rad2deg(np.mean(teardrops)),
        'fusion_method': fusion_method,
        'imu1_valid': imu1_valid,
        'imu2_valid': imu2_valid,
        'cumulative_heading': cumulative_heading,
        'q_relatives': q_relatives,
        # Drift info (gravity-projected)
        'drift_rate': drift_info['drift_rate'],
        'mean_swing_yaw': drift_info['mean_swing_yaw'],
        'mean_swing_duration': drift_info['mean_swing_duration'],
        'swing_yaw': drift_info['swing_yaw'],
        'stance_yaw': drift_info['stance_yaw'],
    }


# =============================================================================
# GUI APPLICATION
# =============================================================================

class TarsPDRApp:
    def __init__(self, root):
        self.root = root
        self.root.title("TARS PDR V17 - Quaternion-Centered Zero-Cycle")
        self.root.geometry("1500x950")
        self.root.minsize(1300, 850)
        
        style = ttk.Style()
        style.configure('Title.TLabel', font=('Helvetica', 16, 'bold'))
        
        self.create_widgets()
        self.results = None
        
    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, pady=(0, 10))
        
        title_label = ttk.Label(
            top_frame, 
            text="TARS PDR V17 - QUATERNION-CENTERED ZERO-CYCLE",
            style='Title.TLabel'
        )
        title_label.pack(side=tk.LEFT)
        
        subtitle_label = ttk.Label(
            top_frame,
            text="  Q_rel = Q_cur âŠ— Q_refâ»Â¹ â€¢ No Euler â€¢ Pure Quaternion Math",
            font=('Helvetica', 10, 'italic'),
            foreground='gray'
        )
        subtitle_label.pack(side=tk.LEFT, padx=(10, 0))
        
        # File selection
        file_frame = ttk.LabelFrame(main_frame, text="Data File", padding="10")
        file_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.file_path_var = tk.StringVar()
        self.file_entry = ttk.Entry(file_frame, textvariable=self.file_path_var, width=80)
        self.file_entry.pack(side=tk.LEFT, padx=(0, 10), fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(file_frame, text="Browse...", command=self.browse_file)
        browse_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.process_btn = ttk.Button(file_frame, text="Process", command=self.process_file)
        self.process_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.save_btn = ttk.Button(file_frame, text="Save PNG", command=self.save_plot, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT)
        
        # Content
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # Plot
        plot_frame = ttk.LabelFrame(content_frame, text="PDR Visualization", padding="5")
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.fig = Figure(figsize=(12, 9), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        
        # Results panel
        right_frame = ttk.Frame(content_frame, width=420)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        right_frame.pack_propagate(False)
        
        results_frame = ttk.LabelFrame(right_frame, text="Results", padding="10")
        results_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.results_text = tk.Text(
            results_frame, height=24, width=48, 
            font=('Courier', 9), bg='#f5f5f5', relief=tk.FLAT
        )
        self.results_text.pack(fill=tk.X)
        self.results_text.config(state=tk.DISABLED)
        
        log_frame = ttk.LabelFrame(right_frame, text="Processing Log", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = tk.Text(
            log_frame, height=15, width=48,
            font=('Courier', 8), bg='#1e1e1e', fg='#00ff00', relief=tk.FLAT
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        log_scroll = ttk.Scrollbar(self.log_text, command=self.log_text.yview)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=log_scroll.set)
        
        self.status_var = tk.StringVar(value="Ready. Select a .BIN file to begin.")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, relief=tk.SUNKEN)
        status_bar.pack(fill=tk.X, pady=(10, 0))
        
        self.show_welcome_plot()
        
    def show_welcome_plot(self):
        self.fig.clear()
        ax = self.fig.add_subplot(111)
        
        welcome_text = """
TARS PDR V17 - QUATERNION-CENTERED ZERO-CYCLE
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

THE ZERO-CYCLE QUATERNION METHOD:

    At each heel strike N:
        Q_reference = Q[HS_N]
    
    At heel strike N+1:
        Q_relative = Q[HS_N+1] âŠ— Q_referenceâ»Â¹
        heading_change = yaw(Q_relative)

WHY THIS IS BETTER:
    âœ“ Pure quaternion math (no Euler in rotation chain)
    âœ“ Gimbal-lock immune
    âœ“ Each cycle starts from identity
    âœ“ Relative rotation bounded to single cycle
    âœ“ No np.unwrap hacks needed

THE FORMULA:
    stride = 2 Ã— L_eff Ã— sin(Î¸)

NO EULER ANGLES. NO GIMBAL LOCK. PURE PHYSICS.

Select a .BIN file and click 'Process'
"""
        
        ax.text(0.5, 0.5, welcome_text, ha='center', va='center',
                fontsize=10, fontfamily='monospace', transform=ax.transAxes,
                bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9))
        ax.axis('off')
        self.canvas.draw()
        
    def browse_file(self):
        filepath = filedialog.askopenfilename(
            title="Select NimBrace Data File",
            filetypes=[("Binary files", "*.BIN *.bin"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~")
        )
        if filepath:
            self.file_path_var.set(filepath)
            self.status_var.set(f"Selected: {os.path.basename(filepath)}")
            
    def log(self, message):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.root.update_idletasks()
        
    def clear_log(self):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        
    def update_results(self, results):
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete(1.0, tk.END)
        
        imu_status = "Both IMUs âœ“" if results['imu1_valid'] and results['imu2_valid'] else \
                     "IMU1 only" if results['imu1_valid'] else "IMU2 only"
        
        drift_rate = np.rad2deg(results['drift_rate'])
        swing_yaw = np.rad2deg(results['mean_swing_yaw'])
        swing_dur = results['mean_swing_duration'] * 1000
        
        text = f"""
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
   QUATERNION-CENTERED ZERO-CYCLE
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

  Status:          {imu_status}
  Fusion:          {results['fusion_method']}

â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
       HEADING (Pure Quaternion)
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

  Method:          Q_rel = Q_cur âŠ— Q_refâ»Â¹
  Total rotation:  {np.rad2deg(np.sum(results['headings'])):.1f}Â°
  Mean per cycle:  {np.rad2deg(np.mean(results['headings'])):.3f}Â°

â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
       DRIFT MEASUREMENT (Gravity)
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

  Mean swing yaw:  {swing_yaw:.3f}Â°
  Swing duration:  {swing_dur:.0f} ms
  
  â–¶ Drift rate:    {drift_rate:.3f}Â°/s

â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
          DUAL CENTRIPETAL
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

  r_midstance:     {results['r_stance']:.3f} m
  r_midswing:      {results['r_swing']:.3f} m
  
  â–¶ L_eff:         {results['L_eff']:.3f} m
  â–¶ Height:        {results['height']:.2f} m

â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
            PATH RESULTS
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

  Gait Cycles:     {results['n_cycles']}
  Mean Stride:     {np.mean(results['strides']):.3f} m
  
  â–¶ Distance:      {results['total_distance']:.1f} m
  â–¶ Closure:       {results['closure']:.2f} m ({results['closure_pct']:.2f}%)

â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
"""
        self.results_text.insert(tk.END, text)
        self.results_text.config(state=tk.DISABLED)
        
    def plot_results(self, results):
        self.fig.clear()
        
        gs = self.fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3)
        
        x, y = results['x'], results['y']
        
        # Main path
        ax1 = self.fig.add_subplot(gs[0, :])
        ax1.plot(x, y, 'b-', lw=2, alpha=0.8)
        ax1.scatter([x[0]], [y[0]], c='green', s=150, marker='o', zorder=5, label='Start')
        ax1.scatter([x[-1]], [y[-1]], c='red', s=150, marker='s', zorder=5, label='End')
        
        drift_rate = np.rad2deg(results['drift_rate'])
        ax1.set_title(
            f"Quaternion-Centered PDR | Distance: {results['total_distance']:.1f}m | "
            f"Closure: {results['closure']:.2f}m ({results['closure_pct']:.2f}%) | "
            f"Drift: {drift_rate:.3f}Â°/s",
            fontweight='bold', fontsize=11
        )
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        
        # Cumulative heading (Quaternion)
        ax2 = self.fig.add_subplot(gs[1, 0])
        cum_deg = np.rad2deg(results['cumulative_heading'])
        ax2.plot(cum_deg, 'b-', lw=1)
        ax2.set_xlabel('Cycle')
        ax2.set_ylabel('Cumulative Heading (Â°)')
        ax2.set_title(f'Quaternion Heading: {cum_deg[-1]:.1f}Â°', fontweight='bold')
        ax2.grid(True, alpha=0.3)
        
        # Per-cycle heading (Quaternion)
        ax3 = self.fig.add_subplot(gs[1, 1])
        heading_deg = np.rad2deg(results['headings'])
        ax3.plot(heading_deg, 'b-', lw=0.5, alpha=0.7)
        ax3.axhline(np.mean(heading_deg), color='red', linestyle='--', lw=1.5,
                   label=f'Mean: {np.mean(heading_deg):.3f}Â°')
        ax3.axhline(0, color='green', linestyle='-', alpha=0.3)
        ax3.set_xlabel('Cycle')
        ax3.set_ylabel('Heading Change (Â°)')
        ax3.set_title('Per-Cycle Î”yaw (Q_rel)', fontweight='bold')
        ax3.legend(fontsize=8)
        ax3.grid(True, alpha=0.3)
        
        # Heading histogram
        ax4 = self.fig.add_subplot(gs[1, 2])
        ax4.hist(heading_deg, bins=50, edgecolor='black', alpha=0.7, color='steelblue')
        ax4.axvline(0, color='green', linestyle='-', lw=2, alpha=0.7)
        ax4.axvline(np.mean(heading_deg), color='red', linestyle='--', lw=2)
        ax4.set_xlabel('Heading Change (Â°)')
        ax4.set_ylabel('Count')
        ax4.set_title(f'Heading Distribution (Ïƒ={np.std(heading_deg):.2f}Â°)', fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        # Swing yaw (gravity-projected, for reference)
        ax5 = self.fig.add_subplot(gs[2, 0])
        swing_deg = np.rad2deg(results['swing_yaw'])
        ax5.plot(swing_deg, 'orange', lw=0.5, alpha=0.7)
        ax5.axhline(np.mean(swing_deg), color='black', linestyle='--', lw=1.5)
        ax5.axhline(0, color='green', linestyle='-', alpha=0.3)
        ax5.set_xlabel('Cycle')
        ax5.set_ylabel('Swing Yaw (Â°)')
        ax5.set_title('Gravity-Projected Swing (drift ref)', fontweight='bold')
        ax5.grid(True, alpha=0.3)
        
        # Stride histogram
        ax6 = self.fig.add_subplot(gs[2, 1])
        ax6.hist(results['strides'], bins=30, edgecolor='black', alpha=0.7, color='steelblue')
        ax6.axvline(np.mean(results['strides']), color='red', linestyle='--', lw=2)
        ax6.set_xlabel('Stride Length (m)')
        ax6.set_ylabel('Count')
        ax6.set_title(f'Stride: {np.mean(results["strides"]):.3f}m Â± {np.std(results["strides"]):.3f}m', 
                     fontweight='bold')
        
        # Info box
        ax7 = self.fig.add_subplot(gs[2, 2])
        ax7.axis('off')
        
        formula_text = f"""
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
  QUATERNION ZERO-CYCLE
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

THE METHOD:
  Q_ref = Q[HS_N]
  Q_rel = Q[HS_N+1] âŠ— Q_refâ»Â¹
  Î”yaw = yaw(Q_rel)

ADVANTAGES:
  âœ“ No Euler in chain
  âœ“ Gimbal-lock immune
  âœ“ Bounded per cycle

â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
       THE FORMULA
â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

stride = 2 Ã— L_eff Ã— sin(Î¸)

L_eff = {results['L_eff']:.3f} m

â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
"""
        
        ax7.text(0.5, 0.5, formula_text, transform=ax7.transAxes,
                fontsize=8, va='center', ha='center', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9))
        
        self.canvas.draw()
        
    def save_plot(self):
        if self.fig is None:
            return
        filepath = filedialog.asksaveasfilename(
            title="Save Analysis",
            defaultextension=".png",
            filetypes=[("PNG files", "*.png"), ("PDF files", "*.pdf")],
            initialfile="quaternion_centered_pdr.png"
        )
        if filepath:
            self.fig.savefig(filepath, dpi=150, bbox_inches='tight')
            self.status_var.set(f"Saved: {filepath}")
            messagebox.showinfo("Saved", f"Analysis saved to:\n{filepath}")
        
    def process_file(self):
        filepath = self.file_path_var.get()
        
        if not filepath:
            messagebox.showwarning("No File", "Please select a .BIN file first.")
            return
        if not os.path.exists(filepath):
            messagebox.showerror("File Not Found", f"File not found: {filepath}")
            return
        
        self.process_btn.config(state=tk.DISABLED)
        self.save_btn.config(state=tk.DISABLED)
        self.status_var.set("Processing...")
        self.clear_log()
        
        thread = threading.Thread(target=self.run_processing, args=(filepath,))
        thread.start()
        
    def run_processing(self, filepath):
        try:
            data = load_bin_file(filepath, self.log)
            results = fuse_dual_imu(data, self.log)
            self.results = results
            
            self.root.after(0, lambda: self.update_results(results))
            self.root.after(0, lambda: self.plot_results(results))
            self.root.after(0, lambda: self.save_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.status_var.set(
                f"Complete: {results['total_distance']:.1f}m | "
                f"Closure: {results['closure_pct']:.2f}% | "
                f"Drift: {np.rad2deg(results['drift_rate']):.3f}Â°/s"
            ))
            
        except Exception as e:
            self.log(f"\nERROR: {str(e)}")
            import traceback
            self.log(traceback.format_exc())
            self.root.after(0, lambda: messagebox.showerror("Processing Error", str(e)))
            self.root.after(0, lambda: self.status_var.set("Error during processing"))
        finally:
            self.root.after(0, lambda: self.process_btn.config(state=tk.NORMAL))


def main():
    root = tk.Tk()
    app = TarsPDRApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
