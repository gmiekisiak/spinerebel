# Nadir-Based Zero-Cycle Heading for Torso-Mounted PDR

Source code, firmware, and data accompanying:

> G. Miekisiak, "Torso-Mounted Pedestrian Dead Reckoning Using Nadir-Based Zero-Cycle Heading and Differential Accelerometry: Extended Analysis," *arXiv preprint*, 2026.

> G. Miekisiak, "Torso-Mounted Pedestrian Dead Reckoning Using Nadir-Based Zero-Cycle Heading and Differential Accelerometry," *IEEE Sensors Letters*, submitted, 2026.

## Summary

Zero-Cycle heading exploits the periodic return of the pelvis to its Panjabi Neutral Zone during gait. By sampling orientation only at the nadir (closest approach to the neutral zone) within each step, per-step heading errors become independent and cumulative drift follows O(√n) rather than O(n). Combined with differential accelerometry stride estimation using the T4–S1 inter-sensor spacing, the method achieves 0.55% mean closure error across 9.3 km (four recordings, 14,483 steps) from spine-mounted dual BNO085 IMUs without magnetometer.

## Results

| Recording | Date | Distance (m) | Steps | Closure (%) |
|-----------|------|-------------|-------|-------------|
| A | Jan 24 | 2138 | 3321 | 0.49 |
| B | Feb 1 | 2428 | 3782 | 0.18 |
| C | Feb 2 | 2339 | 3643 | 0.58 |
| D | Feb 4 | 2407 | 3737 | 0.95 |
| **Mean** | | **2328** | **3621** | **0.55** |

All recordings are closed-loop walks through mixed indoor/outdoor terrain (sidewalks, underground parking garage, vehicle ramps) in Wrocław, Poland. Recording C was cross-validated against GPS (0.4% outdoor distance error). No magnetometer was used. No calibration walk was performed.

## Repository Structure

```
├── paper/                      arXiv preprint (.tex) and figures
│   ├── arxiv_preprint.tex
│   ├── fig_route_map.jpg
│   └── fig_route_photos.jpg
├── pdr/                        PDR algorithms (Python)
│   ├── spine_pdr_v5_nadir.py   Nadir-based spine PDR (paper method)
│   └── tars_pdr_v17.py         Shin-mounted PDR (earlier work)
├── firmware/nimbrace/          NimBrace device firmware (Arduino)
│   ├── nimbrace.ino
│   ├── logging.ino
│   └── cradle.ino
├── data/                       Raw .BIN recordings
├── LICENSE
└── CITATION.cff
```

## Usage

```bash
pip install numpy scipy matplotlib
python pdr/spine_pdr_v5_nadir.py data/RUN_B.BIN
```

A GUI opens with fields for known loop distance (calibrated stride) or subject height (allometric stride, default 1.70 m). Closure percentage is K-independent and holds for any calibration method.

Requires Python 3.8+, NumPy, SciPy, Matplotlib, tkinter.

## Data Format

NimBrace `.BIN` files begin with an 86-byte `NIMDATA` header followed by 92-byte packets at approximately 91 Hz.

| Offset | Type | Content |
|--------|------|---------|
| 0–3 | uint32 | Timestamp (ms since boot) |
| 4–19 | float32 ×4 | Pelvis quaternion (w, x, y, z) |
| 20–31 | float32 ×3 | Pelvis linear acceleration (m/s²) |
| 32–43 | float32 ×3 | Pelvis angular velocity (rad/s) |
| 44–59 | float32 ×4 | Thorax quaternion (w, x, y, z) |
| 60–71 | float32 ×3 | Thorax linear acceleration (m/s²) |
| 72–83 | float32 ×3 | Thorax angular velocity (rad/s) |
| 84–91 | — | Battery voltage, current, event flags |

All values little-endian. Quaternions are BNO085 game rotation vector output (gyroscope + accelerometer fusion, no magnetometer).

## Hardware

The NimBrace device comprises two BNO085 IMUs (CEVA/Hillcrest Labs) at approximately T4 and S1 vertebral levels, connected by 35 cm of pliable elastic material. Processing: Raspberry Pi Pico 2W. Storage: SD card with PCF8523 RTC. Total electronics BOM: $42 USD. See `firmware/nimbrace/` for the complete Arduino source.

## Patent Notice

The Zero-Cycle principle, nadir-based heading extraction, and differential accelerometry stride estimation are the subject of pending PCT patent applications. This repository is provided under an academic and non-commercial research license. See [LICENSE](LICENSE) for terms. Commercial use requires a separate licensing agreement.

## Citation

```bibtex
@article{miekisiak2026zerocycle,
  author  = {Miekisiak, Grzegorz},
  title   = {Torso-Mounted Pedestrian Dead Reckoning Using Nadir-Based
             Zero-Cycle Heading and Differential Accelerometry},
  journal = {IEEE Sensors Letters},
  year    = {2026},
  note    = {Submitted}
}
```

## Author

Grzegorz Miekisiak, MD, PhD
Vratislavia Medica Hospital, Wrocław, Poland
SpineRebel Technology
grzegorz@spinerebel.com
