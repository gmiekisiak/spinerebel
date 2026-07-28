# PDR Algorithms

## spine_pdr_v5_nadir.py — Main (Paper Method)

Nadir-based Zero-Cycle PDR using dual spine-mounted IMU (T4 + S1).
This is the algorithm described in the paper.

**Usage:**
```bash
python spine_pdr_v5_nadir.py DATA_FILE.BIN
```

Opens a GUI with controls for known distance (calibrated mode) or subject
height (allometric mode). If neither is provided, defaults to 1.70 m.

**Dependencies:** numpy, scipy, matplotlib, tkinter

**Key functions:**
- `detect_heel_strikes()` — thorax accelerometer impact detection
- `find_hub()` — Panjabi Neutral Zone center from 2D histogram
- `detect_nadirs()` — closest approach to hub within each step
- `zero_cycle_pdr()` — heading + stride → path reconstruction
- `classify_terrain_nz()` — slope detection from NZ pitch shift

## tars_pdr_v17.py — Shin-Mounted PDR (Earlier Work)

Quaternion-centered Zero-Cycle PDR for shin-mounted dual IMU.
Uses the TARS stride model (stride = 2 × L_eff × sin(θ)) with dual
centripetal extraction for automatic height estimation.

This was the original validation of Zero-Cycle heading. The spine-mounted
version (v5) supersedes it for the NimBrace use case but the shin version
remains useful for leg-mounted applications.

**Usage:**
```bash
python tars_pdr_v17.py DATA_FILE.BIN
```

## License

Academic and non-commercial use only. See repository LICENSE.
PCT patent pending.
