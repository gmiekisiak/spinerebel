# Zero-Cycle Spine

Data and analysis code accompanying:

> **Zero-Cycle: First Continuous, Unsupervised, At-Home Monitoring of Spinal
> Motion as a Per-Cycle Substrate for Machine Learning**
> G. Miekisiak, T. Szczepański, A. Palczewska.
> *Sensors* (MDPI), submitted 2026.

Companion engineering letter:

> G. Miekisiak. Torso-mounted pedestrian dead reckoning using nadir-based
> Zero-Cycle heading and differential accelerometry.
> *IEEE Sensors Letters* **2026**, 10(8), 6007204.
> doi:10.1109/LSENS.2026.3702726

This repository is the reference cited in the manuscript's Data Availability
Statement. It contains the recording format specification, the analysis code,
integrity manifests, and the recordings themselves.

---

## Contents

| Path | Purpose |
|---|---|
| [`data/`](data/) | The per-cycle substrate `X` — one directory per patient (Z, K, P, F) |
| [`DATA.md`](DATA.md) | Where the recordings are, and how to verify them |
| [`FORMAT.md`](FORMAT.md) | Byte-level specification of the `.BIN` recording format |
| [`REPRODUCE.md`](REPRODUCE.md) | Step-by-step reproduction of the manuscript figures |
| [`ETHICS.md`](ETHICS.md) | Ethics approval, consent, and data-protection basis |
| [`src/read_bin.py`](src/read_bin.py) | Minimal reference reader for `.BIN` files |
| [`src/tars31_ribbon_gui.py`](src/tars31_ribbon_gui.py) | Per-cycle stream viewer (Figure 2 style) |
| `SHA256SUMS.txt` | Checksums of the published archives |
| [`scripts/`](scripts/) | Manifest generation and verification helpers |
| [`PATENTS.md`](PATENTS.md) | Patent notice — read before any non-academic use |

---

## Study at a glance

| | |
|---|---|
| Patients | 4 (Z, K, P, F) |
| Setting | Free-living, unsupervised, patients' own homes |
| Sensors | Two BNO085, upper thoracic (~T1) and sacrum (~S1), ~50 cm apart |
| Fusion mode | Six-axis gyroscope + accelerometer, **magnetometer disabled** |
| Sampling | 100 Hz |
| Feature vector | `d = 6` per cycle |
| Analysed cycles | **88,027** total |
| Frozen extractor | TARS-60 v3 |
| Ethics | Resolution 32/BNBO/2025, 8 October 2025 |

### Per patient

| Patient | Intervention | Days | Dates | Cycles | Cliff's δ |
|---|---|---:|---|---:|---:|
| Z (index) | TESI | 13 | 4–18 Feb 2026 | 19,133 | +0.87 |
| K | Foraminotomy | 16 | 9–26 Mar 2026 | 38,210 | +0.70 |
| P (comparison) | TESI | 16 | 10–25 Mar 2026 | 17,959 | −0.29 |
| F | Foraminotomy | 16 | 14–29 Apr 2026 | 12,725 | +0.95 |

Patient P presented with sciatica for which MRI showed no corresponding
lesion, and serves as the within-study non-structural comparison case. A
positive Cliff's delta indicates reduction of the per-cycle metric after
intervention.

---

## The Zero-Cycle principle in one paragraph

Orientation from a magnetometer-free IMU drifts without bound: gyroscope bias
accumulates linearly, `ε ≈ b·t = O(N)`. The trunk never reaches a true
zero-velocity state during gait, so the zero-velocity update that bounds
foot-mounted navigation is inapplicable. Zero-Cycle uses instead a *kinematic
configuration the segment returns to*: the differential trunk–pelvis
orientation reoccupies a consistent neutral roughly once per gait cycle even
though no segment is ever stationary. At each return the differential
quaternion is snapped to identity, so per-cycle increments are zero-mean and
approximately independent and cumulative heading error becomes a random walk,
`σ_c·√N = O(√N)`. The same reset supplies the correction a magnetometer would
otherwise provide, and the per-cycle local frame removes subject-specific
calibration. The method operates entirely in the orientation domain and never
double-integrates acceleration.

The same event that bounds the drift also brackets the cycle. Spinal motion is
the thoracic sensor relative to the pelvic sensor,
`q_spine = q_T ⊗ q_S⁻¹`, so whole-body turning cancels in the differential and
three inter-segmental components remain: flexion/extension, lateral bending,
and axial rotation — the last inaccessible to a single magnetometer-free
sensor.

---

## Per-cycle feature vector

Per cycle, in the local frame:

| Symbol | Column | Meaning |
|---|---|---|
| `v` | `p0_r` | **Valley** — geodesic distance of the return sample from local neutral; how precisely the spine returns |
| `p` | `p3_r` | **Peak** — maximum geodesic distance from local neutral over the stride (cone-of-economy excursion) |
| `p − v` | — | Inter-segmental dynamic range |
| `a` | — | **Return accessibility** — retained outcome of the gating event: whether the cycle reached local neutral within tolerance |
| `Δ` | — | **Dwell** — time spent near neutral |
| `w` | — | **Within-zone wobble** — residual motion while near neutral |

Stacked over a recording these form the substrate object
`X = [x₁ … x_N]ᵀ ∈ ℝ^(N×d)`. Each row is one cycle, carrying wall-clock
timestamp, cycle duration, and bout identifier. **X, not the sensor, is the
deliverable of this work.** It is committed under [`data/`](data/), one
directory per patient — see [`data/README.md`](data/README.md).

Cycles in which the return *failed* are retained as informative samples rather
than discarded. This is central to the method, not a tolerance: any
reimplementation that filters failed returns is measuring something else.

---

## Day-level summary: Best-200

For day-to-day comparison the per-cycle stream is summarised with a fixed-size
window: on each day the most compact 200 consecutive cycles are taken, and the
median peak and median valley of that window are reported.

The fixed window size makes control for activity **structural rather than
statistical** — a light day and a heavy day contribute the same number of
cycles, so the metric cannot be inflated by walking more. Across 45 plateau
patient-days pooled within patient, daily valley and peak showed no detectable
association with distance walked, walking time, or cycle count
(Spearman |ρ| ≤ 0.24, all p ≥ 0.12).

Days yielding fewer than 200 validated cycles are excluded, because the metric
is undefined below that count. Four such days were removed across the cohort;
two also showed median sensor tilt above 50°, indicating the device was off
the body.

---

## Frozen-extractor discipline

Derived biomarkers are detector-version dependent. **A single frozen extractor
version was used for every file in the study, and versions must never be mixed
within a comparative analysis.** Re-processing produces a new version and a new
manifest; it does not silently replace the published one. See
[`CHANGELOG.md`](CHANGELOG.md).

The extractor version of record is **TARS-60 v3**.

> **The per-cycle data are committed; the extractor is not.** The substrate `X`
> is in [`data/`](data/), but the CSVs there are TARS-31 extractor-generation
> output, not TARS-60 v3, and the per-patient cycle counts differ from the
> manuscript's analysed set for every patient except K. The extractor itself —
> the program that reads `.BIN` and emits the per-cycle columns — is still not
> in this repository, though the published Data Availability Statement asserts
> that it is. See [`data/README.md`](data/README.md) for the generation caveat,
> [`REPRODUCE.md`](REPRODUCE.md) for what this blocks, and
> [`docs/data_availability_statement.md`](docs/data_availability_statement.md)
> for the wording consequences.

---

## Quick start

```bash
git clone https://github.com/gmiekisiak/spinerebel.git
cd "spinerebel/Zero-Cycle Spine"
pip install -r requirements.txt

# inspect a raw recording
python src/read_bin.py path/to/DATA_20260206_181927_18.BIN --summary

# view the per-cycle stream from the committed CSVs
python src/tars31_ribbon_gui.py data/Z
```

---

## Citing

See [`CITATION.cff`](CITATION.cff). Please cite the *Sensors* article rather
than this repository alone.

## Licence

Code: [`LICENSE`](LICENSE) (MIT). Data: [`DATA.md`](DATA.md) (CC BY 4.0,
matching the article's own CC BY terms). Patent rights are expressly reserved
and are independent of both — see [`PATENTS.md`](PATENTS.md).

## Contact

Grzegorz Miekisiak, MD PhD — Vratislavia Medica, Wrocław, Poland.
Correspondence: gmiekisiak@gmail.com

Please open a GitHub issue rather than emailing where possible, so that
answers are visible to other readers.
