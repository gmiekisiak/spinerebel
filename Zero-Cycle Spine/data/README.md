# Per-cycle data

The derived per-cycle substrate for the four study patients. This is the object
the manuscript is about — every published figure, table, and statistic is
computed from these files, not from the raw `.BIN` recordings (which are hosted
separately, see [`../DATA.md`](../DATA.md)).

## Layout

```
data/
├── Z/   index patient   (TESI)          13 days, 4–18 Feb 2026
├── K/                    (foraminotomy)  16 days, 9–26 Mar 2026
├── P/   comparison       (TESI)          16 days, 10–25 Mar 2026
└── F/                    (foraminotomy)  16 days, 14–29 Apr 2026
```

Each patient directory contains:

| File | Content |
|---|---|
| `{date}_cycles.csv` | One row per gait cycle for that recording day — the per-cycle substrate `X` |
| `{date}_summary.csv` | Day-level rollup for that day (absent for patient Z) |
| `tars31_cohort_cycles.csv` | All of that patient's cycles concatenated, with `date` and `source` columns |
| `tars31_timeline.csv` | Day-level timeline: cycle counts, activity minutes, Best-200 valley/peak, walking distance, STS and stair metrics |
| `tars31_cohort_ai_notes.csv`, `tars31_cohort_ai_refined.csv` | Reviewer/AI day-level annotations (absent for patient Z) |

## Cycle columns (`*_cycles.csv`)

| Column | Meaning |
|---|---|
| `cycle_idx` | Cycle index within the source recording |
| `source_file` | Originating `.BIN` recording |
| `p0_r` | **Valley** — geodesic return distance from local neutral (cm) |
| `p3_r` | **Peak** — maximum excursion from local neutral over the stride (cm) |
| `p7_r` | Additional per-cycle return sample |
| `amplitude` | Peak amplitude (cm) |
| `duration_s` | Cycle duration (s) |
| `validated` | 1 if the cycle passed validation, 0 otherwise |
| `tilt_deg` | Median sensor tilt over the cycle (device-off-body indicator) |
| `q_error_deg` | Quaternion consistency error (deg) |

See [`../README.md`](../README.md) for the mapping of these columns to the
paper's `v`, `p`, `p − v`, `a`, `Δ`, `w` feature symbols, and
[`../REPRODUCE.md`](../REPRODUCE.md) for how the figures are built from them.

## Cycle counts as committed

| Patient | Daily files | Days | Cycles in `tars31_cohort_cycles.csv` |
|---|---:|---:|---:|
| Z | 13 | 4–18 Feb 2026 | 19,320 |
| K | 16 | 9–26 Mar 2026 | 38,210 |
| P | 16 | 10–25 Mar 2026 | 18,061 |
| F | 16 | 14–29 Apr 2026 | 12,826 |
| **Total** | | | **88,437** |

## Extractor generation — read before citing these numbers

**These files are TARS-31 extractor-generation output**, as the `tars31_`
filenames indicate. The frozen extractor of record for the *Sensors* manuscript
is **TARS-60 v3** (see [`../CHANGELOG.md`](../CHANGELOG.md)). Derived
per-cycle numbers are detector-version dependent, and the counts above differ
from the manuscript's analysed set (19,133 / 38,210 / 17,959 / 12,725 =
88,027) for every patient except K. Before these are used as the substrate of
record for the paper, reconcile the generation: either regenerate under
TARS-60 v3 or confirm that the `p0_r` / `p3_r` semantics are identical across
generations. Do not mix generations within one comparative analysis.

## Viewing

```bash
python ../src/tars31_ribbon_gui.py data/Z
```

Browse to a patient directory, **Load CSVs**, **Plot**. The viewer discovers
`{date}_cycles.csv` files by date prefix, so the `tars31_cohort_cycles.csv`
aggregate is not plotted as a spurious extra day.
