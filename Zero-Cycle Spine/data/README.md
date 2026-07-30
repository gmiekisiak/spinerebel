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

## Cycle counts — reproduces the manuscript exactly

Applying the paper's own declared exclusion rule — drop each recording day with
fewer than 200 validated cycles — reproduces the published per-patient counts
exactly, all four patients, to the cycle:

| Patient | All cycles | Excluded (< 200-cycle days) | Retained | Paper |
|---|---:|---:|---:|---:|
| Z | 19,320 | 187 (2 days) | 19,133 | 19,133 |
| K | 38,210 | 0 | 38,210 | 38,210 |
| P | 18,061 | 102 (1 day) | 17,959 | 17,959 |
| F | 12,826 | 101 (1 day) | 12,725 | 12,725 |
| **Total** | **88,417** | **390 (4 days)** | **88,027** | **88,027** |

The four excluded days also match the manuscript's independent description:
four days removed, two of them with median sensor tilt above 50° indicating the
device was off the body — Z 17 Feb (10 cycles, median tilt 103.3°) and Z 18 Feb
(177 cycles, 51.7°); the other two (P 12 Mar, F 15 Apr) have normal tilt (~5°).

These files are therefore the extractor output that produced the published
results.

## A note on the filename prefix

The `tars31_` prefix and the manuscript's frozen extractor of record,
**TARS-60 v3** (see [`../CHANGELOG.md`](../CHANGELOG.md)), do not agree on their
face. The numbers above establish that the *data* is correct; what needs
settling is only the *label*. Either the extractor is genuinely of the TARS-31
lineage and the manuscript's "TARS-60 v3" string is wrong, or it is v3 and
these filenames carry a stale prefix. The manuscript, `CHANGELOG.md`, and these
filenames should be made to agree — that is an authoring decision, not a data
problem, and it does not require regenerating anything.

## Viewing

```bash
python ../src/tars31_ribbon_gui.py data/Z
```

Browse to a patient directory, **Load CSVs**, **Plot**. The viewer discovers
`{date}_cycles.csv` files by date prefix, so the `tars31_cohort_cycles.csv`
aggregate is not plotted as a spurious extra day.
