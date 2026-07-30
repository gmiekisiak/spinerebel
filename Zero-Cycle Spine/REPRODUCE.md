# Reproducing the manuscript figures

## 0. Status

| Stage | Reproducible from this repository? |
|---|---|
| `.BIN` → per-cycle substrate `X` | **No — extractor not committed** |
| `X` → Figure 2 (per-cycle stream) | Yes, `src/tars31_ribbon_gui.py` |
| `X` → Figure 3 (Best-200 trajectories) | Not committed |
| `X` → Table 1 (Cliff's delta) | Not committed |
| `X` → Spearman activity analysis | Not committed |

### What the published statement claims

The manuscript's Data Availability Statement reads:

> *The original data presented in the study are openly available in the
> Zero-Cycle Spine directory of https://github.com/gmiekisiak/spinerebel,
> together with the analysis pipeline and the frozen extractor version used for
> every recording in this study.*

Three commitments there are not currently met: there is no `Zero-Cycle Spine`
directory, the frozen extractor is not committed, and the data are not in the
repository. Either the repository is brought into line with the statement or
the statement is amended at revision. See
[`docs/data_availability_statement.md`](docs/data_availability_statement.md).

### The blocking gap

The frozen extractor of record, **TARS-60 v3**, is the program that reads
`.BIN` and emits the per-cycle columns. Without it a reader can verify the
*figures* against the derived data but not the *extraction* from raw signal.

For a paper whose stated principal contribution is the per-cycle substrate `X`
rather than the sensor, extraction is the step reviewers will most want to
inspect. Committing it needs: the source, a version tag matching the string
written into the output, and its dependency list.

Note that `src/tars31_ribbon_gui.py` reads `*_TARS31.csv`, i.e. the TARS-31
generation, whereas the extractor of record is TARS-60 v3. Either repoint
`CSV_SUFFIX` or reconcile the naming. Mixing generations in one comparative
analysis violates the study's own frozen-extractor rule.

## 1. Environment

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Tested on Python 3.10–3.12. The stream viewer needs a Tk-capable display; on
Debian/Ubuntu install `python3-tk`.

## 2. Get and verify the data

See [`DATA.md`](DATA.md). After downloading:

```bash
bash scripts/verify_sums.sh /path/to/ZeroCyclePaper SHA256SUMS.txt
```

Unpack the per-day archives:

```bash
cd /path/to/ZeroCyclePaper/PatientZ
for z in *.zip; do unzip -q -o "$z" -d "${z%.zip}"; done
```

## 3. Inspect raw recordings

```bash
python src/read_bin.py /path/to/PatientZ/2026-02-04/*.BIN --summary
```

Confirms packet counts, derived sample rate (nominally 100 Hz),
dropped-sample gaps, and per-IMU quaternion validity. A file reporting
`imu2 INVALID` had a sensor dropout.

## 4. Extract the per-cycle substrate

```bash
# NOT YET AVAILABLE — see gap above
python src/tars60_v3_extract.py /path/to/PatientZ --out /path/to/PatientZ
```

The extractor implements, per the manuscript:

1. **Walking-bout detection** — heel-strike detector with a 2 Hz Butterworth
   pre-filter, peak prominence, and a walking-bout coefficient-of-variation
   gate. Validated in the companion letter.
2. **Local neutral estimation** — the geodesic median of heel-strike samples in
   a short rolling window,
   `q_ref(t_k) = median_geo{ q_spine(t_j) : |t_j − t_k| ≤ W, t_j ∈ HS }`.
3. **Anchor test** — a per-cycle sample qualifies when its rotation-vector
   representation lies within a fixed tolerance of that local neutral.
4. **Canonicalisation** — at each anchor the differential quaternion is snapped
   to identity: `q'(t) = q_spine(t) ⊗ q_ref(a)⁻¹`, with geodesic angle
   `d(q') = ‖Log(q')‖ = 2·arccos(|w'|)`.
5. **Per-cycle emission** — valley `v = d(q'(t_HS))`, peak
   `p = max_{t∈C} d(q'(t))`, range `p − v`, accessibility `a`, dwell `Δ`,
   wobble `w`.

Detector parameters (tolerance, rolling-window length) were fixed rather than
optimised, and the return criterion is defined by the recorded distribution of
differential orientation at heel strike rather than by any modelled or
anatomical neutral posture. No biomechanical construct enters the detector.

Standing cycles are bridged by interpolating the local neutral between
consecutive walking bouts when the wearer remains continuously upright. Sitting
and lying cycles receive no kinematic measurement and are logged for activity
allocation only.

**All cycles are retained, including those in which the return failed.** Any
reimplementation that filters failed returns is measuring something else.

## 5. Figure 2 — per-cycle stream

```bash
python src/tars31_ribbon_gui.py /path/to/PatientZ
```

Browse to the directory, **Load CSVs**, **Plot**. Blue = valley (`p0_r`),
red = peak (`p3_r`), with a rolling-median trend per ribbon. One panel per
recording day, that day's files concatenated in order.

The day checklist toggles days in and out. **Any day excluded from a published
figure must be accounted for in the manuscript.** The paper declares exactly
four excluded days across the cohort, on the stated `< 200 validated cycles`
rule; two of those also showed median sensor tilt above 50°, indicating the
device was off the body. Ad-hoc exclusion beyond that rule is a deviation and
has to be declared.

"Blowout report" lists which source files produced out-of-range cycles, grouped
by `file_name` — useful for distinguishing a genuine kinematic excursion from a
mounting artefact. Note that the paper reports one mounting artefact in patient
K that was **flagged rather than removed**, on the reasoning that it adds
scatter to the pre-intervention window and can therefore only reduce the
measured effect. Reproductions should preserve that choice rather than
"cleaning" it.

## 6. Best-200 daily metric

On each day, the most compact 200 consecutive cycles are selected, and the
median peak and median valley of that window are reported.

The compactness objective is defined by the frozen extractor. Reproducing
Figure 3 exactly therefore requires the extractor, not just the per-cycle CSVs
— "most compact" needs its objective function and tie-breaking rule stated for
an independent implementation to agree. Stating that rule explicitly in the
methods would remove the dependency.

Rationale, per the manuscript: a fixed window makes a light-activity day and a
heavy-activity day contribute the same number of cycles, so control for
activity is structural rather than statistical, and the metric cannot be
inflated by walking more. Taking the *most compact* window reflects the
best-controlled state the spine reached that day rather than an average diluted
by fatigue or by long, variable bouts.

## 7. Effect sizes and statistics

**Cliff's delta**, computed per patient between a pre-intervention window and
the post-intervention plateau, against that patient's own distribution:

```
δ = ( #{x_pre_i > x_post_j} − #{x_pre_i < x_post_j} ) / (n_pre · n_post)
```

Oriented so that **positive indicates a reduction** of the per-cycle metric
after intervention. Bounded in [−1, 1], no distributional assumption.

Expected values: Z +0.87, K +0.70, P −0.29, F +0.95.

**Activity confounder check.** Across 45 plateau patient-days pooled within
patient, Spearman correlations of daily valley and peak against distance
walked, walking time, and cycle count: all |ρ| ≤ 0.24, all p ≥ 0.12.

**Framing that must be preserved.** The unit of analysis is the movement cycle,
used as a descriptive unit *within* each patient. Adjacent cycles are serially
correlated and nested within bouts, days, and patients; the clinical sample is
four patients. No population-level significance test is appropriate and none is
reported. A reproduction that treats the 88,027 cycles as independent
observations and reports p-values is not reproducing this analysis.

## 8. Internal validation criteria

No external ground truth and no optical motion capture, deliberately —
agreement with a laboratory-frame measurand would compare different quantities.
The per-cycle stream is instead assessed for internal drift consistency:

- approximately zero-mean per-cycle increments;
- near-zero lag-one autocorrelation;
- cumulative heading contained within a `σ√N` envelope;
- recovery of a reference per-cycle return dispersion of approximately 1.85°,
  established on the same hardware in separate recordings (companion letter).

Loop-closure error is not used as a primary metric: when reset timing correlates
with the underlying drift, closure can be small even when the reconstructed path
is wrong.

For scale, the drift these criteria operate against: 12–35°/h of slow garment
or mounting drift within a recording, and 70–100° of placement difference
between recordings.
