# Changelog

Extractor versions are part of the scientific record. Derived biomarkers are
detector-version dependent, so every change that could alter a per-cycle number
gets an entry here and a new version string written into the output. Published
results are never silently re-computed.

The manuscript states this as an operational and regulatory requirement as much
as a scientific one (§2.6, "Version-pinned reproducibility"): features must be
defined identically across training, validation, and deployment.

## Extractor versions

| Version | Status | Used for |
|---|---|---|
| TARS-60 v3 | **frozen** | *Sensors* manuscript — all four recordings |
| TARS-56 | superseded | early Patient K processing; not comparable to v3 |
| TARS-31 | superseded | stream viewer column names still reference this generation |

Never mix versions within a comparative analysis. Re-processing a cohort under a
new extractor requires re-processing **all** patients in that cohort.

Detector parameters were fixed rather than optimised: the anchor tolerance and
the rolling-window length `W` are constants of the frozen version, not fitted
values. Changing either is a version change.

## Repository history

### Unreleased
- Initial public release prepared for *Sensors* submission.
- Added `.BIN` format specification (86-byte header, 92-byte packets, dual
  BNO085 at 100 Hz, magnetometer disabled) and reference reader.
- Added integrity manifests and verification scripts.
- Unified the blowout threshold in the stream viewer at 50 cm; the per-day
  statistics and the blowout report previously used 50 and 60 cm respectively,
  so panel titles and the report disagreed for cycles between the two.
- Stream viewer now names each skipped recording day and the reason, rather
  than reporting only a count.
- Figure export raised to 300 dpi for MDPI raster requirements.

### Outstanding before publication
- Frozen extractor TARS-60 v3 not committed.
- Per-cycle substrate `X` not committed.
- `Zero-Cycle Spine` directory named in the Data Availability Statement does
  not exist.
- Best-200 compactness objective not stated as a formula.
- Figure 3 / Table 1 / Spearman analysis scripts not committed.
- IMU1 vs IMU2 to thoracic/pelvic assignment not confirmed (see `FORMAT.md`).
- Angle-to-centimetre conversion for the Best-200 metric not documented.
