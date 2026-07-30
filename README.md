# SpineRebel

Code, firmware, and data for the Zero-Cycle line of research: magnetometer-free
spinal motion sensing from dual spine-mounted IMUs.

This repository hosts one sub-project per paper.

## Projects

| Folder | Paper | Status |
|---|---|---|
| [`Zero-Cycle Spine/`](Zero-Cycle%20Spine/) | G. Miekisiak, T. Szczepański, A. Palczewska, "Zero-Cycle: First Continuous, Unsupervised, At-Home Monitoring of Spinal Motion as a Per-Cycle Substrate for Machine Learning," *Sensors* (MDPI) | Submitted, 2026 |
| [`Torso-Mounted PDR/`](Torso-Mounted%20PDR/) | G. Miekisiak, "Torso-Mounted Pedestrian Dead Reckoning Using Nadir-Based Zero-Cycle Heading and Differential Accelerometry," *IEEE Sensors Letters* **2026**, 10(8), 6007204. [doi:10.1109/LSENS.2026.3702726](https://doi.org/10.1109/LSENS.2026.3702726) | Published |

Each folder has its own README with the paper summary, repository layout,
data format, and reproduction instructions.

- **`Zero-Cycle Spine/`** — the *Sensors* study: four patients, 88,027 gait
  cycles of free-living at-home monitoring, per-cycle feature substrate,
  frozen-extractor discipline. Start at
  [`Zero-Cycle Spine/README.md`](Zero-Cycle%20Spine/README.md); the data
  availability material referenced by the manuscript is in
  [`Zero-Cycle Spine/DATA.md`](Zero-Cycle%20Spine/DATA.md).
- **`Torso-Mounted PDR/`** — the companion engineering letter: nadir-based
  Zero-Cycle heading for pedestrian dead reckoning, NimBrace firmware, and
  the four closed-loop walk recordings.

## Patent notice

The Zero-Cycle principle, nadir-based heading extraction, and differential
accelerometry stride estimation are the subject of pending PCT patent
applications. See [PATENTS.md](Zero-Cycle%20Spine/PATENTS.md) and the licence
terms below before any non-academic use.

## Licence

The repository-wide default is the academic and non-commercial licence in
[LICENSE](LICENSE). The `Zero-Cycle Spine/` sub-project carries its own
[LICENSE](Zero-Cycle%20Spine/LICENSE) (MIT, copyright only — no patent
grant), which governs that folder. Commercial and patent licensing:
grzegorz@spinerebel.com.

## Citation

Cite the paper matching the sub-project you use — see
[`Zero-Cycle Spine/CITATION.cff`](Zero-Cycle%20Spine/CITATION.cff) and the
Citation section of [`Torso-Mounted PDR/README.md`](Torso-Mounted%20PDR/README.md).

## Author

Grzegorz Miekisiak, MD, PhD
Vratislavia Medica Hospital, Wrocław, Poland
SpineRebel Technology
