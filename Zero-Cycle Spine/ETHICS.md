# Ethics, consent, and data protection

## Ethics approval

The study was conducted in accordance with the Declaration of Helsinki and
approved by:

- **Approving body:** Bioethics Committee of the Lower Silesian Medical Chamber
  in Wrocław, Poland
  (*Komisja Bioetyczna przy Dolnośląskiej Izbie Lekarskiej we Wrocławiu*)
- **Resolution number:** 32/BNBO/2025
- **Date of approval:** 8 October 2025

## Informed consent

Informed consent was obtained from all subjects involved in the study. Written
informed consent was obtained from the patient and from the individual shown in
Figure 1 of the manuscript for publication of the article.

> **Open item — scope of consent.** Consent to participate in research and
> consent to publication of the article are documented. What is not documented
> here is whether consent extends to **open publication of individual-level raw
> recordings**. Those are distinct permissions, and only the third supports a
> world-readable data release. The published Data Availability Statement
> asserts open availability, so this needs to be confirmed against the consent
> form and the approved protocol before the data are made public. If it is not
> covered, an amendment or a restricted-access route is required — see
> [`docs/data_availability_statement.md`](docs/data_availability_statement.md).

## Data protection (GDPR)

Recordings are health data concerning identifiable individuals and fall within
Article 9 special-category processing.

> **Open item.** The following are not established by the manuscript and should
> be recorded here from institutional documentation: the identity of the
> controller (Vratislavia Medica, SpineRebel Technology, or both as joint
> controllers), the lawful basis relied on, the Article 9 condition relied on
> for open publication specifically, and whether a DPIA was completed or
> assessed as not required. Open release of special-category data needs a
> documented basis; research use under approval does not automatically supply
> one for publication.

## De-identification applied

- Direct identifiers removed: names, dates of birth, record numbers.
- Patients referred to by single-letter code (Z, K, P, F) consistently across
  the manuscript, figures, and data archives.
- Recording timestamps within `.BIN` files are milliseconds since firmware
  boot, not wall-clock.
- Figure 1 shows a consented individual, not a study patient.

Note that the manuscript publishes exact monitoring windows per patient
(Z 4–18 February 2026; K 9–26 March 2026; P 10–25 March 2026; F 14–29 April
2026), together with age for the index patient (80), procedure, and level. In
combination with a named institution this is a small candidate set. Archive
filenames carrying the same calendar dates add no protection beyond what the
paper already discloses, but shifting archive filenames to relative day indices
(`day01`, `day02`, …) is a cheap reduction if the raw signal is released.

## Residual re-identification risk

Per-cycle gait kinematics are quasi-biometric. Gait is individually
distinctive, and two weeks of continuous free-living recording from a patient
with a described pathology at a named institution is a high-uniqueness
combination. De-identification reduces but does not eliminate this, and the
point is better stated plainly than asserted away.

Mitigations, in increasing order of protection:

1. Publish derived per-cycle measurements (`X`, the substrate object) instead of
   raw quaternion streams. Preserves reproducibility of every published figure,
   since every figure and statistic in the paper is computed from `X`.
2. Shift archive filenames to relative day indices.
3. Restrict access to approved requesters.

Option 1 deserves emphasis: because the paper's own contribution is `X` rather
than the sensor, publishing `X` in full is not a reduced release — it is the
release of the thing the paper is about.

## Retention

Recordings are retained on institutional servers for a minimum of five years
from the date of publication, consistent with MDPI's data preservation guidance.
