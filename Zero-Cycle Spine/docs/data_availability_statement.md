# Data Availability Statement — current text vs. repository state

## What the submitted manuscript says

> The original data presented in the study are openly available in the
> Zero-Cycle Spine directory of https://github.com/gmiekisiak/spinerebel,
> together with the analysis pipeline and the frozen extractor version used for
> every recording in this study.

## What is currently true

| Claim | Status |
|---|---|
| "openly available" | Per-cycle data are now committed under `data/`; raw `.BIN` remain on OneDrive, linked from the repo |
| "in the Zero-Cycle Spine directory" | Resolved — this is that directory |
| "the analysis pipeline" | Partial: stream viewer only |
| "the frozen extractor version used for every recording" | Not committed |

Two of the four are now resolved: the `Zero-Cycle Spine` directory exists and
the per-cycle substrate is in it under [`../data/`](../data/). Two remain — the
analysis pipeline is only partially present, and the frozen extractor is not
committed. There is also a generation caveat on the committed data: it is
TARS-31 output, not the TARS-60 v3 of record (see
[`../data/README.md`](../data/README.md)). Each is checkable by a reviewer in
under a minute, and the statement is the first thing a data-availability check
looks at. The remaining gap needs to close in one of two directions before the
revision goes back.

---

## Route 1 — make the repository match the statement

Do this if consent and GDPR support open release of raw recordings (see
[`../ETHICS.md`](../ETHICS.md), which flags both as open items).

1. ~~Create `Zero-Cycle Spine/` in the repository.~~ **Done.**
2. Commit the frozen extractor as `src/tars60_v3_extract.py`, with its version
   string matching what it writes into output files. **Outstanding.**
3. ~~Commit the derived per-cycle data — the substrate `X` for all four
   patients.~~ **Done — under [`../data/`](../data/).** Caveat: the committed
   CSVs are TARS-31 generation, not TARS-60 v3, so once the extractor of record
   is committed the substrate should be regenerated (or the generations
   reconciled) so the committed `X` matches the manuscript's analysed counts.
4. Keep raw `.BIN` archives on OneDrive, linked from `DATA.md`, and describe
   them as such rather than as "openly available in the repository."

Amended wording:

> The per-cycle data supporting all reported results, the analysis pipeline,
> and the frozen extractor version used for every recording are openly
> available in the Zero-Cycle Spine directory of
> https://github.com/gmiekisiak/spinerebel. The raw inertial recordings from
> which these were derived are additionally available via the link given in
> that repository.

This keeps the "openly available" claim, keeps it true, and puts the thing the
paper is actually about — `X` — under version control where a DOI-bearing
release can later be minted from it.

---

## Route 2 — amend the statement to match reality (recommended)

Do this if consent does not clearly extend to open publication of raw
individual-level recordings, which is the more likely reading until confirmed.

> The per-cycle data supporting all reported results, together with the
> analysis pipeline and the frozen extractor version used for every recording,
> are openly available at https://github.com/gmiekisiak/spinerebel. The
> underlying raw inertial recordings are not publicly released, because
> continuous gait kinematics are quasi-biometric and carry residual
> re-identification risk; they are available from the corresponding author on
> reasonable request, subject to a data-use agreement and the terms of the
> governing ethics approval (Resolution 32/BNBO/2025).

Why this is the stronger position:

- Every figure, table, and statistic in the paper is computed from `X`, so full
  reproducibility survives intact. Nothing is withheld that a reader needs.
- The paper's own stated contribution is `X`, not the raw stream. Publishing `X`
  in full is releasing the result, not a redacted substitute for it.
- MDPI policy explicitly provides for withholding where privacy issues are
  present, so the reasoning is inside policy rather than an exception to it.
- It does not commit you to open release before the consent question is
  settled — and it is much easier to widen access later than to retract data.

---

## Either way

- **Repository must be public** before the review round. A private repo behind
  the URL in a Data Availability Statement is an immediate editorial query.
- **Do not claim the frozen extractor is available until it is committed.** Of
  everything on this page, that is the one mismatch that damages credibility
  rather than merely inviting a correction, because version-pinned
  reproducibility is itself advertised as a property of the method (§2.6).
- **Naming**: "Zero-Cycle Spine" with a space works but is awkward in URLs and
  shell paths. `zero-cycle-spine/` is friendlier; if you change it, change the
  manuscript to match.
- MDPI's layout guidance asks third-party-hosted supplementary material to
  carry a DataCite DOI and a preservation policy. A GitHub repository is
  reasonably defensible as the stable citable object; if an editor presses,
  minting a Zenodo DOI from a tagged release answers it in one step, since the
  GitHub–Zenodo integration does exactly this.
