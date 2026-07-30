# Data availability

## What is where

| Object | Location |
|---|---|
| Per-cycle substrate `X` (all four patients) | [`data/`](data/) — committed (TARS-31 generation; see caveat) |
| Frozen extractor TARS-60 v3 | *to be committed to this repository* |
| Raw `.BIN` recordings (~1 GB) | linked below |

The per-cycle data are now in the repository under [`data/`](data/), one
directory per patient (Z, K, P, F); see [`data/README.md`](data/README.md) for
the layout and columns. **Caveat:** the committed CSVs are TARS-31
extractor-generation output, whereas the frozen extractor of record is
TARS-60 v3, and the per-patient cycle counts differ from the manuscript's
analysed set for every patient except K. Reconcile the generation before these
are treated as the substrate of record — see [`data/README.md`](data/README.md)
and [`CHANGELOG.md`](CHANGELOG.md).

The manuscript's Data Availability Statement and the current repository state
still do not fully agree — the frozen extractor remains uncommitted. See
[`docs/data_availability_statement.md`](docs/data_availability_statement.md)
before publication.

## Raw recordings

Bundled one archive per recording day, grouped by patient:

**[ZeroCyclePaper](https://1drv.ms/f/c/7007aaea97dde532/IgBEc207qx8lRYg6FnfEbDZWAU-q9oERxNq8oPh0jpGOVkM?e=QhQab2)**

Direct download of the whole folder as a zip:

```
https://1drv.ms/f/c/7007aaea97dde532/IgBEc207qx8lRYg6FnfEbDZWAU-q9oERxNq8oPh0jpGOVkM?e=QhQab2&download=1
```

**If that link no longer resolves, please open an issue in this repository** and
it will be updated. The link is kept here rather than in the published article
so that it can be corrected without a corrigendum.

### Layout

```
ZeroCyclePaper/
├── PatientZ/          13 days, 4-18 Feb 2026,   19,133 cycles
├── PatientK/          16 days, 9-26 Mar 2026,   38,210 cycles
├── PatientP/          16 days, 10-25 Mar 2026,  17,959 cycles
├── PatientF/          16 days, 14-29 Apr 2026,  12,725 cycles
└── SHA256SUMS.txt
```

Per-day bundling is intentional: a reader spot-checking one day's cycle counts
can download a single archive of tens of megabytes rather than the whole
gigabyte.

Four recording days are absent from the analysed set, excluded under the
declared `< 200 validated cycles` rule; two of those also showed median sensor
tilt above 50°, indicating the device was off the body. The raw archives for
those days are retained here rather than deleted, so the exclusion can be
checked.

---

## Verifying your download

`SHA256SUMS.txt` lists the SHA-256 of every archive **as uploaded**. Run the
check from inside the directory containing the downloaded archives, pointing at
the copy of the manifest in this repository:

```bash
cd ~/Downloads/ZeroCyclePaper          # wherever the archives ended up
sha256sum -c /path/to/spinerebel/SHA256SUMS.txt
```

Every line should end in `OK`. A `FAILED` line means an incomplete or corrupted
download — re-fetch that archive. Silent truncation of large downloads is common
and otherwise undetectable.

Platform notes:

- **macOS** has no `sha256sum`; use `shasum -a 256 -c` instead.
- **Windows PowerShell**: `Get-FileHash file.zip -Algorithm SHA256`, one file at
  a time, compared manually. Or use Git Bash, which provides `sha256sum`.
- `scripts/verify_sums.sh` wraps the above with clearer output.

### Archive hashes vs. content hashes

`SHA256SUMS.txt` hashes the **zip archives**, not their contents. Zip files
embed timestamps and entry ordering, so re-creating an archive from identical
`.BIN` files yields a different hash. The archives published here are therefore
treated as immutable and are never regenerated.

`CONTENTS.sha256`, where present, lists the SHA-256 of each `.BIN` inside each
archive as `archive.zip!entry.BIN`. That manifest survives re-bundling but is
not directly consumable by `sha256sum -c`.

---

## Reading the recordings

Format specification: [`FORMAT.md`](FORMAT.md) — 86-byte header, 92-byte
packets, dual BNO085 at 100 Hz, magnetometer disabled.
Reference reader: [`src/read_bin.py`](src/read_bin.py).

```bash
python src/read_bin.py PatientZ/2026-02-04/DATA_20260204_181927_18.BIN --summary
```

---

## Hosting, honestly stated

Raw recordings are hosted on cloud storage, not in a DataCite-registered
repository. That is an interim choice with consequences a reader should know:

- no DOI and no long-term preservation guarantee for the raw archives;
- the link depends on an account remaining active;
- not FAIR-compliant in the archival sense.

Recordings are retained on institutional servers for a minimum of five years
from publication. The derived per-cycle substrate `X` — which is what every
published figure and statistic is computed from — belongs in this repository
under version control, where a citable DOI can be minted from a tagged release.

---

## Licence

**Data: CC BY 4.0**, matching the article's own Creative Commons Attribution
terms. Attribution to the *Sensors* article.

Patent rights are independent of any data licence and are expressly reserved —
see [`PATENTS.md`](PATENTS.md). The data licence permits reuse of the
measurements; it does not license the measurement method.

---

## Re-identification

Per-cycle gait kinematics are quasi-biometric. See [`ETHICS.md`](ETHICS.md) for
the full assessment, the mitigations available, and two open items — the scope
of participant consent and the documented GDPR basis — that should be closed
before raw recordings are made world-readable.
