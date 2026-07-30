# `.BIN` recording format

Raw recordings are little-endian binary files written by the NimBrace
firmware. Each file consists of a fixed-size header followed by an integral
number of fixed-size sample packets.

```
┌──────────────────────────┐
│ header      86 bytes     │
├──────────────────────────┤
│ packet 0    92 bytes     │
│ packet 1    92 bytes     │
│ ...                      │
│ packet N-1  92 bytes     │
└──────────────────────────┘
```

Number of packets:

```
N = (filesize - 86) // 92
```

A non-zero remainder indicates a truncated final packet — expected when a
recording is interrupted by power loss. The reference reader discards the
partial packet and warns.

---

## Header (86 bytes)

Treated as opaque. No published analysis path reads it; all timing is derived
from packet timestamps. It is preserved verbatim by `read_bin.py` as
`rec["header"]` so that anyone wishing to reverse-engineer firmware metadata
has the bytes, but no field layout is asserted here.

---

## Sample packet (92 bytes)

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 4 | `uint32` LE | timestamp, milliseconds since boot |
| 4 | 88 | `float32[22]` LE | sensor payload, see below |

Struct format strings:

```python
ts,      = struct.unpack('<I',   chunk[0:4])
floats   = struct.unpack('<22f', chunk[4:92])
```

### Float payload

| Index | Count | Field |
|---:|---:|---|
| 0–3 | 4 | IMU1 quaternion |
| 4–6 | 3 | IMU1 linear acceleration (m/s²) |
| 7–9 | 3 | IMU1 angular rate (rad/s) |
| 10–13 | 4 | IMU2 quaternion |
| 14–16 | 3 | IMU2 linear acceleration (m/s²) |
| 17–19 | 3 | IMU2 angular rate (rad/s) |
| 20–21 | 2 | **reserved** |

Indices 20 and 21 are written in every packet and are consumed by no published
analysis path. They are declared reserved and carried through unmodified as
`rec["reserved"]`. No interpretation is asserted.

### Quaternion convention

Hamilton convention, ordered **`[w, x, y, z]`**, unit norm. Consistent with the
analysis code, where the conjugate of `[w,x,y,z]` is `[w,-x,-y,-z]` and yaw is
extracted as:

```
yaw = atan2( 2(wz + xy), 1 - 2(y² + z²) )
```

Source is the BNO085 six-axis gyroscope-plus-accelerometer fusion mode with
the **magnetometer disabled**, so pitch and roll are gravity-referenced while
heading is unconstrained and corrected by the per-cycle reset. Absolute heading
is not observable from a single sensor — this is the condition Zero-Cycle is
designed to operate under, not a limitation of the recording.

### Sensor placement

Two BNO085 sensors in a soft over-shoulder garment, integrated into fabric
pockets so that placement is reproduced approximately at each donning:

- **upper thoracic spine**, approximately **T1**
- **sacrum / lumbosacral region**, approximately **S1**

Separation approximately **50 cm**. Spinal motion is computed as thoracic
relative to pelvic:

```
q_spine(t) = q_T(t) ⊗ q_S(t)⁻¹
```

> **Open item — sensor index assignment.** Which physical sensor is written as
> IMU1 and which as IMU2 is a firmware fact and is not stated in the
> manuscript. It matters: swapping them inverts `q_spine` and therefore the
> sign of every reported inter-segmental angle. Confirm against the firmware
> and record the answer here before publication. Nothing in this repository
> guesses it, and `read_bin.py` deliberately labels the streams `imu1`/`imu2`
> rather than `thoracic`/`pelvic` for that reason.

### Timing

Sampled at **100 Hz**. The high rate is deliberate: it resolves the brief
return to the neutral configuration and the fine structure of each cycle that a
lower rate does not.

Timestamps are milliseconds since firmware boot, **not** wall-clock. They reset
on every power cycle, so timestamps are comparable within a file but not across
files. Recover the effective rate per file:

```python
dt_s = np.median(np.diff(timestamps)) / 1000.0
fs   = 1.0 / dt_s
```

Actual rate varies slightly with logging load; the analysis derives it per file
rather than assuming 100 Hz. Do not resample without saying so.

---

## Missing samples

Packets carry no sequence number beyond the timestamp. A gap in the timestamp
series indicates dropped samples. The reference reader reports gaps exceeding
three nominal sample intervals.

---

## Units in derived data

Worth stating explicitly, because the manuscript figures use two different
units for related quantities:

- Equations 7–8 define valley `v` and peak `p` as **geodesic angles**,
  `d(q') = ‖Log(q')‖ = 2·arccos(|w'|)`.
- **Figure 2** (per-cycle stream) plots degrees of differential trunk–pelvis
  orientation.
- **Figure 3** and Table 1 report the Best-200 metric in **centimetres**.

The derived CSV columns `p0_r` and `p3_r` are in centimetres, and the plotting
code's thresholds (`COH_THRESH = 3.0`, `Y_TOP_CM = 50.0`) are centimetre
values. Over the observed range the numeric magnitudes in degrees and
centimetres happen to be similar, which makes an accidental unit mix easy to
miss and hard to spot in a figure. The angle-to-length conversion used to
produce the centimetre values should be stated in the manuscript methods.

---

## Reading a file

Reference reader: [`src/read_bin.py`](src/read_bin.py).

```bash
python src/read_bin.py DATA_20260206_181927_18.BIN --summary
```

Reports packet count, duration, derived sample rate, dropped-sample gaps, and
per-IMU quaternion validity.
