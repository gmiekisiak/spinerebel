# Data Files

Raw `.BIN` recordings from the NimBrace device used in the paper.

## Recordings

| File | Recording | Date | Approx. Distance | Loops |
|------|-----------|------|-------------------|-------|
| `run_A.BIN` | A | Jan 24, 2026 | 2138 m | ~3.5 |
| `run_B.BIN` | B | Feb 1, 2026 | 2428 m | ~4 |
| `run_C.BIN` | C | Feb 2, 2026 | 2339 m | ~3.5 (GPS cross-validated) |
| `run_D.BIN` | D | Feb 4, 2026 | 2407 m | ~4 |

All recordings are closed-loop walks in mixed indoor/outdoor environment
(underground garage + sidewalks, Wroclaw, Poland). Single healthy male subject,
170 cm, 70 kg. Self-selected brisk pace. No calibration walk.

## File Format

86-byte header (`NIMDATA` magic, version 2), followed by 92-byte packets:

```
Header (86 bytes):
  [0:7]   magic     "NIMDATA\0"
  [8:9]   version   uint16 = 2
  [10:11] header_sz uint16 = 86
  [12:13] packet_sz uint16 = 92
  [14:33] basename  char[20]
  [34:65] brace_id  char[32]
  [66:69] unix_ts   uint32
  [70:85] reserved

Packet (92 bytes):
  [0:3]   timestamp     uint32  (ms from boot)
  [4:19]  IMU1 quat     float32 × 4  (w, x, y, z)
  [20:31] IMU1 linacc   float32 × 3  (x, y, z, m/s²)
  [32:43] IMU1 gyro     float32 × 3  (x, y, z, rad/s)
  [44:59] IMU2 quat     float32 × 4  (w, x, y, z)
  [60:71] IMU2 linacc   float32 × 3  (x, y, z, m/s²)
  [72:83] IMU2 gyro     float32 × 3  (x, y, z, rad/s)
  [84:85] voltage       uint16  (mV)
  [86:87] current       int16   (mA)
  [88]    event_flags   uint8
  [89:91] padding
```

IMU1 = pelvis (S1), IMU2 = thorax (~T4).
Game rotation vector mode (gyro + accel fusion, no magnetometer). ~91 Hz.

## Loading Example

```python
import struct
import numpy as np

def load_bin(filepath):
    with open(filepath, 'rb') as f:
        data = f.read()
    
    packets = data[86:]  # skip header
    n = len(packets) // 92
    
    timestamps, pel_quats, tho_quats = [], [], []
    pel_acc, pel_gyr, tho_acc, tho_gyr = [], [], [], []
    
    for i in range(n):
        p = packets[i*92:(i+1)*92]
        ts = struct.unpack('<I', p[0:4])[0]
        f = struct.unpack('<22f', p[4:92])
        
        timestamps.append(ts)
        pel_quats.append(f[0:4])
        pel_acc.append(f[4:7])
        pel_gyr.append(f[7:10])
        tho_quats.append(f[10:14])
        tho_acc.append(f[14:17])
        tho_gyr.append(f[17:20])
    
    return {
        'timestamps': np.array(timestamps),
        'pelvis_quats': np.array(pel_quats),
        'pelvis_acc': np.array(pel_acc),
        'pelvis_gyro': np.array(pel_gyr),
        'thorax_quats': np.array(tho_quats),
        'thorax_acc': np.array(tho_acc),
        'thorax_gyro': np.array(tho_gyr),
    }
```

## License

Data provided for academic research and non-commercial use only.
See repository LICENSE.
