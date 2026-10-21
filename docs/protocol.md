# ISSS protocol

The one description of how the DeepVision Interferometric Side Scan Sonar talks
over the network. Code in `isss_core/src/isss/protocol.py` implements it; the
manufacturer's reference reader is `ISSSReader_ubuntu.cpp` in
[`julRusVal/isss_processing`](https://github.com/julRusVal/isss_processing).

"Observed" means checked against recorded logs; "documented" means it comes from
the manufacturer reference and has not been checked independently.

## Connection

| Parameter | Default |
|---|---|
| IP | `192.168.2.62` |
| Port | `57349` (`0xE005`), TCP |

Configure the host with a static IP on the same subnet as the sonar.

## Register writes (documented)

Each write is 4 bytes: `[0xFE, register, value_hi, value_lo]`.

| Register | Address | Meaning |
|---|---|---|
| Decimation | `0x10` | `int(range / num_samples / 750 * (12.5e6 / 3))` |
| Num samples | `0x11` | Samples per channel (default 1000) |
| Num phases | `0x12` | Phase count (default 2; 0 = off) |

The manufacturer reference has the `0x12` write commented out; the drivers send
it. Channel registers at `0x20` / `0x30` appear in the reference but are not
described and are not implemented.

## Data packets

**Header, 12 bytes, big-endian**

| Byte | Field | Notes |
|---|---|---|
| 0–1 | `0xFE 0x05` | Start of packet |
| 2–3 | "Packet size" | **Observed: always 6007**, also for 2000-sample pings (12012 bytes apart). Not a packet size; do not use it to frame packets. |
| 4 | Type | `0xE1` = ISSS (the only type seen), `0xE2` = 8-bit sonar (not implemented) |
| 5–8 | Ping time | Device counter in 100 µs units |
| 9 | Frequency ID | `0x01` observed |
| 10–11 | Decimation | Echoes register `0x10` (observed: 55, 83, 111, 222 for 10/15/20/40 m at 1000 samples) |

**Payload: `num_samples` × 6 bytes**

Per sample: port phase `[15:8]`, port phase `[7:0]`, port RSS, starboard phase
`[15:8]`, starboard phase `[7:0]`, starboard RSS. Phase is a signed 16-bit value
(values above 32768 wrap negative).

Sample spacing is `decimation × 3 / 12.5 MHz` (26.6 µs at decimation 111), so
the slant range of bin `i` is about `i × spacing × c / 2`. The zero offset of
that range has not been calibrated.

The packet length is `12 + 6 × num_samples`, so a reader must know the
configured sample count; framing is by searching for the header pattern
(bytes 0, 1, 4 and 9). Recorded logs do not start on a packet boundary.

## Timing (observed)

Pings arrive once per two-way travel time for the configured range, at roughly
1500 m/s: 13.4 ms at 10 m, 20.1 ms at 15 m, 26.8 ms at 20 m, 53.5 ms at 40 m.
`isss.sim.SimulatedSonar` paces pings the same way by default.

## Phase and depression

The phase of each sample is the electrical phase difference between the two
receivers of that side, as a signed count. **One cycle (2π) is 20480 counts**:
the counts fill ±10240 and wrap there. A return at depression angle β (below the
sensor's horizontal, toward that side) gives

```
counts = K · sin(β0 − β)        so        β = β0 − asin(counts / K)
```

the response of a two-element interferometer, `K = 20480 · B / λ`. Both sides use
the same formula and sign, and `port` is the port side for intensity and phase
(the channel swap in `ASVROS2/post_process_smarc` is not needed).

| Parameter | Default | Meaning |
|---|---|---|
| `counts_per_unit_sine` (K) | 15218 | Counts per unit sine of the off-boresight angle. With the 3.2 mm baseline this implies roughly 330–340 kHz acoustic frequency (inferred, not documented). |
| `boresight_depression_deg` (β0) | 44.5 | Depression of zero phase; the arrays look about 45° down |

The defaults come from fitting phase against a surveyed mesh of the Beckholmen
dry dock (December 2025 stick recordings), validated on two held-out recordings:
starboard points land a median 0.1 m from the mesh, against about 4 m with the
earlier driver formula. Method and code: `docs/usage/isss-ingestion.md` in
[`julRusVal/sonar3d_reference`](https://github.com/julRusVal/sonar3d_reference);
`isss.protocol.depression_from_counts` gives identical values to its
`experimental/isss/phase.py`.

Consequences:

- K is 1.49 × 10240, so returns more than `asin(10240 / K)` ≈ 42° from the
  boresight **alias**: their counts wrap and decode to the wrong angle. Nadir
  (45.5° off boresight) and near-horizontal returns are affected. One sample
  cannot reveal this; it needs geometry (first-surface rules, bottom tracking).
- β0 contains the mounting tilt, so it changes with the mount, and K scales
  with the speed of sound (`λ = c / f`). Recalibrate when either changes.
- Weak returns collapse to a few fixed phase values ("bias states"), and the
  port phase has been poor. Filter on intensity before trusting a depression.

The earlier drivers computed `asin(counts · c / (12.5 MHz · 3.2 mm))`: 12.5 MHz
is the sample clock, not the carrier, and the result is off by a factor of 512,
clamping almost every bearing to ±90°. Their "phase difference" in radians was
`counts / 512 · π / 180`, a ninth of the true phase.

## Recorded data in hand

- `isss_data/*.bin` (Aug 2025, 20 m): intensity is flat (about 51) at every
  range, with no bottom return; most likely recorded out of the water. Phase stays
  within ±10240, as above. Good for protocol tests, not for geometry.
- [`julRusVal/isss_processing`](https://github.com/julRusVal/isss_processing)
  `data/*.bin` (July 2024, 10–40 m, file names `switch`, `set0_1`, `phase2`):
  a flat seabed, but a **different phase configuration**. Strong starboard
  returns lie in `[0, 20480)` (the same cycle, shifted), in parallel tracks
  about 5120 counts (a quarter cycle) apart, which suggests the phase reference
  was switched between pings. Port phase sits in a fixed band with no geometry.
  A flat-bottom fit does not reproduce the 2025 calibration, so these logs need
  their own calibration and decode with the current defaults gives wrong angles.

## Open questions

1. **Header bytes 2–3.** Constant 6007; meaning unknown.
2. **Ping time.** 32-bit counter of 100 µs wraps after ~4.97 days; its epoch
   (device boot?) is unknown, which matters for `use_device_time`.
3. **Does streaming start on connect** or only after configuration? Unknown;
   the simulator streams on connect.
4. **Range zero offset and sound speed** were fitted together with a vertical
   offset in the mesh check and trade off; neither is pinned down independently.
5. **The July 2024 configuration**: what the switched settings were, and
   whether the current firmware can still produce them.
