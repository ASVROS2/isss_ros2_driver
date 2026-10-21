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

The packet length is `12 + 6 × num_samples`, so a reader must know the
configured sample count; framing is by searching for the header pattern
(bytes 0, 1, 4 and 9). Recorded logs do not start on a packet boundary.

## Timing (observed)

Pings arrive once per two-way travel time for the configured range, at roughly
1500 m/s: 13.4 ms at 10 m, 20.1 ms at 15 m, 26.8 ms at 20 m, 53.5 ms at 40 m.
`isss.sim.SimulatedSonar` paces pings the same way by default.

## Open questions

These need answering against data or hardware; until then the code keeps the
behaviour the drivers already had, and the golden test pins decoding (except
bearing) so any change is deliberate.

1. **Phase units and bearing.** Both drivers compute
   `asin(raw_phase × c / (12.5 MHz × 3.2 mm))` on the raw count. 12.5 MHz looks
   like the ADC clock rather than the acoustic carrier, and with these numbers
   the argument exceeds 1 for |raw| > ~28 counts, so almost every bearing clamps
   to ±90°. `isss_processing/plot_isss.py` instead treats `raw / 512` as degrees,
   subtracts a `19 × 512` offset, uses a 700 kHz carrier and a `3 × 3.2 mm`
   baseline. `ASVROS2/post_process_smarc` recomputes bearing from the published
   phase in degrees, still with 12.5 MHz and no clamp.
2. **Port / starboard.** `post_process_smarc` swaps the two channels before use,
   which suggests the driver's labels are reversed. Unconfirmed.
3. **Header bytes 2–3.** Constant 6007; meaning unknown.
4. **Ping time.** 32-bit counter of 100 µs wraps after ~4.97 days; its epoch
   (device boot?) is unknown, which matters for `use_device_time`.
5. **Does streaming start on connect** or only after configuration? Unknown;
   the simulator streams on connect.
