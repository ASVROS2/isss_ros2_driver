# ISSS bench test: 192.168.2.62:57349

Started 2026-10-21T20:45:25.152348+00:00. Overall: **PASS**.

| Status | Step | Result |
|---|---|---|
| PASS | connect | TCP connect in 1.1 ms |
| INFO | stream on connect | streams on connect: 14 packets, 1000 samples, decimation [165] |
| PASS | baseline configuration | 33 packets of 6012 bytes, decimation 111, 26.9 ms apart |
| PASS | range sweep | all ranges as expected; interval = 1.333 ms/m · range + 0.20 ms (≈ 1501 m/s) |
| PASS | sample count sweep | packet size and decimation follow the sample count |
| INFO | phase setting | phases=0: 4280 distinct values, 0% zero; phases=2: 4215 distinct values, 0% zero |
| INFO | configuration across connections | kept: still decimation 166 (30 m) after reconnecting |
| PASS | device clock | device clock +71 ppm against host over 6 s; arrival jitter p95 1.4 ms; 0 gaps; counter at 94.0 s (0.03 h) |
| PASS | driver and logging | 55 pings at 36.4 Hz; log and sidecar agree: 1792615542.550076.bin |
| PASS | driver reconnect | pings resumed 0.7 s after the connection dropped |

Measurements for each step are in `bench.json`.
