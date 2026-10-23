# ISSS bench test: 192.168.2.62:57349

Started 2026-10-21T20:40:58.575391+00:00. Overall: **PASS**.

| Status | Step | Result |
|---|---|---|
| PASS | connect | TCP connect in 1.1 ms |
| INFO | stream on connect | streams on connect: 47 packets, 1000 samples, decimation [165] |
| PASS | baseline configuration | 110 packets of 6012 bytes, decimation 111, 26.9 ms apart |
| PASS | range sweep | all ranges as expected; interval = 1.333 ms/m · range + 0.20 ms (≈ 1501 m/s) |
| PASS | sample count sweep | packet size and decimation follow the sample count |
| INFO | phase setting | phases=0: 4360 distinct values, 0% zero; phases=2: 4432 distinct values, 0% zero |
| INFO | configuration across connections | kept: still decimation 166 (30 m) after reconnecting |
| PASS | device clock | device clock +13 ppm against host over 20 s; arrival jitter p95 1.5 ms; 0 gaps; counter at 36.3 s (0.01 h) |
| PASS | driver and logging | 186 pings at 37.2 Hz; log and sidecar agree: 1792615309.936273.bin |
| PASS | driver reconnect | pings resumed 0.7 s after the connection dropped |

Measurements for each step are in `bench.json`.
