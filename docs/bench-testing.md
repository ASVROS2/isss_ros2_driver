# Bench testing the ISSS

Out of the water the sonar hears no echoes, so intensity and phase say little.
Everything else can be checked on a desk: the protocol, configuration, timing,
the device clock, and our driver and logging. `isss-bench` does that.

## Before you start

- **Running in air.** Check DeepVision's guidance on operating the transducers
  out of the water before powering up; keep runs short if in doubt. The full bench
  takes about two minutes, `--quick` about thirty seconds.
- Give the host a static IP on the sonar's subnet (default sonar address
  `192.168.2.62`, TCP port 57349) and check `ping 192.168.2.62`.
- Note the time the sonar was powered on. The clock step reports the device
  counter, and comparing it with the uptime tells us whether the counter starts
  at power-on (open question 2 in [protocol.md](protocol.md)).

## Run it

Rehearse against the simulator first; this is what a pass looks like:

```bash
cd isss_core
uv run isss-sim &                       # fake sonar on 127.0.0.1:57349
uv run isss-bench --ip 127.0.0.1 --quick
kill %1
```

Then the sonar:

```bash
uv run isss-bench --ip 192.168.2.62 -o bench_$(date +%Y%m%d)_stick
```

It writes `README.md` (a table of steps), `bench.json` (every measurement) and
`logs/` (a `.bin` log with its sidecar from the logging step). Afterwards it
leaves the sonar at 20 m, 1000 samples, 2 phases. `--ranges` and `--samples`
change the sweeps.

To watch the device: `uv run isss-view --ip 192.168.2.62`, before or after the
bench rather than during it. Whether the sonar accepts a second client is not
known (the simulator does not), and a second client's register writes would
disturb the bench.

## The steps

| Step | Checks | Pass when |
|---|---|---|
| connect | TCP connection | it connects |
| stream on connect | does data arrive before any register write; framed without assuming a sample count (from header spacing) | info |
| baseline configuration | 20 m / 1000 samples: packet size, decimation echoed in the header, ping interval, framing | size 6012, decimation 111, interval within 15 % of 26.7 ms, no framing errors |
| range sweep | the same at 10, 20, 40 m; fits interval against range | every range as expected; the fit gives the pacing sound speed (logs say ~1500 m/s) |
| sample count sweep | 500, 1000, 2000 samples at 20 m | packet size and decimation follow the sample count |
| phase setting | register `0x12` at 0 and 2: what the phase bytes do | info |
| configuration across connections | configure 30 m, reconnect without writing, read the header | info: kept or reset |
| device clock | 20 s of pings: device counter against host clock | no gaps or backwards steps; reports drift (ppm), arrival jitter, counter value |
| driver and logging | `SonarDriver` with logging for 5 s | log, sidecar and delivered pings agree; health checks attached |
| driver reconnect | drop the connection under the driver | pings resume |

Steps marked info answer questions nobody has checked yet; there is no wrong
answer, but write the answers into the open questions of
[protocol.md](protocol.md) (and the simulator, `isss_core/src/isss/sim.py`, if
the device behaves differently from it).

## By hand

Some things need a person:

- **Cable pull.** Run `isss-view --ip …` and unplug the Ethernet for ten seconds:
  the view should freeze, then resume after the driver's reconnect interval.
- **Power cycle.** Power the sonar off and on while `isss-view` runs; note
  whether it resumes and whether the configuration and the ping counter reset
  (then rerun the "configuration across connections" and "device clock" steps).
- **ROS node.** On a ROS machine: `colcon test --packages-select isss_core isss_ros2_driver`,
  then launch the node against the sonar and check `ros2 topic hz /isss/raw`
  against the bench's ping rate and `ros2 topic echo /isss/status`.
