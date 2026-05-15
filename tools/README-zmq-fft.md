# dvmbbsdr ZeroMQ FFT Viewer

This tool subscribes to the dvmbbsdr RX IQ ZeroMQ tap and displays tabbed live FFT views:

- `Wideband` tab: full SDR device wideband IQ spectrum.
- `Modem N` tabs: one tab per modem channel (created from runtime metadata), each fed by a fixed modem IQ topic.

The viewer uses fixed topics by design:

- IQ stream (`tcp://127.0.0.1:5557`):
  - Wideband topic: `wb-iq`
  - Per-modem topics: `modem-iq-<modemId>` (example: `modem-iq-1`)
- Runtime metadata (`tcp://127.0.0.1:5567`):
  - Topic: `radio-state`

Topic values are not configurable from CLI.

## 1) Enable RX IQ tap in dvmbbsdr config

Set one of these in your SDR config:

```yaml
sdr:
  runtimeStatusPubAddress: "tcp://127.0.0.1:5567"
  defaults:
    rxIqTapAddress: "tcp://127.0.0.1:5557"
```

Or per device:

```yaml
sdr:
  devices:
    - args: "driver=rtl,index=0"
      rxIqTapAddress: "tcp://127.0.0.1:5557"
```

## 2) Run dvmbbsdr

Start dvmbbsdr normally. Confirm log line similar to:

- `SDR 0 RX IQ tap enabled (tcp://127.0.0.1:5557)`

## 3) Launch FFT viewer

From repository root:

```bash
python3 tools/zmq_fft_view.py --address tcp://127.0.0.1:5557 --sample-rate 960000
```

To follow dynamic retunes from `RadioManager` automatically via ZMQ (recommended):

```bash
python3 tools/zmq_fft_view.py \
  --address tcp://127.0.0.1:5557 \
  --runtime-status-zmq-address tcp://127.0.0.1:5567 \
  --device-index 0
```

## Notes

- This viewer expects complex float32 IQ from GNU Radio ZeroMQ blocks.
- IQ topics are fixed in runtime and viewer (`wb-iq` and `modem-iq-<modemId>`).
- Runtime status topic is fixed to `radio-state`.
- `--sample-rate` controls FFT axis scaling and should match the SDR device sample rate.
- `--center-freq` is optional and only affects axis labeling.
- When `--runtime-status-zmq-address` is set, the viewer updates the FFT axis at runtime
  from manager-published state (`rxCenter` and `sampleRate`) and overrides static axis values.
