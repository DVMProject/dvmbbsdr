# dvmbbsdr ZeroMQ FFT Viewer

This tool subscribes to the dvmbbsdr RX IQ ZeroMQ tap and displays a live FFT.

## 1) Enable RX IQ tap in dvmbbsdr config

Set one of these in your SDR config:

```yaml
sdr:
  runtimeStatusPubAddress: "tcp://127.0.0.1:5567"
  runtimeStatusPubTopic: "radio-state"
  defaults:
    rxIqTapAddress: "tcp://127.0.0.1:5557"
    rxIqTapTopic: ""
```

Or per device:

```yaml
sdr:
  devices:
    - args: "driver=rtl,index=0"
      rxIqTapAddress: "tcp://127.0.0.1:5557"
      rxIqTapTopic: "sdr0"
```

## 2) Run dvmbbsdr

Start dvmbbsdr normally. Confirm log line similar to:

- `SDR 0 RX IQ tap enabled (tcp://127.0.0.1:5557)`

## 3) Launch FFT viewer

From repository root:

```bash
python3 tools/zmq_fft_view.py --address tcp://127.0.0.1:5557 --sample-rate 960000
```

If you configured a topic:

```bash
python3 tools/zmq_fft_view.py --address tcp://127.0.0.1:5557 --topic sdr0 --sample-rate 960000
```

To follow dynamic retunes from `RadioManager` automatically via ZMQ (recommended):

```bash
python3 tools/zmq_fft_view.py \
  --address tcp://127.0.0.1:5557 \
  --runtime-status-zmq-address tcp://127.0.0.1:5567 \
  --runtime-status-zmq-topic radio-state \
  --device-index 0
```

## Notes

- This viewer expects complex float32 IQ from GNU Radio ZeroMQ blocks.
- `--sample-rate` controls FFT axis scaling and should match the SDR device sample rate.
- `--center-freq` is optional and only affects axis labeling.
- When `--runtime-status-zmq-address` is set, the viewer updates the FFT axis at runtime
  from manager-published state (`rxCenter` and `sampleRate`) and overrides static axis values.
