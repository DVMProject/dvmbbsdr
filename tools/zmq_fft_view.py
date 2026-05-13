#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#/*
# * Digital Voice Modem - Baseband SDR RF Runtime
# * GPLv2 Open Source. Use is subject to license terms.
# * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
# *
# *  Copyright (C) 2026 Bryan Biedenkapp, N2PLL
# *
# */
"""
Headless companion viewer for dvmbbsdr RX IQ ZeroMQ tap.

Subscribes to a GNU Radio ZeroMQ PUB stream of complex float32 IQ samples and
renders a live FFT window using GNU Radio Qt GUI blocks.
"""

import argparse
import json
import signal
import sys

try:
    import zmq
except Exception:
    zmq = None

from gnuradio import fft
from gnuradio import gr
from gnuradio import qtgui
from gnuradio import zeromq
from gnuradio.qtgui import util as qtgui_util
from PyQt5 import Qt
from PyQt5 import QtWidgets
from PyQt5 import sip


class ZmqFftViewer(gr.top_block, Qt.QWidget):
    def __init__(
        self,
        address: str,
        sample_rate: float,
        center_freq: float,
        fft_size: int,
        update_time: float,
        topic: str,
        title: str,
        runtime_status_zmq_address: str,
        runtime_status_zmq_topic: str,
        device_index: int,
        poll_ms: int,
    ):
        gr.top_block.__init__(self, title)
        Qt.QWidget.__init__(self)

        self.setWindowTitle(title)
        qtgui_util.check_set_qss()

        layout = Qt.QVBoxLayout()
        self.setLayout(layout)

        self._src = zeromq.sub_source(
            gr.sizeof_gr_complex,
            1,
            address,
            100,
            False,
            -1,
            topic,
        )

        self._fft = qtgui.freq_sink_c(
            fft_size,
            fft.window.WIN_BLACKMAN_HARRIS,
            center_freq,
            sample_rate,
            "RX IQ FFT",
            1,
            None,
        )
        self._fft.set_update_time(update_time)
        self._fft.set_y_axis(-140, 10)
        self._fft.set_y_label("Relative Gain", "dB")
        self._fft.set_fft_average(0.2)
        self._fft.enable_grid(True)
        self._fft.enable_axis_labels(True)
        if hasattr(self._fft, "set_plot_pos_half"):
            self._fft.set_plot_pos_half(False)
        self._fft.set_line_label(0, "RX IQ")

        self._runtime_status_zmq_address = runtime_status_zmq_address
        self._runtime_status_zmq_topic = runtime_status_zmq_topic
        self._device_index = device_index
        self._last_runtime_signature = None
        self._status_zmq_ctx = None
        self._status_zmq_sock = None

        if self._runtime_status_zmq_address and zmq is not None:
            try:
                self._status_zmq_ctx = zmq.Context.instance()
                self._status_zmq_sock = self._status_zmq_ctx.socket(zmq.SUB)
                if self._runtime_status_zmq_topic:
                    self._status_zmq_sock.setsockopt(zmq.SUBSCRIBE, self._runtime_status_zmq_topic.encode("utf-8"))
                else:
                    self._status_zmq_sock.setsockopt(zmq.SUBSCRIBE, b"")
                self._status_zmq_sock.connect(self._runtime_status_zmq_address)
            except Exception:
                self._status_zmq_sock = None
        elif self._runtime_status_zmq_address and zmq is None:
            raise RuntimeError("pyzmq is required for --runtime-status-zmq-address")

        # Ensure initial axis setup is explicit and can be updated later.
        self._set_frequency_range(center_freq, sample_rate)

        # GNU Radio Qt widgets expose qwidget() on recent versions.
        # Keep a fallback for bindings that still expose pyqwidget().
        if hasattr(self._fft, "qwidget"):
            raw_widget = self._fft.qwidget()
        else:
            raw_widget = self._fft.pyqwidget()

        fft_widget = sip_wrap(raw_widget)
        layout.addWidget(fft_widget)

        self.connect((self._src, 0), (self._fft, 0))

        self._runtime_timer = None
        if self._status_zmq_sock is not None:
            self._runtime_timer = Qt.QTimer(self)
            self._runtime_timer.setInterval(max(100, poll_ms))
            self._runtime_timer.timeout.connect(self._refresh_runtime_status)
            self._runtime_timer.start()
            self._refresh_runtime_status()

    def _set_frequency_range(self, center_freq: float, sample_rate: float):
        if sample_rate <= 0:
            return
        if hasattr(self._fft, "set_frequency_range"):
            self._fft.set_frequency_range(center_freq, sample_rate)

    def _refresh_runtime_status(self):
        self._refresh_runtime_status_zmq()

    def _refresh_runtime_status_zmq(self):
        if self._status_zmq_sock is None:
            return

        while True:
            try:
                parts = self._status_zmq_sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception:
                break

            if not parts:
                continue

            if len(parts) == 1:
                payload = parts[0]
            else:
                payload = parts[-1]

            try:
                status = json.loads(payload.decode("utf-8", errors="ignore"))
            except Exception:
                continue

            devices = status.get("devices", [])
            if not isinstance(devices, list):
                continue

            selected = None
            for dev in devices:
                if isinstance(dev, dict) and int(dev.get("index", -1)) == self._device_index:
                    selected = dev
                    break

            if selected is None:
                continue

            try:
                center = float(selected.get("rxCenter", 0.0))
                sample_rate = float(selected.get("sampleRate", 0.0))
            except (TypeError, ValueError):
                continue

            signature = (center, sample_rate)
            if signature == self._last_runtime_signature:
                continue

            self._set_frequency_range(center, sample_rate)
            self._last_runtime_signature = signature


def sip_wrap(widget):
    # Some GNU Radio builds return an already-wrapped QWidget from qwidget().
    if isinstance(widget, QtWidgets.QWidget):
        return widget

    # Older bindings can return a raw pointer value that needs wrapping.
    return sip.wrapinstance(int(widget), QtWidgets.QWidget)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe to dvmbbsdr RX IQ ZeroMQ stream and show live FFT"
    )
    parser.add_argument(
        "--address",
        default="tcp://127.0.0.1:5557",
        help="ZeroMQ SUB address to connect (default: tcp://127.0.0.1:5557)",
    )
    parser.add_argument(
        "--topic",
        default="",
        help="PUB/SUB topic filter (must match rxIqTapTopic if set)",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=960000.0,
        help="Sample rate in Hz for FFT axis labeling (default: 960000)",
    )
    parser.add_argument(
        "--center-freq",
        type=float,
        default=0.0,
        help="Center frequency in Hz for FFT axis labeling (default: 0)",
    )
    parser.add_argument(
        "--fft-size",
        type=int,
        default=4096,
        help="FFT size (default: 4096)",
    )
    parser.add_argument(
        "--update-time",
        type=float,
        default=0.1,
        help="FFT GUI update interval in seconds (default: 0.1)",
    )
    parser.add_argument(
        "--title",
        default="dvmbbsdr ZMQ FFT Viewer",
        help="Window title",
    )
    parser.add_argument(
        "--runtime-status-zmq-address",
        default="tcp://127.0.0.1:5567",
        help="Optional RadioManager runtime status ZMQ PUB address for dynamic center/sample-rate updates (tcp://127.0.0.1:5567)",
    )
    parser.add_argument(
        "--runtime-status-zmq-topic",
        default="radio-state",
        help="Optional RadioManager runtime status ZMQ topic filter (default: radio-state)",
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="SDR device index to track from runtime status ZMQ data (default: 0)",
    )
    parser.add_argument(
        "--poll-ms",
        type=int,
        default=500,
        help="Runtime status poll interval in milliseconds (default: 500)",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    app = Qt.QApplication(sys.argv)

    tb = ZmqFftViewer(
        address=args.address,
        sample_rate=args.sample_rate,
        center_freq=args.center_freq,
        fft_size=args.fft_size,
        update_time=args.update_time,
        topic=args.topic,
        title=args.title,
        runtime_status_zmq_address=args.runtime_status_zmq_address,
        runtime_status_zmq_topic=args.runtime_status_zmq_topic,
        device_index=args.device_index,
        poll_ms=args.poll_ms,
    )

    tb.start()
    tb.show()

    shutting_down = False

    def _shutdown(*_):
        nonlocal shutting_down
        if shutting_down:
            return
        shutting_down = True
        if getattr(tb, "_status_zmq_sock", None) is not None:
            try:
                tb._status_zmq_sock.close(0)
            except Exception:
                pass
        tb.stop()
        tb.wait()
        app.quit()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    app.aboutToQuit.connect(_shutdown)
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
