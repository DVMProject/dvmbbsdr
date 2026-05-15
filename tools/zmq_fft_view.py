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
from typing import Dict, List

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


# Fixed, tool-controlled topics used by this viewer.
IQ_WIDEBAND_TOPIC = "wb-iq"
IQ_MODEM_TOPIC_PREFIX = "modem-iq-"
RADIO_STATUS_TOPIC = "radio-state"


def modem_iq_topic(modem_id: int) -> str:
    return f"{IQ_MODEM_TOPIC_PREFIX}{modem_id}"


class FftTabPane(Qt.QWidget):
    def __init__(
        self,
        address: str,
        topic: str,
        fft_size: int,
        center_freq: float,
        sample_rate: float,
        update_time: float,
        line_label: str,
        title: str,
    ):
        super().__init__()

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
            title,
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
        self._fft.set_line_label(0, line_label)

        # GNU Radio Qt widgets expose qwidget() on recent versions.
        # Keep a fallback for bindings that still expose pyqwidget().
        if hasattr(self._fft, "qwidget"):
            raw_widget = self._fft.qwidget()
        else:
            raw_widget = self._fft.pyqwidget()

        fft_widget = sip_wrap(raw_widget)
        layout.addWidget(fft_widget)

        self._tb = gr.top_block(f"dvmbbsdr-fft-tab-{line_label}")
        self._tb.connect((self._src, 0), (self._fft, 0))

    def start(self):
        self._tb.start()

    def stop(self):
        self._tb.stop()
        self._tb.wait()

    def set_frequency_range(self, center_freq: float, sample_rate: float):
        if sample_rate <= 0:
            return
        if hasattr(self._fft, "set_frequency_range"):
            self._fft.set_frequency_range(center_freq, sample_rate)


class ZmqFftViewer(Qt.QWidget):
    def __init__(
        self,
        address: str,
        sample_rate: float,
        center_freq: float,
        fft_size: int,
        update_time: float,
        title: str,
        runtime_status_zmq_address: str,
        device_index: int,
        poll_ms: int,
    ):
        Qt.QWidget.__init__(self)

        self.setWindowTitle(title)
        qtgui_util.check_set_qss()

        layout = Qt.QVBoxLayout()
        self.setLayout(layout)

        self._tabs = Qt.QTabWidget(self)
        layout.addWidget(self._tabs)

        self._iq_address = address
        self._fft_size = fft_size
        self._update_time = update_time

        self._wideband_tab = FftTabPane(
            address=address,
            topic=IQ_WIDEBAND_TOPIC,
            fft_size=fft_size,
            center_freq=center_freq,
            sample_rate=sample_rate,
            update_time=update_time,
            line_label="RX Wideband IQ",
            title="Wideband RX IQ FFT",
        )
        self._tabs.addTab(self._wideband_tab, "Wideband")

        self._channel_tabs: Dict[int, FftTabPane] = {}

        self._runtime_status_zmq_address = runtime_status_zmq_address
        self._device_index = device_index
        self._last_runtime_signature = None
        self._status_zmq_ctx = None
        self._status_zmq_sock = None

        self._current_center_freq = center_freq
        self._current_sample_rate = sample_rate

        if self._runtime_status_zmq_address and zmq is not None:
            try:
                self._status_zmq_ctx = zmq.Context.instance()
                self._status_zmq_sock = self._status_zmq_ctx.socket(zmq.SUB)
                # Subscribe broadly and validate topic frame in software so we
                # can consume both legacy unframed JSON and topic-framed payloads.
                self._status_zmq_sock.setsockopt(zmq.SUBSCRIBE, b"")
                self._status_zmq_sock.connect(self._runtime_status_zmq_address)
            except Exception:
                self._status_zmq_sock = None
        elif self._runtime_status_zmq_address and zmq is None:
            raise RuntimeError("pyzmq is required for --runtime-status-zmq-address")

        # Ensure initial axis setup is explicit and can be updated later.
        self._set_frequency_range(center_freq, sample_rate)

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

        self._current_center_freq = center_freq
        self._current_sample_rate = sample_rate
        self._wideband_tab.set_frequency_range(center_freq, sample_rate)

        for tab in self._channel_tabs.values():
            tab.set_frequency_range(center_freq, sample_rate)

    def _ensure_channel_tabs(self, modem_ids: List[int]):
        wanted = set(modem_ids)

        # Remove tabs for channels that are no longer present.
        for modem_id in list(self._channel_tabs.keys()):
            if modem_id in wanted:
                continue

            pane = self._channel_tabs.pop(modem_id)
            idx = self._tabs.indexOf(pane)
            if idx >= 0:
                self._tabs.removeTab(idx)
            pane.stop()
            pane.deleteLater()

        # Create tabs for new channels.
        for modem_id in sorted(wanted):
            if modem_id in self._channel_tabs:
                continue

            pane = FftTabPane(
                address=self._iq_address,
                topic=modem_iq_topic(modem_id),
                fft_size=self._fft_size,
                center_freq=self._current_center_freq,
                sample_rate=self._current_sample_rate,
                update_time=self._update_time,
                line_label=f"Modem {modem_id} IQ",
                title=f"Modem {modem_id} FFT",
            )
            pane.start()
            self._channel_tabs[modem_id] = pane
            self._tabs.addTab(pane, f"Modem {modem_id}")

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
                # Ignore unrelated topic-framed messages on the same socket.
                topic = parts[0].decode("utf-8", errors="ignore")
                if topic and topic != RADIO_STATUS_TOPIC:
                    continue
                payload = parts[-1]

            try:
                status = json.loads(payload.decode("utf-8", errors="ignore"))
            except Exception:
                continue

            devices = status.get("devices", [])
            if not isinstance(devices, list):
                continue

            channels = status.get("channels", [])
            modem_ids = []
            if isinstance(channels, list):
                for ch in channels:
                    if not isinstance(ch, dict):
                        continue
                    try:
                        modem_ids.append(int(ch.get("modemId", -1)))
                    except (TypeError, ValueError):
                        continue
            self._ensure_channel_tabs([m for m in modem_ids if m > 0])

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

    def start(self):
        self._wideband_tab.start()

    def stop(self):
        self._wideband_tab.stop()
        for pane in list(self._channel_tabs.values()):
            pane.stop()


def sip_wrap(widget):
    # Some GNU Radio builds return an already-wrapped QWidget from qwidget().
    if isinstance(widget, QtWidgets.QWidget):
        return widget

    # Older bindings can return a raw pointer value that needs wrapping.
    return sip.wrapinstance(int(widget), QtWidgets.QWidget)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe to fixed dvmbbsdr ZeroMQ topics and show tabbed live FFT"
    )
    parser.add_argument(
        "--address",
        default="tcp://127.0.0.1:5557",
        help="ZeroMQ SUB address to connect for IQ streams (default: tcp://127.0.0.1:5557)",
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
        help="Optional RadioManager runtime status ZMQ PUB address (default: tcp://127.0.0.1:5567)",
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
        title=args.title,
        runtime_status_zmq_address=args.runtime_status_zmq_address,
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
        app.quit()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    app.aboutToQuit.connect(_shutdown)
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
