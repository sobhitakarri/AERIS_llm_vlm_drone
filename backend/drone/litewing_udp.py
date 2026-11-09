"""
CRTP-over-UDP driver for the LiteWing / ESP-Drone firmware.

The firmware (components/drivers/general/wifi/wifi_esp32.c) ends every datagram
with a checksum byte, the sum of the CRTP bytes mod 256: it drops incoming
packets without a valid one and appends one to what it sends. Older cflib UDP
drivers did this; cflib 0.1.32's rewritten driver does not ("removed checksum"),
so stock cflib never gets a handshake from a LiteWing.

This driver adds the checksum, keeps the cflib interface, and registers under
its own scheme (`lwudp://host:port`) so it never shadows the stock `udp://`
driver used for other vehicles. `to_driver_uri` maps the official
`udp://192.168.43.42` form to it.
"""
from __future__ import annotations

import logging
import queue
import socket
import struct
import threading
from urllib.parse import urlparse

LITEWING_PORT = 2390            # UDP_SERVER_PORT in the firmware
SCHEME = "lwudp://"

logger = logging.getLogger(__name__)

try:
    from cflib.crtp.crtpdriver import CRTPDriver
    from cflib.crtp.crtpstack import CRTPPacket
    from cflib.crtp.exceptions import WrongUriType
    CFLIB = True
except ImportError:  # pragma: no cover - depends on the machine
    CFLIB = False
    CRTPDriver = object


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def to_driver_uri(uri: str) -> str:
    """udp://host[:port] (official form) -> lwudp://host:port. Other URIs unchanged."""
    if uri.startswith("udp://"):
        rest = uri[len("udp://"):].rstrip("/")
        if ":" not in rest:
            rest += f":{LITEWING_PORT}"
        return SCHEME + rest
    return uri


class _Receiver(threading.Thread):
    def __init__(self, sock, in_queue, on_error):
        super().__init__(name="LiteWingUdpRx", daemon=True)
        self._sock, self._queue, self._on_error = sock, in_queue, on_error
        self._stop_flag = False

    def stop(self):
        self._stop_flag = True
        try:
            self.join(timeout=2.0)
        except Exception:
            pass

    def run(self):
        self._sock.settimeout(0.5)
        while not self._stop_flag:
            try:
                data = self._sock.recv(1024)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop_flag and self._on_error:
                    self._on_error(f"LiteWing UDP receive failed: {exc}")
                return
            if len(data) < 2 or checksum(data[:-1]) != data[-1]:
                continue                                   # not a valid frame
            self._queue.put(CRTPPacket(header=data[0], data=data[1:-1]))


class LiteWingUdpDriver(CRTPDriver):
    """cflib link driver: one CRTP packet per datagram, plus a checksum byte."""

    def __init__(self):
        super().__init__()
        self.socket = None
        self.in_queue = queue.Queue()
        self._thread = None
        self.link_error_callback = None
        self.uri = ""

    def connect(self, uri, radio_link_statistics_callback, link_error_callback):
        if not uri.startswith(SCHEME):
            raise WrongUriType("Not a LiteWing UDP URI")
        if self.socket is not None:
            raise Exception("Link already open!")
        parse = urlparse(uri)
        self.uri, self.link_error_callback = uri, link_error_callback
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((parse.hostname, parse.port or LITEWING_PORT))
        self._thread = _Receiver(self.socket, self.in_queue, link_error_callback)
        self._thread.start()

    def send_packet(self, pk):
        if self.socket is None:
            return
        try:
            raw = bytes([pk.header]) + bytes(pk.data)
            self.socket.send(raw + bytes([checksum(raw)]))
        except Exception as exc:
            if self.link_error_callback:
                self.link_error_callback(f"LiteWing UDP send failed: {exc}")

    def receive_packet(self, wait=0):
        try:
            if wait == 0:
                return self.in_queue.get(False)
            return self.in_queue.get(True, None if wait < 0 else wait)
        except queue.Empty:
            return None

    def close(self):
        if self._thread:
            self._thread.stop()
            self._thread = None
        try:
            if self.socket:
                self.socket.close()
        except Exception:
            pass
        self.socket = None
        self.link_error_callback = None

    def get_status(self):
        return "No information available"

    def get_name(self):
        return "lwudp"

    def scan_interface(self, address=None):
        return []


def register() -> None:
    """Make cflib.crtp.get_link_driver() know the lwudp:// scheme (idempotent)."""
    import cflib.crtp
    if LiteWingUdpDriver not in cflib.crtp.CLASSES:
        cflib.crtp.CLASSES.insert(0, LiteWingUdpDriver)
