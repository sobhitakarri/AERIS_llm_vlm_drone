"""The LiteWing UDP driver against a stand-in for the ESP-Drone firmware's checksummed framing."""
import socket
import threading
import time

import pytest

cflib_crtp = pytest.importorskip("cflib.crtp")
from cflib.crtp.crtpstack import CRTPPacket  # noqa: E402

from backend.drone import litewing_udp as udp  # noqa: E402


class FirmwareStandIn:
    """wifi_esp32.c: accept only datagrams whose last byte is the byte-sum, reply with a checksum."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.accepted, self.dropped = [], 0
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(data) >= 2 and sum(data[:-1]) & 0xFF == data[-1]:
                self.accepted.append(data[:-1])
                reply = bytes([data[0]]) + b"\x01\x02"
                self.sock.sendto(reply + bytes([sum(reply) & 0xFF]), addr)
            else:
                self.dropped += 1

    def close(self):
        self._stop = True
        self.sock.close()


@pytest.fixture
def firmware():
    fw = FirmwareStandIn()
    yield fw
    fw.close()


def test_checksum_is_the_byte_sum_mod_256():
    assert udp.checksum(b"\xff") == 0xFF
    assert udp.checksum(bytes([200, 100])) == 44


def test_official_uri_maps_to_the_checksummed_driver_with_the_firmware_port():
    assert udp.to_driver_uri("udp://192.168.43.42") == "lwudp://192.168.43.42:2390"
    assert udp.to_driver_uri("udp://192.168.43.42:2390") == "lwudp://192.168.43.42:2390"
    assert udp.to_driver_uri("sim://litewing") == "sim://litewing"


def test_round_trip_with_checksum(firmware):
    cflib_crtp.init_drivers()
    udp.register()
    drv = cflib_crtp.get_link_driver(f"lwudp://127.0.0.1:{firmware.port}", None, lambda m: None)
    assert isinstance(drv, udp.LiteWingUdpDriver)
    try:
        drv.send_packet(CRTPPacket(header=0x2D, data=b"\x05\x06"))
        pk = drv.receive_packet(wait=2.0)
        assert pk is not None and pk.header == 0x2D and bytes(pk.data) == b"\x01\x02"
        assert firmware.accepted == [b"\x2d\x05\x06"] and firmware.dropped == 0
    finally:
        drv.close()


def test_stock_udp_driver_is_dropped_by_the_firmware(firmware):
    """Why this driver exists: cflib 0.1.32's own UDP driver sends no checksum."""
    cflib_crtp.init_drivers()
    drv = cflib_crtp.get_link_driver(f"udp://127.0.0.1:{firmware.port}", None, lambda m: None)
    try:
        drv.send_packet(CRTPPacket(header=0x2D, data=b"\x05\x06"))
        assert drv.receive_packet(wait=0.6) is None
        time.sleep(0.1)
        assert firmware.dropped >= 1 and firmware.accepted == []
    finally:
        drv.close()


def test_frames_with_a_bad_checksum_are_ignored(firmware):
    cflib_crtp.init_drivers()
    udp.register()
    drv = cflib_crtp.get_link_driver(f"lwudp://127.0.0.1:{firmware.port}", None, lambda m: None)
    try:
        drv.socket.send(b"\x2d\x05\x06\x00")            # wrong checksum: firmware drops it, no reply
        assert drv.receive_packet(wait=0.5) is None
        assert firmware.dropped == 1
    finally:
        drv.close()
