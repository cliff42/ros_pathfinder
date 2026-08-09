import math
from dataclasses import dataclass
from smbus2 import SMBus


@dataclass
class MagnetStatus:
    detected: bool
    too_weak: bool
    too_strong: bool

    @property
    def healthy(self) -> bool:
        return (
            self.detected and not self.too_weak and not self.too_strong
        )

# TODO: add link to datasheet here (find in notes doc)
# guide: https://www.instructables.com/Interfacing-With-the-AS5600-Magnetic-Encoder/
class AS5600Encoder:
    DEFAULT_ADDRESS = 0x36

    STATUS_REGISTER = 0x0B
    RAW_ANGLE_REGISTER = 0x0C  # read 2 bytes: 0x0C,0x0D

    COUNTS_PER_REVOLUTION = 4096

    MAGNET_DETECTED_MASK = 1 << 5
    MAGNET_TOO_WEAK_MASK = 1 << 4
    MAGNET_TOO_STRONG_MASK = 1 << 3

    def __init__(self, bus_number: int, address: int = DEFAULT_ADDRESS) -> None:
        self._address = address
        self._bus = SMBus(bus_number)
        self._closed = False

    def read_raw_count(self) -> int:
        self._require_open()

        data = self._bus.read_i2c_block_data(self._address, self.RAW_ANGLE_REGISTER, 2)

        if len(data) != 2:
            raise Exception(f"expected 2 bytes for angle data, got: {len(data)}")

        return ((data[0] << 8) | data[1]) & 0x0FFF

    def read_angle_rad(self) -> float:
        return (
            (self.read_raw_count() * 2 * math.pi) / self.COUNTS_PER_REVOLUTION
        )

    def read_magnet_status(self) -> MagnetStatus:
        self._require_open()

        mag_status = self._bus.read_byte_data(self._address, self.STATUS_REGISTER)

        return MagnetStatus(
            detected=bool(mag_status & self.MAGNET_DETECTED_MASK),
            too_weak=bool(mag_status & self.MAGNET_TOO_WEAK_MASK),
            too_strong=bool(mag_status & self.MAGNET_TOO_STRONG_MASK)
        )

    def close(self) -> None:
        if self._closed:
            return
        
        self._bus.close()
        self._closed = True

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("encoder has been closed")