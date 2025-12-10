"""
Core library for the USB-TPA64 thermal camera.

Dependencies:
    pip install pyserial pillow
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Iterable, Optional, Tuple

import serial
from serial import SerialException
from PIL import Image
import colorsys


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class USBTPA64Error(Exception):
    """Base exception for USB-TPA64 errors."""


class USBTPA64TimeoutError(USBTPA64Error):
    """Raised when a read from the device times out or is incomplete."""


class USBTPA64ProtocolError(USBTPA64Error):
    """Raised when the device returns unexpected data (wrong length, bad ID, etc.)."""


# ---------------------------------------------------------------------------
# Frame object
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    """Represents a single thermal frame from the USB-TPA64."""

    ambient_c: float
    temps: List[List[float]]  # 8x8 matrix [row][col] in °C
    timestamp: float

    @property
    def min_c(self) -> float:
        return min(t for row in self.temps for t in row)

    @property
    def max_c(self) -> float:
        return max(t for row in self.temps for t in row)

    def to_flat_list(self) -> List[float]:
        """Return the 64 temperatures as a flat list (row-major)."""
        return [t for row in self.temps for t in row]

    def to_image(
        self,
        size: Tuple[int, int] = (256, 256),
        dynamic_range: bool = True,
        vmin: Optional[float] = None,
        vmax: Optional[float] = None,
    ) -> Image.Image:
        """
        Convert this frame to a color image (blue -> red).

        Args:
            size: Output image size (width, height).
            dynamic_range:
                If True, map min->blue and max->red based on this frame's min/max.
                If False, use vmin/vmax for mapping (defaults to frame min/max).
            vmin, vmax:
                Temperature range for fixed scaling when dynamic_range=False.

        Returns:
            PIL.Image.Image (RGB)
        """
        flat = self.to_flat_list()

        if dynamic_range:
            t_min = min(flat)
            t_max = max(flat)
        else:
            t_min = self.min_c if vmin is None else vmin
            t_max = self.max_c if vmax is None else vmax

        # Avoid zero range
        if t_max == t_min:
            t_range = 1.0
        else:
            t_range = t_max - t_min

        pixels = []
        for t in flat:
            norm = (t - t_min) / t_range  # 0..1
            norm = max(0.0, min(1.0, norm))

            # Hue 240° (blue) -> 0° (red)
            hue_deg = 240.0 * (1.0 - norm)
            h = hue_deg / 360.0
            s = 1.0
            v = 1.0

            r, g, b = colorsys.hsv_to_rgb(h, s, v)
            pixels.append((int(r * 255), int(g * 255), int(b * 255)))

        img_small = Image.new("RGB", (8, 8))
        img_small.putdata(pixels)
        img_big = img_small.resize(size, Image.BILINEAR)
        return img_big


# ---------------------------------------------------------------------------
# Device class
# ---------------------------------------------------------------------------

class USBTPA64:
    """
    Low-level interface to the USB-TPA64 thermal camera.

    Example:
        from tpa64 import USBTPA64

        with USBTPA64("COM3") as cam:
            module_id, fw = cam.get_info()
            print("Module:", module_id, "FW:", fw)
            frame = cam.get_frame()
            img = frame.to_image()
            img.save("snap.png")
    """

    # Command bytes (from firmware/datasheet)
    CMD_GET_VERSION = 0x5A
    CMD_GET_THERM_PLUS_PIXELS = 0x5B
    CMD_GET_THERM_ONLY = 0x5C
    CMD_GET_PIXELS_ONLY = 0x5D
    CMD_GET_SERIAL = 0x38

    EXPECTED_MODULE_ID = 37  # 0x25

    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: float = 0.5,
        write_timeout: float = 0.5,
        open_on_init: bool = True,
    ):
        """
        Initialize the object.

        Args:
            port: Serial port name (e.g. 'COM3', '/dev/ttyUSB0').
            baudrate: Serial baudrate (default: 9600).
            timeout: Read timeout in seconds (default: 0.5).
            write_timeout: Write timeout in seconds (default: 0.5).
            open_on_init: If True, open the port immediately.
        """
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self._ser: Optional[serial.Serial] = None

        if open_on_init:
            self.open()

    # ------------------ context manager support ------------------

    def __enter__(self) -> "USBTPA64":
        if not self.is_open:
            self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ------------------ serial management ------------------

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        """Open the serial port."""
        if self.is_open:
            return
        try:
            self._ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
        except SerialException as e:
            raise USBTPA64Error(f"Failed to open port {self.port_name}: {e}") from e

    def close(self) -> None:
        """Close the serial port."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ------------------ low-level helpers ------------------

    def _ensure_open(self) -> serial.Serial:
        if not self.is_open:
            raise USBTPA64Error("Serial port is not open.")
        return self._ser  # type: ignore[return-value]

    def _read_exact(self, n: int) -> bytes:
        """
        Read exactly n bytes from the serial port, or raise USBTPA64TimeoutError.
        """
        ser = self._ensure_open()
        data = bytearray()
        while len(data) < n:
            chunk = ser.read(n - len(data))
            if not chunk:
                break
            data.extend(chunk)

        if len(data) != n:
            raise USBTPA64TimeoutError(f"Expected {n} bytes, got {len(data)}.")
        return bytes(data)

    def _send_cmd(self, cmd: int, response_length: int) -> bytes:
        """
        Send a single-byte command and read a fixed-length response.
        """
        ser = self._ensure_open()

        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        try:
            ser.write(bytes([cmd]))
        except SerialException as e:
            raise USBTPA64Error(f"Failed to write command 0x{cmd:02X}: {e}") from e

        return self._read_exact(response_length)

    # ------------------ decoding helpers ------------------

    @staticmethod
    def _decode_ambient(lo: int, hi: int) -> float:
        """
        Decode ambient/thermistor temperature from two bytes.

        Format (from module / AMG8833 thermistor):
            11-bit magnitude with 0.0625°C per LSB,
            sign bit in bit 3 of the high byte.
        """
        raw = ((hi & 0x07) << 8) | lo  # 11-bit magnitude
        temp = raw * 0.0625
        if hi & 0x08:  # sign bit
            temp = -temp
        return temp

    @staticmethod
    def _decode_pixel_temp(lo: int, hi: int) -> float:
        """
        Decode a single pixel temperature from two bytes.

        Thermopile pixels: 12-bit two's complement, 0.25°C per LSB.
        """
        # Compose 16-bit, then mask to 12-bit
        raw16 = (hi << 8) | lo
        raw12 = raw16 & 0x0FFF  # lower 12 bits

        if raw12 & 0x800:       # sign bit (bit 11)
            raw12 -= 0x1000     # convert from two's complement

        return raw12 * 0.25

    # ------------------ public API ------------------

    def get_info(self) -> Tuple[int, int]:
        """
        Get module ID and firmware version using command 0x5A.

        Returns:
            (module_id, firmware_version)

        Raises:
            USBTPA64TimeoutError, USBTPA64ProtocolError
        """
        resp = self._send_cmd(self.CMD_GET_VERSION, 2)
        module_id, fw_version = resp[0], resp[1]
        if module_id != self.EXPECTED_MODULE_ID:
            raise USBTPA64ProtocolError(
                f"Unexpected module ID: {module_id} (expected {self.EXPECTED_MODULE_ID})"
            )
        return module_id, fw_version

    def get_serial(self) -> str:
        """
        Read the 8-byte module serial number using command 0x38.

        Returns:
            Serial number as a string (ASCII, stripped of trailing NULs).
        """
        resp = self._send_cmd(self.CMD_GET_SERIAL, 8)
        return resp.decode("ascii", errors="replace").rstrip("\x00\r\n ")

    def get_ambient(self) -> float:
        """
        Read only the ambient (thermistor) temperature using command 0x5C.

        Returns:
            Ambient temperature in °C.
        """
        resp = self._send_cmd(self.CMD_GET_THERM_ONLY, 2)
        lo, hi = resp[0], resp[1]
        return self._decode_ambient(lo, hi)

    def get_pixels(self) -> List[List[float]]:
        """
        Read only the 64 thermopile pixel temperatures using command 0x5D.

        Returns:
            8x8 list-of-lists of temperatures in °C.
        """
        resp = self._send_cmd(self.CMD_GET_PIXELS_ONLY, 128)
        temps_flat: List[float] = []
        for i in range(0, 128, 2):
            lo = resp[i]
            hi = resp[i + 1]
            temps_flat.append(self._decode_pixel_temp(lo, hi))

        # Convert to 8x8 row-major
        return [temps_flat[r * 8:(r + 1) * 8] for r in range(8)]

    def get_frame(self) -> Frame:
        """
        Read a full frame: ambient + 64 pixels using command 0x5B.

        Returns:
            Frame object containing ambient, 8x8 pixel temps, and timestamp.
        """
        resp = self._send_cmd(self.CMD_GET_THERM_PLUS_PIXELS, 130)
        timestamp = time.time()

        # Ambient
        amb_lo = resp[0]
        amb_hi = resp[1]
        ambient_c = self._decode_ambient(amb_lo, amb_hi)

        # Pixels
        temps_flat: List[float] = []
        for i in range(2, 130, 2):
            lo = resp[i]
            hi = resp[i + 1]
            temps_flat.append(self._decode_pixel_temp(lo, hi))

        temps_8x8 = [temps_flat[r * 8:(r + 1) * 8] for r in range(8)]

        return Frame(ambient_c=ambient_c, temps=temps_8x8, timestamp=timestamp)

    def iter_frames(
        self,
        interval: Optional[float] = None,
    ) -> Iterable[Frame]:
        """
        Generator yielding frames continuously.

        Args:
            interval:
                Minimum time between frames in seconds. If None, frames are read
                as fast as the device responds. If set, this function will sleep
                as needed between frames.

        Yields:
            Frame objects.

        Raises:
            USBTPA64Error on fatal I/O errors.
        """
        while True:
            start = time.time()
            frame = self.get_frame()
            yield frame

            if interval is not None:
                elapsed = time.time() - start
                remaining = interval - elapsed
                if remaining > 0:
                    time.sleep(remaining)
