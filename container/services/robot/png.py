"""Minimal PNG encoder (zlib + struct): 8-bit RGB and 16-bit grayscale, no filtering."""

import struct
import zlib

_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_COLOR_TYPES = {1: 0, 3: 2}  # channels -> PNG colour type (grayscale, truecolour)


def _chunk(kind: bytes, data: bytes) -> bytes:
  return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))


def encode(pixels: bytes, width: int, height: int, channels: int, bit_depth: int) -> bytes:
  """pixels: rows top to bottom, interleaved channels; 16-bit samples big-endian (as PNG stores them)."""
  row = width * channels * bit_depth // 8
  if len(pixels) != row * height:
    raise ValueError(f"{len(pixels)} bytes for {width}x{height}x{channels} at {bit_depth} bit (want {row * height})")
  # Filter type 0 (none) in front of every row.
  raw = b"".join(b"\x00" + pixels[y * row:(y + 1) * row] for y in range(height))
  header = struct.pack(">IIBBBBB", width, height, bit_depth, _COLOR_TYPES[channels], 0, 0, 0)
  return _SIGNATURE + _chunk(b"IHDR", header) + _chunk(b"IDAT", zlib.compress(raw)) + _chunk(b"IEND", b"")
