"""Puts the repo root and the generated stubs on sys.path for every test."""

from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(_ROOT), str(_ROOT / "gen")]
