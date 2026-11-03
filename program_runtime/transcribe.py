"""Speech in a demo clip to text, with Muse Voice Transcribe (Meta Model API, POST /v1/asr/transcribe).

The clip's audio goes to 16 kHz mono WAV (the API takes WAV only, up to 10 min) with the ffmpeg that imageio-ffmpeg
ships; the reply is requested as text/plain, one spoken turn per line. A clip without an audio track gives "".
"""

import json
import os
from pathlib import Path
import subprocess
import tempfile
import urllib.error
import urllib.request
import uuid

import imageio_ffmpeg

BASE = "https://api.meta.ai/v1"
MODEL = "muse-voice-transcribe-1.0"
MAX_SECONDS = 600


class TranscribeError(Exception):
  pass


def wav_from_clip(clip: Path, out: Path) -> bool:
  """Writes the clip's audio as 16 kHz mono WAV; False if the clip has no audio stream."""
  ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
  try:
    probe = subprocess.run([ffmpeg, "-hide_banner", "-i", str(clip)], capture_output=True, text=True, timeout=60)
    if " Audio: " not in probe.stderr:  # ffmpeg lists every stream on input; no audio stream, nothing to send
      return False
    result = subprocess.run(
        [ffmpeg, "-y", "-v", "error", "-i", str(clip), "-vn", "-ac", "1", "-ar", "16000", "-t", str(MAX_SECONDS),
         "-f", "wav", str(out)], capture_output=True, text=True, timeout=120)
  except (OSError, subprocess.TimeoutExpired) as error:
    raise TranscribeError(f"could not extract audio: {error}") from error
  if result.returncode != 0:
    raise TranscribeError(f"could not extract audio: {result.stderr.strip()[-300:]}")
  return out.exists() and out.stat().st_size > 44  # a bare WAV header means no samples


def multipart(fields: list[tuple[str, str, str, bytes]]) -> tuple[bytes, str]:
  """(name, filename or "", content type, data) parts -> (body, content type header)."""
  boundary = uuid.uuid4().hex
  body = b""
  for name, filename, content_type, data in fields:
    disposition = f'form-data; name="{name}"' + (f'; filename="{filename}"' if filename else "")
    body += (f"--{boundary}\r\nContent-Disposition: {disposition}\r\nContent-Type: {content_type}\r\n\r\n"
             .encode() + data + b"\r\n")
  return body + f"--{boundary}--\r\n".encode(), f"multipart/form-data; boundary={boundary}"


def transcribe(clip, key: str | None = None, base: str | None = None, timeout: float = 120) -> str:
  """The clip's speech as text ("" if it has no audio or no speech)."""
  key = key or os.environ.get("META_API_KEY")
  if not key:
    raise TranscribeError("META_API_KEY is not set")
  with tempfile.TemporaryDirectory() as directory:
    wav = Path(directory) / "audio.wav"
    if not wav_from_clip(Path(clip), wav):
      return ""
    request = {"model": MODEL, "audioEncoding": "WAV", "mode": "PUSH_TO_TALK"}
    body, content_type = multipart([("request", "", "application/json", json.dumps(request).encode()),
                                    ("audio", "audio.wav", "audio/wav", wav.read_bytes())])
  call = urllib.request.Request(f"{(base or os.environ.get('META_API_BASE') or BASE).rstrip('/')}/asr/transcribe",
                                data=body, method="POST", headers={
                                    "Authorization": f"Bearer {key}", "Content-Type": content_type,
                                    "Accept": "text/plain"})
  try:
    with urllib.request.urlopen(call, timeout=timeout) as response:
      text = response.read().decode("utf-8", "replace")
  except urllib.error.HTTPError as error:
    raise TranscribeError(f"transcription failed: HTTP {error.code}: {error.read()[:300].decode('utf-8', 'replace')}") \
        from error
  except Exception as error:  # pylint: disable=broad-except
    raise TranscribeError(f"transcription failed: {str(error)[:300]}") from error
  return " ".join(line.split(":", 1)[1] if line.startswith("unknown:") else line
                  for line in (part.strip() for part in text.splitlines()) if line).strip()
