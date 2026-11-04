"""Runs the demo scenes through the real pipeline once and saves what the models produced, for recording later.

  LLM_BASE_URL=http://127.0.0.1:18800/v1 LLM_API_KEY=unused .venv/bin/python tools/demo_programs.py [scene ...]

For each scene (a demo preset id or `family:seed`): the scene's request text -> rules (reader, base Glimmer) ->
program (writer, the fine-tuned model), and, for the comparison scenes, the program the untrained model writes for
the same rules. Writes logs/demo_programs/<scene>.json (request, seen, intent, programs, attempts, seconds) and one
<scene>.<model>.py per program, for tools/record_take.py --program.
"""

import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "learn")]

from program_runtime.agent import ChatClient, ProgramWriter  # noqa: E402
from program_runtime.reader import IntentReader  # noqa: E402

OUT = ROOT / "logs/demo_programs"
SCENES = ("cubes_in_box:0", "sort_by_color:0", "yellow_on_red:0", "heart_box:0", "stacking:100007", "mixed:100002",
          "cluster:100001")
COMPARE = {"stacking:100007", "mixed:100002"}  # the untrained model's program too (side-by-side shots)
READER, WRITER, UNTRAINED = "G0", "G1d", "G0"


def scene(scene_id: str):
  import families
  from demo_ui.scenes import BY_ID, fake_objects
  if scene_id in BY_ID:
    preset = BY_ID[scene_id]
    return preset.request, fake_objects(preset)
  family, seed = scene_id.split(":")
  task = families.generate(family, int(seed))
  return task.intent["description"], families.fake_objects(task)


def main(scene_ids) -> None:
  OUT.mkdir(parents=True, exist_ok=True)
  client = ChatClient()
  for scene_id in scene_ids:
    request, objects = scene(scene_id)
    record = {"scene": scene_id, "request": request}
    start = time.monotonic()
    reading = IntentReader(client, model=READER).read_text(request)
    record.update(seen=reading.seen, intent=reading.intent, read_seconds=round(time.monotonic() - start, 1))
    models = [WRITER] + ([UNTRAINED] if scene_id in COMPARE else [])
    record["programs"] = {}
    for model in models:
      writer = ProgramWriter(client, model=model)
      start = time.monotonic()
      try:
        program = writer.write(reading.intent, objects)
        record["programs"][model] = {"program": program, "attempts": writer.last_attempts,
                                     "seconds": round(time.monotonic() - start, 1)}
        (OUT / f"{scene_id.replace(':', '_')}.{model}.py").write_text(program)
      except Exception as error:  # pylint: disable=broad-except
        record["programs"][model] = {"error": str(error)[:500]}
    (OUT / f"{scene_id.replace(':', '_')}.json").write_text(json.dumps(record, indent=1))
    print(f"== {scene_id}: {request!r}\n   rules: {json.dumps(reading.intent['rules'])}")
    for model, value in record["programs"].items():
      print(f"   {model}: {'ERROR ' + value['error'][:120] if 'error' in value else str(value['attempts']) + ' attempt(s), ' + str(value['seconds']) + ' s'}")


if __name__ == "__main__":
  main(sys.argv[1:] or SCENES)
