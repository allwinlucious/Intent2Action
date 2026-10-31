import json
import sys
import urllib.error

import pytest

from learn import families, glimmer_sample
from learn.episodes import Episode

OBJECTS = [{"id": "obj_12", "category": "cube", "color": "red", "size": "medium", "x": 0.5, "y": 0.1, "z": 0.0,
            "yaw": 0.0, "sx": 0.05, "sy": 0.05, "sz": 0.05, "attached": False}]
PROGRAM = "verify(lambda: True)\n"


def solve_record(seed, objects=True):
  task = families.generate("cluster", seed)
  value = {"family": task.family, "seed": seed, "split": task.split, "intent": task.intent, "expert_program": None,
           "objects": OBJECTS, "episodes": []}
  if not objects:
    del value["objects"]
  return value


@pytest.fixture
def endpoint(monkeypatch):
  """Request bodies sent to the fake endpoint; set `fail` to make every request a URLError."""
  state = {"bodies": [], "fail": False}

  class Response:
    def __enter__(self):
      return self

    def __exit__(self, *args):
      pass

    def read(self):
      return json.dumps({"choices": [{"message": {"content": f"```python\n{PROGRAM}```"}}]}).encode()

  def open_request(request, timeout):
    del timeout
    if state["fail"]:
      raise urllib.error.URLError("down")
    state["bodies"].append(json.loads(request.data))
    return Response()

  monkeypatch.setattr("urllib.request.urlopen", open_request)
  for name, value in (("LLM_BASE_URL", "http://model"), ("LLM_API_KEY", "secret"), ("LLM_MODEL", "glimmer")):
    monkeypatch.setenv(name, value)
  monkeypatch.delenv("LLM_EXTRA_BODY", raising=False)
  return state


def generate(monkeypatch, solve, out):
  monkeypatch.setattr(sys, "argv", ["glimmer_sample.py", "generate", "--split", "train", "--n", "3", "--k", "3",
                                    "--families", "cluster", "--solve", str(solve), "--out", str(out)])
  glimmer_sample.main()


def test_generate_k_samples_resume_and_temperature(endpoint, monkeypatch, tmp_path):
  solve = tmp_path / "train.jsonl"
  solve.write_text("".join(json.dumps(solve_record(seed, objects=seed != 2)) + "\n" for seed in range(3)))
  out = tmp_path / "programs.jsonl"
  endpoint["fail"] = True
  generate(monkeypatch, solve, out)
  errors = [json.loads(line) for line in out.read_text().splitlines()]
  assert len(errors) == 6 and all(set(value) == {"family", "seed", "split", "sample", "error"} for value in errors)

  endpoint["fail"] = False
  generate(monkeypatch, solve, out)  # the failed requests are retried
  values = [value for value in map(json.loads, out.read_text().splitlines()) if "program" in value]
  assert sorted((value["seed"], value["sample"]) for value in values) == [(s, i) for s in (0, 1) for i in range(3)]
  assert all(value["program"] == PROGRAM and value["attempts"] == 1 and value["split"] == "train" for value in values)
  assert len(endpoint["bodies"]) == 6 and all(body["temperature"] == 0.7 for body in endpoint["bodies"])
  assert "Scene:\nobj_12 cube red medium" in endpoint["bodies"][0]["messages"][1]["content"]

  generate(monkeypatch, solve, out)  # nothing left to do
  assert len(endpoint["bodies"]) == 6 and len(out.read_text().splitlines()) == 12


def test_extra_body_temperature_wins(endpoint, monkeypatch):
  monkeypatch.setenv("LLM_EXTRA_BODY", '{"temperature": 0.2}')
  glimmer_sample.ChatClient(temperature=0.7).chat([{"role": "user", "content": "hi"}])
  assert endpoint["bodies"][0]["temperature"] == 0.2


def test_run_one_record_per_sample_and_resume(monkeypatch, tmp_path, capsys):
  ran = []

  def run_episode(task, program, sim_stub, robot_stub, agent=None, strategy=None):
    assert agent is None and strategy is None
    ran.append((task.seed, program))
    return Episode(task.family, task.seed, task.split, strategy, program, True, True, True, "", {}, None, None, [], 2,
                   0.5, [{"kind": "done"}], OBJECTS)

  monkeypatch.setattr(glimmer_sample, "run_episode", run_episode)
  samples = [{"family": "cluster", "seed": 0, "split": "train", "sample": 0, "program": PROGRAM},
             {"family": "cluster", "seed": 0, "split": "train", "sample": 1, "error": "URLError: down"},
             {"family": "cluster", "seed": 1, "split": "train", "sample": 1, "program": PROGRAM}]
  out = tmp_path / "episodes.jsonl"
  glimmer_sample.run(samples, out, None, None)
  values = [json.loads(line) for line in out.read_text().splitlines()]
  assert ran == [(0, PROGRAM), (1, PROGRAM)]
  assert [(value["seed"], value["sample"]) for value in values] == [(0, 0), (1, 1)]
  assert values[1]["intent"] == families.generate("cluster", 1).intent and "events" not in values[0]
  assert "cluster: samples=2 ok=100%" in capsys.readouterr().out
  glimmer_sample.run(samples, out, None, None)
  assert len(ran) == 2 and len(out.read_text().splitlines()) == 2
