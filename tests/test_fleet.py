import json

from learn import fleet
from learn.episodes import in_shard


def _write(path, values):
  path.write_text("".join(json.dumps(v) + "\n" for v in values))


def test_split_then_merge_keeps_one_record_per_task(tmp_path):
  out = tmp_path / "train.jsonl"
  _write(out, [{"family": "cluster", "seed": s, "v": "old"} for s in range(10)])
  shards = fleet.split(out, 3)
  for k, path in enumerate(shards):
    seeds = [json.loads(line)["seed"] for line in path.read_text().splitlines()]
    assert all(in_shard("cluster", s, f"{k}/3") for s in seeds)
  assert sorted(json.loads(line)["seed"] for p in shards for line in p.read_text().splitlines()) == list(range(10))
  # A worker appends a new task and rewrites nothing: merge has 11 tasks, shards removed.
  with shards[0].open("a") as handle:
    handle.write(json.dumps({"family": "cluster", "seed": 10, "v": "new"}) + "\n")
  assert fleet.merge(out, shards) == 11
  assert not any(p.exists() for p in shards)
  assert len(out.read_text().splitlines()) == 11


def test_samples_are_keyed_by_sample_index(tmp_path):
  out = tmp_path / "samples.jsonl"
  _write(out, [{"family": "reach", "seed": 1, "sample": i} for i in range(4)])
  assert fleet.merge(out, fleet.split(out, 2)) == 4


def test_shards_partition_tasks():
  owners = [sum(in_shard("packing", s, f"{k}/4") for k in range(4)) for s in range(200)]
  assert set(owners) == {1}
