"""Version C: Qwen3 8B frozen + a small action head on its last hidden state.

  .venv-ml/bin/python learn/head.py featurize --stage1 logs/learn/stage1_train.jsonl [--stage2 ...] --out runs/feat_x.pt
  .venv-ml/bin/python learn/head.py train --features runs/feat_x.pt [--replay-from runs/feat_s1.pt --replay 0.3] \
      [--init runs/c1] --out runs/c2

Per assistant turn the head sees h = the last-layer hidden state at the end of the generation prompt, and
outputs: the tool (classification, plus FINISH: the frozen model then writes the text), numeric arguments
(regression, squashed into each slot's range with tanh), the motion enum, and an object id by pointing at
the hidden state of the id's last token in the latest list_objects result.
"""

import argparse
import json
import os
import random
import re
import sys
import time

import torch
from torch import nn

import agent
import qwen
import sft

FINISH = "FINISH"
POSE = ["x", "y", "z", "qx", "qy", "qz", "qw"]
SCALARS = ["dx", "dy", "dz", "max_force", "speed", "stiffness", "mass", "factor", "joint", "velocity", "duration_s"]
SLOTS = [f"pose.{k}" for k in POSE] + [f"joints.{i}" for i in range(7)] + SCALARS
# (low, high) of every slot; outputs are squashed into these, so out-of-range values cannot be produced.
RANGE = {**{f"pose.{k}": r for k, r in zip(POSE, [(-1, 1), (-1, 1), (-0.3, 1.2)] + [(-1, 1)] * 4)},
         **{f"joints.{i}": (-3.0, 3.0) for i in range(7)},
         "dx": (-1, 1), "dy": (-1, 1), "dz": (-1, 1), "max_force": (0, 60), "speed": (0, 0.12),
         "stiffness": (0, 5000), "mass": (0, 2), "factor": (0, 1), "joint": (1, 7), "velocity": (-1, 1),
         "duration_s": (0, 3)}
TOOL_SLOTS = {
    "move_to_pose": [f"pose.{k}" for k in POSE], "compute_ik": [f"pose.{k}" for k in POSE],
    "check_collisions": [f"pose.{k}" for k in POSE],
    "move_compliant": [f"pose.{k}" for k in POSE] + ["stiffness", "max_force"],
    "move_to_contact": ["dx", "dy", "dz", "max_force", "speed"],
    "move_joints": [f"joints.{i}" for i in range(7)], "compute_fk": [f"joints.{i}" for i in range(7)],
    "jog": ["joint", "velocity", "duration_s"], "set_speed": ["factor"], "set_payload": ["mass"],
}
POINTER_KEY = {"plan_grasp": "object_id", "attach_object": "id", "detach_object": "id", "remove_object": "id"}
MOTIONS = ["", "ANY", "LINEAR"]
ID = re.compile(r'"id": "(obj_\d+)"')
HIDDEN = 4096
SHARD = 200


def tool_names() -> list[str]:
  return [t["function"]["name"] for t in agent.tool_specs()] + [FINISH]


def encode_call(name: str, arguments: dict) -> dict:
  slots, mask = [0.0] * len(SLOTS), [0.0] * len(SLOTS)
  for slot in TOOL_SLOTS.get(name, []):
    if slot.startswith("pose."):
      value = arguments.get("pose", {}).get(slot[5:], 0.0)
    elif slot.startswith("joints."):
      positions = arguments.get("joints", {}).get("positions", [])
      value = positions[int(slot[7:])] if int(slot[7:]) < len(positions) else 0.0
    else:
      value = arguments.get(slot, 0.0)
    lo, hi = RANGE[slot]
    i = SLOTS.index(slot)
    slots[i], mask[i] = min(max(float(value), lo), hi), 1.0
  motion = MOTIONS.index(arguments.get("motion", "")) if name == "move_to_pose" else -1
  return {"slots": slots, "mask": mask, "motion": motion}


def decode_call(name: str, slots: list[float], motion: int, pointer_id: str | None) -> dict:
  value = dict(zip(SLOTS, slots))
  args = {}
  used = TOOL_SLOTS.get(name, [])
  if any(s.startswith("pose.") for s in used):
    args["pose"] = {k: round(value[f"pose.{k}"], 4) for k in POSE}
  if any(s.startswith("joints.") for s in used):
    args["joints"] = {"positions": [round(value[f"joints.{i}"], 4) for i in range(7)]}
  for s in used:
    if s in SCALARS:
      args[s] = int(round(value[s])) if s == "joint" else round(value[s], 5)
  if name == "move_to_pose" and MOTIONS[motion]:
    args["motion"] = MOTIONS[motion]
  if name in POINTER_KEY and pointer_id:
    args[POINTER_KEY[name]] = pointer_id
  return args


class Head(nn.Module):
  def __init__(self, n_tools: int, width: int = 1024):
    super().__init__()
    self.trunk = nn.Sequential(nn.LayerNorm(HIDDEN), nn.Linear(HIDDEN, width), nn.GELU(), nn.Dropout(0.1),
                               nn.Linear(width, width), nn.GELU())
    self.tool = nn.Linear(width, n_tools)
    self.slots = nn.Linear(width, len(SLOTS))
    self.motion = nn.Linear(width, len(MOTIONS))
    self.query = nn.Linear(width, 256)
    self.key = nn.Sequential(nn.LayerNorm(HIDDEN), nn.Linear(HIDDEN, 256))
    lo, hi = zip(*(RANGE[s] for s in SLOTS))
    self.register_buffer("lo", torch.tensor(lo, dtype=torch.float32))
    self.register_buffer("hi", torch.tensor(hi, dtype=torch.float32))

  def forward(self, h: torch.Tensor, cands: torch.Tensor | None):
    z = self.trunk(h.float())
    slots = self.lo + (self.hi - self.lo) * (torch.tanh(self.slots(z)) + 1) / 2
    pointer = None
    if cands is not None and len(cands):
      pointer = self.key(cands.float()) @ self.query(z).T  # (k, 1)
    return self.tool(z), slots, self.motion(z), pointer


# ---- features -------------------------------------------------------------------------------------------

def _hidden(lm, tokenizer, text: str) -> tuple[torch.Tensor, list[tuple[int, int]]]:
  enc = tokenizer(text, add_special_tokens=False, return_offsets_mapping=True)
  ids = torch.tensor([enc["input_ids"]], device="cuda:0")  # a CPU embedding table moves them itself (qwen.load)
  with torch.no_grad():
    h = lm.model(input_ids=ids).last_hidden_state[0]
  return h, enc["offset_mapping"]


def _candidates(text: str, offsets) -> tuple[list[str], list[int]]:
  """Ids of the latest list_objects result in `text` and the token index of each id's last character."""
  start = text.rfind('{"objects": [')
  if start < 0:
    return [], []
  end = text.find("<|im_end|>", start)
  ids, positions = [], []
  for match in ID.finditer(text, start, end):
    char = match.end(1) - 1
    token = next(i for i, (a, b) in enumerate(offsets) if a <= char < b)
    if match.group(1) not in ids:
      ids.append(match.group(1)); positions.append(token)
  return ids, positions


def featurize(tokenizer, lm, examples, tools) -> list[dict]:
  """One forward pass per trained turn, on exactly the prompt the model sees at that turn: Qwen3's template
  renders an earlier assistant turn without the empty <think></think> block the live prompt has, so hidden
  states read from a later, longer rendering would differ from what the policy sees."""
  names = tool_names()
  out = []
  t0 = time.monotonic()
  for n, (messages, trained) in enumerate(examples):
    for i in trained:
      m = messages[i]
      if m.get("tool_calls"):
        f = m["tool_calls"][0]["function"]
        name, args = f["name"], f["arguments"]
      else:
        name, args = FINISH, {}
      if name not in names:
        continue
      prompt = tokenizer.apply_chat_template(messages[:i], tools=tools, add_generation_prompt=True,
                                             enable_thinking=False, tokenize=False)
      h, offsets = _hidden(lm, tokenizer, prompt)
      cand_ids, cand_pos = _candidates(prompt, offsets)
      target = args.get(POINTER_KEY.get(name, ""), None)
      out.append({"h": h[-1].half().cpu(), "cands": h[cand_pos].half().cpu() if cand_pos else torch.zeros(0, HIDDEN).half(),
                  "cand_ids": cand_ids, "tool": names.index(name), **encode_call(name, args),
                  "pointer": cand_ids.index(target) if target in cand_ids else -1})
    if n % 50 == 0:
      print(f"featurized {n}/{len(examples)} examples, {len(out)} turns, {(time.monotonic() - t0) / 60:.1f} min", flush=True)
  return out


# ---- training -------------------------------------------------------------------------------------------

def train(head: Head, data: list[dict], epochs: int, lr: float) -> None:
  opt = torch.optim.AdamW(head.parameters(), lr=lr, weight_decay=0.01)
  scale = (head.hi - head.lo)
  for epoch in range(epochs):
    random.shuffle(data)
    totals = {"tool": 0.0, "slots": 0.0, "motion": 0.0, "pointer": 0.0}
    head.train()
    for d in data:
      tool, slots, motion, pointer = head(d["h"][None].cuda(), d["cands"].cuda())
      loss = nn.functional.cross_entropy(tool, torch.tensor([d["tool"]], device="cuda"))
      totals["tool"] += loss.item()
      mask = torch.tensor(d["mask"], device="cuda")
      if mask.any():
        err = ((slots[0] - torch.tensor(d["slots"], device="cuda")) / scale) ** 2
        l = (err * mask).sum() / mask.sum() * 20
        loss = loss + l; totals["slots"] += l.item()
      if d["motion"] >= 0:
        l = nn.functional.cross_entropy(motion, torch.tensor([d["motion"]], device="cuda"))
        loss = loss + l; totals["motion"] += l.item()
      if d["pointer"] >= 0 and pointer is not None:
        l = nn.functional.cross_entropy(pointer.T, torch.tensor([d["pointer"]], device="cuda"))
        loss = loss + l; totals["pointer"] += l.item()
      opt.zero_grad(); loss.backward(); opt.step()
    print(f"epoch {epoch}: " + ", ".join(f"{k} {v / len(data):.4f}" for k, v in totals.items()), flush=True)


# ---- policy ---------------------------------------------------------------------------------------------

class Policy:
  def __init__(self, name: str, path: str):
    self.name = name
    self.tokenizer, self.lm = qwen.load()
    self.names = tool_names()
    self.head = Head(len(self.names)).cuda()
    self.head.load_state_dict(torch.load(f"{path}/head.pt"))
    self.head.eval()

  def reset(self, task) -> None:
    pass

  @torch.no_grad()
  def act(self, messages, tools) -> tuple[str, list[dict]]:
    text = self.tokenizer.apply_chat_template(messages, tools=tools, add_generation_prompt=True,
                                              enable_thinking=False, tokenize=False)
    h, offsets = _hidden(self.lm, self.tokenizer, text)
    cand_ids, cand_pos = _candidates(text, offsets)
    tool, slots, motion, pointer = self.head(h[-1][None], h[cand_pos] if cand_pos else None)
    name = self.names[int(tool.argmax())]
    if name == FINISH:
      ids = self.tokenizer(text, add_special_tokens=False, return_tensors="pt")["input_ids"].to("cuda:0")
      out = self.lm.generate(ids, attention_mask=torch.ones_like(ids), max_new_tokens=200, do_sample=False,
                             temperature=None, top_p=None, top_k=None)
      words, _ = qwen.parse(self.tokenizer.decode(out[0, ids.shape[1]:], skip_special_tokens=True))
      return words or "Done.", []
    pointer_id = cand_ids[int(pointer.argmax())] if pointer is not None else None
    return "", [{"name": name, "arguments": decode_call(name, slots[0].tolist(), int(motion.argmax()), pointer_id)}]


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("mode", choices=["featurize", "train"])
  parser.add_argument("--stage1")
  parser.add_argument("--stage2")
  parser.add_argument("--features")
  parser.add_argument("--replay-from")
  parser.add_argument("--replay", type=float, default=0.3)
  parser.add_argument("--init")
  parser.add_argument("--epochs", type=int, default=8)
  parser.add_argument("--lr", type=float, default=3e-4)
  parser.add_argument("--out", required=True)
  args = parser.parse_args()
  if args.mode == "featurize":
    examples = []
    if args.stage1:
      examples += sft.load(args.stage1, sft.stage1_examples)
    if args.stage2:
      examples += sft.load(args.stage2, sft.stage2_examples)
    tokenizer, lm = qwen.load(embed_on_cpu=False)
    shards = []
    for k in range(0, len(examples), SHARD):  # shards, so a stopped run resumes
      path = f"{args.out}.part{k // SHARD:03d}"
      if not os.path.exists(path):
        torch.save(featurize(tokenizer, lm, examples[k:k + SHARD], agent.tool_specs()), path + ".tmp")
        os.replace(path + ".tmp", path)
      shards.append(path)
    torch.save([d for path in shards for d in torch.load(path)], args.out)
    print(f"DONE {args.out}", flush=True)
    return
  data = torch.load(args.features)
  if args.replay_from:
    old = torch.load(args.replay_from)
    data = sft.mix(data, old, args.replay, random.Random(0))
  head = Head(len(tool_names())).cuda()
  if args.init:
    head.load_state_dict(torch.load(f"{args.init}/head.pt"))
  train(head, data, args.epochs, args.lr)
  os.makedirs(args.out, exist_ok=True)
  torch.save(head.state_dict(), f"{args.out}/head.pt")
  print(f"DONE {args.out}: {len(data)} turns", flush=True)


if __name__ == "__main__":
  if len(sys.argv) > 1 and sys.argv[1] == "selftest":
    call = {"pose": {"x": 0.5, "y": 0.1, "z": 0.2, "qx": 0, "qy": 1, "qz": 0, "qw": 0}, "motion": "LINEAR"}
    e = encode_call("move_to_pose", call)
    assert decode_call("move_to_pose", e["slots"], e["motion"], None) == \
        {"pose": {"x": 0.5, "y": 0.1, "z": 0.2, "qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}, "motion": "LINEAR"}
    text = 'x <|im_start|>user\n<tool_response>\n{"objects": [{"id": "obj_12", "a": 1}, {"id": "obj_7"}]}<|im_end|>'
    offsets = [(i, i + 1) for i in range(len(text))]
    assert _candidates(text, offsets) == (["obj_12", "obj_7"], [text.index("obj_12") + 5, text.index("obj_7") + 4])
    print("ok")
  else:
    main()
