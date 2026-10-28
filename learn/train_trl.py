"""Version B with the Glimmer post-training recipe (TRL + PEFT): SFT (QLoRA), DPO and GRPO.

The hyperparameters are the recipe's published TRL settings; only the model differs (Qwen3 8B here, the
target model in the cloud: pass --model). Data come from our records, exported to TRL's formats:
  sft   prompt-completion, conversational, with a per-example `tools` column; loss on the completion only
  dpo   prompt + chosen (the call the stack accepted) / rejected (the wrong call it refused), same situation
  grpo  prompt-only (rendered text) + the wanted call; reward = deterministic tiers, as in the recipe

  .venv-ml/bin/python learn/train_trl.py sft  --stage1 logs/learn/stage1_train.jsonl [--stage2 ...] --out runs/b_sft
  .venv-ml/bin/python learn/train_trl.py dpo  --stage1 logs/learn/stage1_train.jsonl --init runs/b_sft --out runs/b_dpo
  .venv-ml/bin/python learn/train_trl.py grpo --stage1 logs/learn/stage1_train.jsonl --init runs/b_sft --out runs/b_grpo
"""

import argparse
import json
import os
import random

import datasets
import peft
import torch
import transformers
import trl

import agent
import bench
import qwen
import sft

# The recipe's TRL settings (QLoRA SFT, DPO, GRPO).
LORA = dict(r=16, lora_alpha=32, lora_dropout=0.05, task_type="CAUSAL_LM",
            target_modules=["q_proj", "v_proj", "k_proj", "o_proj"])
SFT = dict(learning_rate=2e-4, num_train_epochs=3, per_device_train_batch_size=1, gradient_accumulation_steps=16)
# per-device batch 2 x accumulation 8 in the recipe; 1 x 16 keeps the effective batch of 16 on a 10 GB GPU.
DPO = dict(beta=0.1, learning_rate=5e-7, per_device_train_batch_size=1, gradient_accumulation_steps=16, num_train_epochs=1)
# Group size 8 in the recipe; 4 (inside its 4-16 guidance) fits 4 x 6.5k-token rollouts in 10 GB. KL 0.04 (0.01-0.1).
GRPO = dict(learning_rate=1e-6, per_device_train_batch_size=4, num_generations=4, max_completion_length=256, beta=0.04)
TEMPLATE = {"enable_thinking": False}


def _flat(messages: list[dict]) -> list[dict]:
  """Tool-call arguments as JSON strings: Arrow cannot store differently shaped argument objects in one column, and
  the chat template renders a string argument exactly as it renders the object (tojson)."""
  out = []
  for m in messages:
    if m.get("tool_calls"):
      m = {**m, "tool_calls": [{"type": "function", "function": {"name": c["function"]["name"],
          "arguments": json.dumps(c["function"]["arguments"], ensure_ascii=False)}} for c in m["tool_calls"]]}
    out.append(m)
  return out


def _turns(messages: list[dict], trained: list[int], tools: str) -> list[dict]:
  messages = _flat(messages)
  return [{"prompt": messages[:i], "completion": [messages[i]], "tools": tools, "chat_template_kwargs": TEMPLATE}
          for i in trained]


def sft_rows(stage1: str, stage2: str | None, replay: float, rng: random.Random) -> list[dict]:
  tools = json.dumps(agent.tool_specs())
  rows1 = [r for m, t in sft.load(stage1, sft.stage1_examples) for r in _turns(m, t, tools)]
  if not stage2:
    return rows1
  rows2 = [r for m, t in sft.load(stage2, sft.stage2_examples) for r in _turns(m, t, tools)]
  return sft.mix(rows2, rows1, replay, rng)


def dpo_rows(stage1: str) -> list[dict]:
  tools = json.dumps(agent.tool_specs())
  rows = []
  for line in open(stage1):
    record = json.loads(line)
    valid, wrong = record["valid"], record["wrong"]
    if record["infeasible"] or valid["status"] != "OK" or not wrong or wrong["status"] == "OK":
      continue
    examples = sft.stage1_examples(record)
    messages, trained = examples[0]
    messages = _flat(messages + [sft._assistant(wrong["name"], wrong["arguments"])])
    rows.append({"prompt": messages[:trained[0]], "chosen": [messages[trained[0]]], "rejected": [messages[-1]],
                 "tools": tools, "chat_template_kwargs": TEMPLATE})
  return rows


def grpo_rows(stage1: str, tokenizer) -> list[dict]:
  tools = agent.tool_specs()
  rows = []
  for line in open(stage1):
    for kind, messages, want in bench.cases(json.loads(line)):
      prompt = tokenizer.apply_chat_template(messages, tools=tools, add_generation_prompt=True, tokenize=False, **TEMPLATE)
      rows.append({"prompt": prompt, "want": json.dumps(want), "kind": kind})
  return rows


def reward(completions, want, **_) -> list[float]:
  """Deterministic tiers like the recipe's: 1 right call (or right refusal), 0.3 right tool with wrong
  arguments, 0.1 a well-formed call to a known tool, 0 no call when one was wanted, -0.05 unknown tool or bad JSON."""
  names = {t["function"]["name"] for t in agent.tool_specs()}
  out = []
  for text, w in zip(completions, want):
    w = json.loads(w)
    _, calls = qwen.parse(text if isinstance(text, str) else text[0]["content"])
    if w is None:
      out.append(1.0 if not calls else 0.0)
    elif not calls:
      out.append(0.0)
    elif calls[0].get("parse_error") or calls[0]["name"] not in names:
      out.append(-0.05)
    elif bench.call_ok(calls[0], w):
      out.append(1.0)
    elif calls[0]["name"] == w["name"]:
      out.append(0.3)
    else:
      out.append(0.1)
  return out


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("mode", choices=["sft", "dpo", "grpo"])
  parser.add_argument("--stage1", required=True)
  parser.add_argument("--stage2")
  parser.add_argument("--replay", type=float, default=0.3)
  parser.add_argument("--init")
  parser.add_argument("--model", default=qwen.MODEL)
  parser.add_argument("--limit", type=int, default=0)
  parser.add_argument("--epochs", type=float)
  parser.add_argument("--max-steps", type=int, default=-1)
  parser.add_argument("--accum", type=int, help="override gradient accumulation (smoke tests only)")
  parser.add_argument("--out", required=True)
  args = parser.parse_args()
  rng = random.Random(42)  # the recipe's seed
  tokenizer, model = qwen.load(model=args.model, embed_on_cpu=False)
  model.config.use_cache = False
  if args.init:
    model = peft.PeftModel.from_pretrained(model, args.init, is_trainable=True)
  # Checkpoints every 10 optimizer steps (the supervisor stops training at 17:00 and resumes it at 00:00).
  common = dict(output_dir=args.out, bf16=True, gradient_checkpointing=True, logging_steps=5, save_strategy="steps",
                save_steps=10, save_total_limit=1,
                report_to="none", seed=42, max_steps=args.max_steps,
                gradient_checkpointing_kwargs={"use_reentrant": False})
  if args.mode == "sft":
    rows = sft_rows(args.stage1, args.stage2, args.replay, rng)
    config = trl.SFTConfig(**common, **SFT, max_length=None, use_liger_kernel=True, completion_only_loss=True)
    trainer_class = trl.SFTTrainer
  elif args.mode == "dpo":
    rows = dpo_rows(args.stage1)
    # Reference log-probs in a pass before training: chosen + rejected + reference together exceed 10 GB.
    config = trl.DPOConfig(**common, **DPO, max_length=None, use_liger_kernel=True, precompute_ref_log_probs=True,
                           precompute_ref_batch_size=1)
    trainer_class = trl.DPOTrainer
  else:
    rows = grpo_rows(args.stage1, tokenizer)
    config = trl.GRPOConfig(**common, **GRPO, use_liger_kernel=True)
    trainer_class = trl.GRPOTrainer
  if args.epochs is not None:
    config.num_train_epochs = args.epochs
  if args.accum:
    config.gradient_accumulation_steps = args.accum
  rng.shuffle(rows)
  if args.limit:
    rows = rows[:args.limit]
  print(f"{args.mode}: {len(rows)} rows", flush=True)
  dataset = datasets.Dataset.from_list(rows)
  kwargs = dict(model=model, args=config, train_dataset=dataset, processing_class=tokenizer)
  if not args.init:
    kwargs["peft_config"] = peft.LoraConfig(**LORA)
  if args.mode == "grpo":
    kwargs["reward_funcs"] = reward
  trainer = trainer_class(**kwargs)
  last = transformers.trainer_utils.get_last_checkpoint(args.out) if os.path.isdir(args.out) else None
  print(f"resuming from {last}" if last else "starting", flush=True)
  trainer.train(resume_from_checkpoint=last)
  trainer.model.save_pretrained(args.out)
  json.dump(vars(args) | {"rows": len(rows), "log": trainer.state.log_history[-3:],
                          "peak_gib": torch.cuda.max_memory_allocated() / 2**30},
            open(f"{args.out}/train_args.json", "w"), indent=1)
  print(f"DONE {args.out}; peak {torch.cuda.max_memory_allocated() / 2**30:.1f} GiB", flush=True)


if __name__ == "__main__":
  main()
