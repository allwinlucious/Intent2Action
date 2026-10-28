"""QLoRA fine-tune of Qwen3 8B on sft.py examples (version B).

  .venv-ml/bin/python learn/train_lora.py --stage1 logs/learn/stage1_train.jsonl --out runs/b1
  .venv-ml/bin/python learn/train_lora.py --stage1 ... --stage2 logs/learn/expert_train.jsonl \
      --init runs/b1 --replay 0.3 --out runs/b2

4-bit base, LoRA r 16 on attention + MLP, batch 1 with gradient accumulation, gradient checkpointing. The
loss is computed only at labelled positions (the lm_head over 9k positions x 152k vocab would not fit).
"""

import argparse
import json
import random
import time

import peft
import torch

import agent
import qwen
import sft


def loss_of(model, ids: torch.Tensor, labels: torch.Tensor) -> torch.Tensor:
  base = model.get_base_model()
  hidden = base.model(input_ids=ids).last_hidden_state[0, :-1]
  target = labels[0, 1:]
  keep = target != sft.IGNORE
  logits = base.lm_head(hidden[keep]).float()
  return torch.nn.functional.cross_entropy(logits, target[keep])


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--stage1", required=True)
  parser.add_argument("--stage2")
  parser.add_argument("--init")
  parser.add_argument("--replay", type=float, default=0.3)
  parser.add_argument("--limit", type=int, default=0)
  parser.add_argument("--epochs", type=int, default=1)
  parser.add_argument("--lr", type=float, default=2e-4)
  parser.add_argument("--accum", type=int, default=8)
  parser.add_argument("--out", required=True)
  args = parser.parse_args()
  rng = random.Random(0)
  stage1 = sft.load(args.stage1, sft.stage1_examples)
  if args.stage2:
    data = sft.mix(sft.load(args.stage2, sft.stage2_examples), stage1, args.replay, rng)
  else:
    data = stage1
  rng.shuffle(data)
  if args.limit:
    data = data[:args.limit]
  tokenizer, model = qwen.load()
  model = peft.prepare_model_for_kbit_training(model, use_gradient_checkpointing=True)
  if args.init:
    model = peft.PeftModel.from_pretrained(model, args.init, is_trainable=True)
  else:
    model = peft.get_peft_model(model, peft.LoraConfig(
        r=16, lora_alpha=32, lora_dropout=0.05, task_type="CAUSAL_LM",
        target_modules=["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"]))
  model.print_trainable_parameters()
  tools = agent.tool_specs()
  encoded = [sft.tokenize(tokenizer, m, t, tools) for m, t in data]
  print(f"{len(encoded)} examples, mean {sum(len(i) for i, _ in encoded) / len(encoded):.0f} tokens, "
        f"{sum(sum(l != sft.IGNORE for l in labels) for _, labels in encoded)} trained tokens", flush=True)
  params = [p for p in model.parameters() if p.requires_grad]
  optim = torch.optim.AdamW(params, lr=args.lr, weight_decay=0.0)
  steps = args.epochs * len(encoded) // args.accum
  sched = torch.optim.lr_scheduler.LambdaLR(optim, lambda s: min(1.0, (s + 1) / 10) * max(0.0, 1 - s / max(steps, 1)))
  model.train()
  start, step, running = time.monotonic(), 0, []
  for epoch in range(args.epochs):
    for n, (ids, labels) in enumerate(encoded, 1):
      ids_t = torch.tensor([ids], device="cuda")
      labels_t = torch.tensor([labels], device="cuda")
      loss = loss_of(model, ids_t, labels_t) / args.accum
      loss.backward()
      running.append(loss.item() * args.accum)
      if n % args.accum == 0:
        torch.nn.utils.clip_grad_norm_(params, 1.0)
        optim.step(); sched.step(); optim.zero_grad(set_to_none=True)
        step += 1
        if step % 5 == 0:
          print(f"epoch {epoch} step {step}/{steps} loss {sum(running) / len(running):.4f} "
                f"{(time.monotonic() - start) / 60:.1f} min, peak {torch.cuda.max_memory_allocated() / 2**30:.1f} GiB", flush=True)
          running = []
  model.save_pretrained(args.out)
  json.dump(vars(args) | {"examples": len(encoded), "minutes": (time.monotonic() - start) / 60},
            open(f"{args.out}/train_args.json", "w"))
  print(f"DONE saved {args.out} in {(time.monotonic() - start) / 60:.0f} min", flush=True)


if __name__ == "__main__":
  main()
