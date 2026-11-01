"""DPO on top of an SFT adapter (runs on the GPU host; see runpod/). Policy = the SFT adapter, trainable; reference =
the same adapter frozen (a second NF4 copy of base + adapter), both on the cookbook's NF4 base, as for the QLoRA SFT.

  python dpo_train.py --root /workspace/train --data dpo --init G1b --run G5 [--beta 0.1 --lr 5e-6 --epochs 1]

Data: <root>/data/<data>/{train,validation}.jsonl from learn/dpo_data.py render (prompt/chosen/rejected text).
Writes the adapter to <root>/runs/<run> and <root>/runs/<run>.done.json.
"""

import argparse
import json
from pathlib import Path

MODEL = "meta-models/Muse-Glimmer-30B"


def strip_bos(text: str, bos: str) -> str:
  """The rendered prompt starts with the BOS token; the trainer's tokenizer adds its own."""
  return text[len(bos):] if bos and text.startswith(bos) else text


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--root", default="/workspace/train")
  parser.add_argument("--data", required=True)
  parser.add_argument("--init", required=True, help="SFT adapter run to start from (policy and reference)")
  parser.add_argument("--run", required=True)
  parser.add_argument("--beta", type=float, default=0.1)
  parser.add_argument("--lr", type=float, default=5e-6)
  parser.add_argument("--epochs", type=float, default=1.0)
  parser.add_argument("--max-length", type=int, default=4096)
  args = parser.parse_args()

  import datasets
  import peft
  import torch
  import transformers
  import trl

  out = Path(f"{args.root}/runs/{args.run}")
  status = {"run": args.run}
  try:
    tokenizer = transformers.AutoProcessor.from_pretrained(MODEL).tokenizer
    bos = tokenizer.bos_token or ""
    data = {}
    for split in ("train", "validation"):
      rows = [json.loads(line) for line in Path(f"{args.root}/data/{args.data}/{split}.jsonl").read_text().splitlines()]
      data[split] = datasets.Dataset.from_list([{**row, "prompt": strip_bos(row["prompt"], bos)} for row in rows])
    quantization = transformers.BitsAndBytesConfig(
        load_in_4bit=True, bnb_4bit_quant_type="nf4", bnb_4bit_use_double_quant=True,
        bnb_4bit_compute_dtype=torch.bfloat16, bnb_4bit_quant_storage=torch.bfloat16)
    init = Path(f"{args.root}/runs/{args.init}")

    def load(trainable: bool):
      base = transformers.AutoModelForImageTextToText.from_pretrained(
          MODEL, device_map="cuda", dtype=torch.bfloat16, quantization_config=quantization, attn_implementation="sdpa")
      if trainable:
        base = peft.prepare_model_for_kbit_training(base, use_gradient_checkpointing=True)
      return peft.PeftModel.from_pretrained(base, str(init), is_trainable=trainable)

    # Reference = the SFT adapter, frozen, as a second NF4 copy: this TRL has no adapter-name options, and without an
    # explicit ref_model it would compare against the base model with the adapter disabled.
    model, reference = load(True), load(False).eval()
    config = trl.DPOConfig(
        output_dir=str(out), beta=args.beta, learning_rate=args.lr, num_train_epochs=args.epochs,
        per_device_train_batch_size=1, gradient_accumulation_steps=8, lr_scheduler_type="cosine",
        warmup_steps=0.1, bf16=True, gradient_checkpointing=True, logging_steps=1, eval_strategy="epoch",
        save_strategy="no", max_length=args.max_length, report_to=[], seed=42)
    trainer = trl.DPOTrainer(model=model, ref_model=reference, args=config, train_dataset=data["train"],
                             eval_dataset=data["validation"], processing_class=tokenizer)
    result = trainer.train()
    metrics = trainer.evaluate()
    model.save_pretrained(str(out))
    status.update(ok=True, train=result.metrics, eval=metrics, log=trainer.state.log_history[-40:])
  except Exception as error:  # pylint: disable=broad-except
    import traceback
    status.update(ok=False, error=traceback.format_exc()[-3000:])
  out.parent.mkdir(parents=True, exist_ok=True)
  Path(f"{args.root}/runs/{args.run}.done.json").write_text(json.dumps(status, indent=1, default=str))
  print(json.dumps({k: v for k, v in status.items() if k != "log"}, default=str)[:3000])


if __name__ == "__main__":
  main()
