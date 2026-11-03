# Everyday entry points; docs/REPRODUCE.md has the full experiment commands.
PYTHON ?= python3
VENV := .venv/bin/python

.PHONY: setup test ui-fake tables

setup:  ## .venv with the runtime, tests and evaluation tools
	$(PYTHON) -m venv .venv
	.venv/bin/pip install -q -r requirements-dev.txt

test:  ## unit tests (no Docker, simulator or GPU)
	$(VENV) -m pytest tests -q

ui-fake:  ## the demo panel with a fake robot and model: http://127.0.0.1:8765
	$(VENV) -m demo_ui.server --fake --port 8765

tables:  ## restore the published data into logs/ and print the main results tables
	$(VENV) tools/results.py restore
	$(VENV) learn/eval_table.py --repeat 3 G0 G1b G1d
	$(VENV) tools/results.py isaac
