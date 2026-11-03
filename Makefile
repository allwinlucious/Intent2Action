# Everyday entry points; docs/REPRODUCE.md has the full experiment commands.
PYTHON ?= python3
VENV := .venv/bin/python
# Intrinsic Core, public commit this project builds against (CI uses the same one).
INTRINSIC_COMMIT := 895281d62b950ca46d21885906796848692b9638
INTRINSIC_DIR ?= .cache/intrinsic-core

.PHONY: setup vendor test ui-fake tables

setup:  ## .venv with the runtime, tests and evaluation tools, plus the vendored ICON client
	$(PYTHON) -m venv .venv
	.venv/bin/pip install -q -r requirements-dev.txt
	$(MAKE) vendor

vendor:  ## Intrinsic Core's ICON Python client into container/stage/vendor (sparse checkout of the pinned commit)
	@if [ ! -d $(INTRINSIC_DIR)/.git ]; then \
	  git clone -q --filter=blob:none --sparse https://github.com/intrinsic-ai/intrinsic-core $(INTRINSIC_DIR) && \
	  git -C $(INTRINSIC_DIR) sparse-checkout set intrinsic_control/intrinsic/icon; fi
	git -C $(INTRINSIC_DIR) checkout -q $(INTRINSIC_COMMIT)
	INTRINSIC_DIR=$(abspath $(INTRINSIC_DIR)) PYTHON=$(VENV) container/stage_vendor.sh

test:  ## unit tests (no Docker, simulator or GPU)
	$(VENV) -m pytest tests -q

ui-fake:  ## the demo panel with a fake robot and model: http://127.0.0.1:8765
	$(VENV) -m demo_ui.server --fake --port 8765

tables:  ## restore the published data into logs/ and print the main results tables
	$(VENV) tools/results.py restore
	$(VENV) learn/eval_table.py --repeat 3 G0 G1b G1d
	$(VENV) tools/results.py isaac
