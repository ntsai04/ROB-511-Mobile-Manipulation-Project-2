PYTHON ?= python3
VIS_PORT ?= 8002

.PHONY: build run test visualizer clean

build:
	$(PYTHON) -m compileall -q src

run:
	sh scripts/run.sh

test:
	@$(PYTHON) tests/run_tests.py $(TESTS)

visualizer:
	@$(PYTHON) debug_visualizer/server.py --port $(VIS_PORT)

clean:
	$(PYTHON) -c "import pathlib, shutil; [shutil.rmtree(p, ignore_errors=True) for p in pathlib.Path('.').rglob('__pycache__')]"
