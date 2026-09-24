PYTHON ?= python3

.PHONY: build run clean

build:
	$(PYTHON) -m compileall -q src

run:
	sh scripts/run.sh

clean:
	$(PYTHON) -c "import pathlib, shutil; [shutil.rmtree(p, ignore_errors=True) for p in pathlib.Path('.').rglob('__pycache__')]"
