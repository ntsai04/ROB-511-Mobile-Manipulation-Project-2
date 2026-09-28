PYTHON ?= python3

.PHONY: build run test clean

build:
	$(PYTHON) -m compileall -q src

run:
	sh scripts/run.sh

test:
	PYTHONPATH=src $(PYTHON) tests/run_mutation_tests.py

clean:
	$(PYTHON) -c "import pathlib, shutil; [shutil.rmtree(p, ignore_errors=True) for p in pathlib.Path('.').rglob('__pycache__')]"
