PYTHON ?= python3

.PHONY: build run test visualizer submission clean

build:
	$(PYTHON) -m compileall -q src

run:
	$(PYTHON) src/main.py

test:
	@$(PYTHON) tests/run_tests.py $(TESTS)

visualizer:
	$(PYTHON) visualizer/server.py

submission:
	$(PYTHON) tools/package_submission.py

clean:
	$(PYTHON) -c "import pathlib, shutil; [shutil.rmtree(p, ignore_errors=True) for p in pathlib.Path('.').rglob('__pycache__')]"
