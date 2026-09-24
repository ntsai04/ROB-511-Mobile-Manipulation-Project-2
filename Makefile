.PHONY: build run clean
build:
	python3 -m compileall -q src
run:
	sh scripts/run.sh
clean:
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
