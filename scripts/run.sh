#!/usr/bin/env sh
# Keep this wrapper usable from any current working directory.  The Makefile
# is the supported entry point, but this makes direct invocation unsurprising.
set -eu

cd "$(dirname "$0")/.."
export ARM_SIM_LINKS="${ARM_SIM_LINKS:-2}"
exec python3 src/main.py
