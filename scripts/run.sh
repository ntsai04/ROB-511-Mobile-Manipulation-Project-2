#!/usr/bin/env sh
set -eu

export ARM_SIM_LINKS="${ARM_SIM_LINKS:-2}"
exec python3 src/main.py
