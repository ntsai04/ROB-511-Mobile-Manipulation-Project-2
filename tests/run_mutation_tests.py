"""Run mutation tests silently, optionally selecting individual test IDs.

Examples (both intentionally produce no console output):
  PYTHONPATH=src python3 tests/run_mutation_tests.py
  PYTHONPATH=src python3 tests/run_mutation_tests.py \\
      tests.test_mutation_suite.MutationTests.test_04_rk4_integrates_time_varying_forcing
"""

from __future__ import annotations

import io
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main(arguments: list[str]) -> int:
    loader = unittest.defaultTestLoader
    suite = loader.loadTestsFromNames(arguments) if arguments else loader.discover(
        str(ROOT / "tests"), pattern="test_*.py", top_level_dir=str(ROOT)
    )
    # A mutation platform should inspect each test's pass/fail result itself.
    # Capturing the runner's normal status text keeps this local runner quiet.
    result = unittest.TextTestRunner(stream=io.StringIO(), verbosity=0).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
