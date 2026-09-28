# Mutation-test scaffold

`test_mutation_suite.py` contains exactly 25 standard-library `unittest`
methods. Every method has one named behavior target and makes no console,
logging, network, or filesystem output. That gives a mutation runner a clean
per-test pass/fail signal and leaves room to replace one test without exceeding
the class's 25-test limit.

Run the whole suite silently (its only result is the process exit code):

```sh
PYTHONPATH=src python3 tests/run_mutation_tests.py
```

Run one diagnostic test silently by its full ID:

```sh
PYTHONPATH=src python3 tests/run_mutation_tests.py \
  tests.test_mutation_suite.MutationTests.test_04_rk4_integrates_time_varying_forcing
```

The tests are organized as a deliberately balanced mutation budget:

| Area | Tests |
| --- | --- |
| Integrators, expressions, and checkpoint service | 01–08 |
| PID and simulation-control contract | 09–18 |
| Lagrangian dynamics | 19–23 |
| Closed loop and inverse kinematics | 24–25 |

When the course distributes its official test API, retain the focused test
intent and replace only the direct node calls with that API's setup and
assertion helpers. Do not combine several numbered checks into one broad
scenario: that would make a failed mutant harder to localize.
