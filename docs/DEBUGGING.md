# Run and debug Project 2

From the project root in the practice container:

```sh
make build
ARM_SIM_LINKS=2 make run
# In another terminal:
make test
make test TESTS=test_pendularm.py::test_04_rk4_matches_polynomial_antiderivatives
python3 tests/run_tests.py --list
# In a third terminal:
make visualizer
```

Open `http://127.0.0.1:8002` in a local browser (or forward port 8002 through
VS Code's Ports tab). The runtime must already be running on TCP 9095. Stop
the first runtime and repeat with `ARM_SIM_LINKS=3 make run` and `make test`.
Do not run multiple project runtimes together: they share 9095. Stop `make run`
before using `autograde`.

Close the visualizer or turn off Live sampling before running tests. Tests
deliberately pause/reset/update the arm, and changing controls concurrently can
interfere with their isolated state.

`make test` uses the course-supplied runner unchanged: exactly 25 separate
processes, a 30-second limit each, file/function selection via `TESTS`, and an
overall nonzero exit status on failure. Individual tests are silent even on
failure; the runner alone reports pass/fail. Tests use Python's standard library
and the supplied TCP client. No packages or network installation are required.
Use [MUTATION_TEST_NOTES.md](MUTATION_TEST_NOTES.md) as the exam note sheet.

The Live arm view renders measured `/joint_states` positions with current link
lengths, displays velocity/effort, and plots measurements against simulation
time. Pause/resume/reset, physical parameter and integrator controls call their
runtime services. Click the drawing to set x/y; Solve only overlays the actual
IK response, and Send IK goal commands the action service. Enable PID and resume
the simulation to move. For 3 links set the desired end-effector orientation φ.
The joint setpoint controls publish a real `/joint_trajectory` message.

The Integrator bench sends the same standalone particle problem to all four
runtime integrators. Select position or velocity and inspect final values and
raw replies. With expression `t`, the white dashed curve is the closed-form
reference. The benchmark service does not move the live arm. Rejected requests
and timeouts appear explicitly in the Runtime response panel.

The visualizer is separate debugging support and does not import the project's
source or provide substitute implementations. Its HTTP server bridges browser
requests to the course's newline-delimited TCP protocol. There are no CDN assets
or third-party Python/JavaScript dependencies. On a host without `make`, use
`python src/main.py`, `python tests/run_tests.py` (the course runner targets
Linux), and `python debug_visualizer/server.py --port 8002`. To override the
HTTP port, use `make visualizer VIS_PORT=8102`. Additional server options are
listed by `python3 debug_visualizer/server.py --help`.

The repository's existing `make_submission.sh` is the original kit packaging
script and expects a `starter/<language>` tree. Use the workspace packaging
helper documented in the workspace guide to package this repository root,
including its new `tests/`, `debug_visualizer/`, and Makefile.
