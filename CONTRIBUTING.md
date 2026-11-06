# Contributing to USB3-PIPE

## Tests

```sh
$ pytest test                                   # Default suite (pytest.ini: -m "not slow"), Verilator needed.
$ pytest test -n auto -m "not slow and not sim" # Migen tests only, in parallel (pytest-xdist).
$ pytest test -m "not slow and sim"             # Verilator simulations (sim.py builds).
$ pytest test -m slow                           # Long simulations (nightly).
$ pytest test/test_core_bulk.py                 # One file.
```

Markers (`pytest.ini`): `slow` (long simulations, not run by default), `sim` (Verilator simulations
built from `sim.py`). The CI (`.github/workflows/ci.yml`: Python 3.9, LiteX `standard` install,
Verilator at the commit pinned by the LiteX CI) runs `-m "not slow and not sim"` then
`-m "not slow and sim"` on every push and pull request, and `-m slow` nightly. Keep the default suite
fast: long simulations go behind the `slow` marker (a few minutes of soak at most). The targets
elaboration test (`test/test_targets.py`) covers the Xilinx targets; the Tang Mega 138K Pro target is
built manually (Gowin IDE).

## Coding style

LiteX coding style: https://github.com/enjoy-digital/litex/blob/master/doc/coding_style.md (100
columns, aligned assignments and keyword arguments, `# # #` separator between the interface and the
logic of a module, header comment with copyright/SPDX, module docstrings with the USB 3.2 section
numbers, `Reduce` from `litex.gen`). Match the surrounding file; do not reformat unrelated code, do
not rename options or CSRs.

## Commits and pull requests

- Commit titles `module: Description of the change.`, one change per commit, a body saying what and
  why, and what was tested (simulation, hardware and on which board).
- One topic per pull request, against the default development branch; dependent pull requests as a
  linear stack. Say if an agent wrote the code and who supervised it.
- Never edit generated files (`build/`, `csr.h`, `csr.csv`, the Verilog, `gateware/analyzer.csv`):
  change the Python.
- Documentation goes in the same pull request: an option added to `usb3_soc_args`
  (`bench/usb3_soc.py`), to a target or to `sim.py` is documented in `doc/examples.md`/`doc/targets.md`
  or `doc/usb3-pipe.md` ("Simulation"); a new debug CSR or flow in `doc/debugging-guide.md`; new
  figures or milestones in the README status table and `doc/gen2-x2-roadmap.md`.

## Hardware

- `--load` (volatile) is the default for experiments: never flash a board or write its SPI flash
  unless asked.
- Never retrain a link from the debug CSRs while the device is in U3 (suspended): the host still sees
  its port in U3 and the device ends in Compliance (`doc/debugging-guide.md`, "Suspend/Resume").
- Read `doc/debugging-guide.md` before hardware work: registers before traces, narrow LiteScope
  captures, reproduce every finding in simulation (a Migen test or `sim.py`).
- Slow fabrics (Gowin GW5): Fmax is the limiting factor, keep new logic registered and check timing
  on the GW5 target, not only on Xilinx. Gowin synthesis drops the init contents of memories with an
  asynchronous read port: use logic lookups or synchronous reads.
