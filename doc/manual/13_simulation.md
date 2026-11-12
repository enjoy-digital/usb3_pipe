# Simulation and verification {#simulation}

This chapter describes how the project is verified before any bitstream is built: the host models,
the unit and system tests, the full-system Verilator simulation and the continuous integration. It
is for contributors who change the gateware and for integrators who want to reproduce a link
problem without hardware.

## Verification strategy {#sim-strategy}

A USB 3 link fails in ways that are slow to diagnose on a board: a link that never reaches U0, a
host that gives up after a few retries, a packet lost once per gigabyte. The project therefore
checks every layer in simulation first, each at the cheapest level that can exercise it.

| Level | What runs | Partner | Speed |
|--------------------|-----------------------------------|-----------------------------------|----------------|
| Unit | One module (scrambler, aligner, LFPS) | Python stimulus and reference model | Seconds |
| LTSSM | The link state machine alone | Python driver of its inputs | Seconds |
| Device core | Link and protocol layers, classes | Reactive Python host, PHY stand-in | Seconds to minutes |
| System | Two PHYs and the device core | Gateware host model (`sim.py`) | Minutes |
| Hardware | Bench target on a board | Linux xHCI host | Minutes |

A finding on hardware is turned into a test at the lowest level that reproduces it. The
[Debugging](#debugging) chapter ends with this loop: observe on hardware, reproduce in simulation,
fix, then check again on the board.

## Host models {#sim-host-models}

The device must be tested against a host, but no USB 3 host exists as gateware. The project provides
two host models of the link and protocol layers, written from the specification and checked against
packets captured on hardware with a Linux xHCI host (`test/test_host_model.py`).

| Model | Form | Used by | File |
|------------------|--------------------------|----------------------|------------------------------------------------|
| Scripted host | Gateware playing a script | `sim.py`, system tests | `usb3_core/host_model.py` |
| Reactive host | Pure Python generators | Device core unit tests | `test/model/host.py` |

The scripted host (`USB3HostModel`) plays a list of steps: send words, wait for received words,
request SKP Ordered Sets. A script builder (`HostScript`) tracks the header sequence numbers and the
credits, and `enumeration_script` builds the sequence a Linux host plays: link initialization with
the credit advertisements, port capability and configuration LMPs, then the enumeration control
transfers and Bulk transfers.

The reactive host (`PyHost`) behaves like an xHCI host: it acknowledges each header packet with
LGOOD and LCRD, and the test scenarios are written as Python generators. Helpers cover the control
transfers, Bulk IN with its ACK and ERDY handshake, Isochronous service intervals and the Mass
Storage command blocks. It talks to the device core through a PHY stand-in (`PHYStandIn`) that
replaces the PHY and its link interface, as the figure below shows.

![Device core unit tests: the Python host drives the link interface of the device core through a
PHY stand-in, without PHY or transceiver.](sim_core_test.svg)

The other models live in `test/model/`:

| File | Content |
|------------------|------------------------------------------------------------------------|
| `gt.py` | Transceiver model with the `USB3SerDes` interface, LFPS face of the transceiver |
| `ltssm.py` | Drivers of the LTSSM inputs, two LTSSMs back to back |
| `device.py` | PHY stand-in in word pairs and quads, Bulk loopback design, enumeration checks |
| `gen2.py` | Bit-serial reference of the Gen 2 encoding, scrambling and SKP rules, channel model |

The Gen 2 reference model is written register by register from the scrambler figure of the
specification, independently of the gateware. A channel model on top of it adds bit offsets,
polarity inversion, bit slips and SKP edits, so the block aligner is tested against the rules and
not against itself.

## Unit and system tests {#sim-tests}

The tests are Python `unittest` classes run by pytest, in `test/`. Most simulate the gateware with
the Migen simulator; the system tests build `sim.py` with Verilator. The table below groups the
files by area.

| Area | Test files |
|----------------------------|--------------------------------------------------------------------|
| PHY datapath | `test_datapath`, `test_datapath_random`, `test_rx_datapath`, `test_pipe`, `test_pipe_rx`, `test_pipe_tx`, `test_pipe_framing`, `test_skp`, `test_tx_skp`, `test_scrambling`, `test_disable_scrambling`, `test_8b10b_errors`, `test_async_fifo`, `test_rx_idle_filter` |
| LFPS | `test_lfps`, `test_lfps_clk`, `test_lfps_coverage`, `test_lfps_ping`, `test_lfps_polling_rx`, `test_lfps_reset`, `test_lfps_tx_waveform` |
| Training and LTSSM | `test_training`, `test_training_coverage`, `test_ts_checker`, `test_ts2_link_config`, `test_ltssm`, `test_ltssm_states`, `test_ltssm_exit`, `test_ltssm_ts_exit`, `test_ltssm_tx_idle`, `test_ltssm_link`, `test_ltssm_pm`, `test_idle_handshake`, `test_phy_rx_reset`, `test_phy_pipe_cd`, `test_debug` |
| Gen 2 and lanes | `test_coding`, `test_framing`, `test_lfps_ssp`, `test_ltssm_ssp`, `test_core_link_ssp`, `test_lanes` |
| Link and protocol | `test_core_crc`, `test_core_device`, `test_core_bulk`, `test_core_bulk_wide`, `test_core_endpoint_converters`, `test_core_link_wide`, `test_core_recovery`, `test_core_lpm`, `test_core_slots`, `test_core_control`, `test_core_iso`, `test_host_model` |
| Classes and examples | `test_core_cdc_acm`, `test_core_cdc_ecm`, `test_core_uvc`, `test_core_msc`, `test_core_msc_luns`, `test_core_class_wide`, `test_core_msc_dram`, `test_core_wishbone`, `test_core_bridge`, `test_bench` |
| Transceivers and targets | `test_serdes`, `test_ecp5`, `test_gw5`, `test_gw5_config`, `test_targets`, `test_gen` |
| System (Verilator) | `test_phy`, `test_sim_host`, `test_phy_long` |

Two families deserve a note. The coverage tests (`test_lfps_coverage`, `test_training_coverage`)
walk the timing and Ordered Set tables of the specification one row at a time. The target tests
(`test_targets`) elaborate the AMD bench targets (Acorn, KC705, XCKU3P) with every example they
support and with the debug options, without an FPGA toolchain; `test_gw5` and `test_ecp5` cover the
Gowin and Lattice transceivers. A change that breaks a board is caught in seconds.

### Markers and commands {#sim-commands}

Tests carry pytest markers that select them by cost. `pytest.ini` deselects the slow ones by
default.

| Marker | Content | Default run |
|----------|----------------------------------------------------|-------------|
| none | Migen simulations and elaborations | Yes |
| `sim` | Verilator simulations of `sim.py` | Yes |
| `slow` | Long Verilator simulations (ppm, SSC, long PRBS runs) | No |

```sh
# Fast suite, as the continuous integration runs it
python3 -m pytest -v -n auto -m "not slow and not sim"
# Verilator simulations
python3 -m pytest -v -n 2 -m "not slow and sim"
# Long simulations
python3 -m pytest -v -n auto -m slow
# Full suite
python3 -m pytest -v -m ""
```

The `-n` option runs the tests in parallel and needs the `pytest-xdist` package. Each test writes
its outputs in its own directory, `build/test/<test id>/`, kept after the run for inspection.

## Full-system simulation {#sim-system}

The full-system simulation puts a host and a device face to face, each with its own PHY and a
transceiver model, as the figure below shows. It runs the real link training from LFPS to U0, then
the link initialization, the enumeration and Bulk transfers through the device core. The script
`sim.py` builds it with the LiteX Verilator simulator.

![Full-system simulation: the host model and the device each have a PHY and a transceiver model.
The cyan arrows are the only link between the two sides; the clock offset, the spread spectrum and
the word shift apply there.](sim_setup.svg)

The transceiver models (`USB3SerDesModel`) encode and decode 8b/10b, emulate LFPS, polarity and a
receive word shift, and receive in the clock domain of the partner, as the clock recovery of a real
transceiver does. The host and device clocks come from separate generators, so a frequency offset
and spread spectrum clocking can be applied to the device side.

```sh
# Gen 1 link training, enumeration and Bulk transfers
./sim.py
# Same with a 300 ppm clock offset and spread spectrum clocking
./sim.py --ppm 300 --ssc
# Gen 2 training to U0, SuperSpeedPlus link layer
./sim.py --gen2
# Device link layer and core at half the PIPE frequency (slow fabrics)
./sim.py --dev-pipe-cd
# Waveforms in build/sim/gateware/
./sim.py --trace --trace-start 0 --trace-end 200000
```

The main options are below; [Build options](#build-options) lists them all.

| Option | Effect |
|--------------------------|---------------------------------------------------------------|
| `--no-core` | PRBS stand-ins replace the device core; data checked on both sides in U0 |
| `--timer-scale N` | Divide the LTSSM and LFPS timers by `N` (shorter runs) |
| `--ppm`, `--ssc` | Device clock offset, spread spectrum clocking on the device |
| `--rx-word-shift N` | Receive word shift of the transceiver models (0 to 3 symbols) |
| `--gen2-capable` | Advertise and negotiate Gen 2 in the SuperSpeedPlus polling |
| `--gen2` | Train to U0 at Gen 2, 128b/132b blocks |
| `--lanes 2` | Gen 1x2: two lanes with skew and per-lane polarity, PRBS stand-ins |
| `--dev-mode`, `--lanes-crossed` | With `--lanes 2`: Gen 1x1 fallback, flipped USB Type-C plug |
| `--link-words N` | Device link layer on 32, 64 or 128-bit words |
| `--dev-pipe-cd` | Device LTSSM, link layer and core at half the PIPE frequency |
| `--host-pm`, `--host-lpm` | Host U1 and U2 requests or entries after the enumeration |
| `--host-recovery` | Host-directed Recovery from U0 after the script |
| `--host-hot-reset N` | `N` host-directed Hot Resets from U0, back to back |
| `--host-lbad-test` | Bulk OUT burst with a header CRC error: LBAD, LRTY, retransmission |

The host model prints each packet it receives as a `HOST_RX` line and marks each script section with
`HOST_MARK`. The system tests (`test_sim_host`, `test_phy`) parse these lines and check the
enumeration answers and the Bulk loopback data, so a `sim.py` run is also a pass or fail test.

## Continuous integration {#sim-ci}

The CI workflow (`.github/workflows/ci.yml`) runs on every push and pull request, and every night.
It installs a pinned Verilator, LiteX with its standard cores, and the project, then runs:

1. the fast suite (`-m "not slow and not sim"`);
2. the Verilator simulations (`-m "not slow and sim"`);
3. the long simulations (`-m slow`), only on the nightly and manual runs.

## Adding a test {#sim-add-test}

A new test goes in `test/test_<area>.py`, as a `unittest.TestCase`, with the reference model of the
rule it checks. The helpers of `test/common.py` give symbol and word conversions, stream drivers
and monitors, and a fresh output directory per test (`get_output_dir`).

- Prefer a Migen unit test with a Python reference model; reserve Verilator for what needs both
  PHYs and real timers.
- Mark a Verilator test with `pytestmark = pytest.mark.sim`, and a test of more than a minute
  with `slow`.
- Seed every random generator, so a failure replays.
- Keep the default suite fast: shorten timers with `timer_scale` in simulation only, never in a
  hardware build.

Where to go next: [Debugging](#debugging) for the hardware side of the loop, and
[Build options](#build-options) for every option of `sim.py`.
