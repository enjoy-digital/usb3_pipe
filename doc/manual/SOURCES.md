# Sources of the manual's facts

Every number and name in the manual is taken from the repository at the documented revision. This
file records where each chapter's facts come from, so that a reviewer can re-check them and a
maintainer knows what to update when the code changes.

| Chapter / section | Sources |
|---|---|
| Front matter | `README.md`, repository tree |
| 1 Introduction | `README.md` (status table, hardware list, credits, demos), `setup.py` (`usb3_pipe_gen`), `examples/*.yml`, `usb3_core/examples.py` (link words per example), repository tree |
| 2 Layers, wire | `usb3_pipe/phy.py` (link interface), `usb3_pipe/serdes.py` (`ECP5USB3SerDes.rx_settings`: RX termination) |
| 2 LFPS | `usb3_pipe/lfps.py` (timings, module docstring: SCD1/SCD2, LBPM) |
| 2 Coding, scrambling | `usb3_pipe/common.py` (symbols, TS1/TS2/TSEQ), `usb3_pipe/scrambling.py` (polynomials), `usb3_pipe/datapath.py` (SKP interval), `usb3_pipe/coding.py` |
| 2 Training | `usb3_pipe/training.py` (65536 TSEQ), `usb3_pipe/core.py` (idle handshake), `usb3_pipe/common.py` (link configuration bits, TS1_INV) |
| 2 LTSSM, power states | `usb3_pipe/ltssm.py` (states, Rx.Detect not modelled), `usb3_core/link.py` docstring (LPM) |
| 2 Packets, link layer | `usb3_core/common.py` (framing, link commands, packet and TP types), `usb3_core/link.py` (header buffers, LUP period, Type 1/Type 2), `usb3_pipe/crc.py` |
| 2 Protocol layer, enumeration | `usb3_core/protocol.py`, `bulk.py`, `iso.py`, `control.py`, `descriptors.py` docstrings |
| 3 At a glance, overview, modules | `usb3_pipe/*.py` module docstrings, `README.md` (hardware status) |
| 3 Link interface | `usb3_pipe/phy.py` (`USB3PHY_STATUS`, `USB3PHY_LINK_CTRL`, `USB3PHY_LINK_INTERFACE`, `USB3PHY` docstring), `usb3_pipe/framing.py` (`gen2_link_layout`), `usb3_pipe/ltssm.py` (`u2_timeout`) |
| 3 Clock domains, resets | `usb3_pipe/serdes.py` (`add_xcvr_datapath`), `usb3_pipe/datapath.py` (`TXDatapath` CDC, `GEN2_DATA_WIDTH`), `usb3_pipe/phy.py` (receiver reset, `do_finalize`) |
| 3 Slower fabrics (`pipe_cd`) | `usb3_pipe/phy.py` (`LinkWordsCDC`, `add_pipe_cd_datapath`, `add_ltssm_pipe_cdc`), `usb3_pipe/core.py` (`Gen2PIPE.link_rx_cd`), `usb3_pipe/framing.py` (`LogicalIdleFilter`, `WordsPacker`, `PacketsBuffer`), `bench/sipeed_tang_mega_138k_pro.py`, `bench/lambdaconcept_ecpix5.py` |
| 3 Parameters, usage, Gen 2, lanes, limitations | `usb3_pipe/phy.py` (`__init__`, `add_gen2`, `add_lanes`), `usb3_pipe/lanes.py` |
| 4 LFPS | `usb3_pipe/lfps.py` (pattern constants, `ResetDetectDelay`, `LFPSUnit`, `SCDGenerator`, `LBPMGenerator`) |
| 4 Training sequences | `usb3_pipe/common.py` (`TSEQ`, `TS1`, `TS1_INV`, `TS2`, `LinkConfig`), `usb3_pipe/training.py` (`TSUnit`, `TSEQ_COUNT`) |
| 4 LTSSM, Polling, Recovery, power management, resets, timers | `usb3_pipe/ltssm.py` (header comments, timers, FSM states, HotReset.Active exit, `SSPPolling`, U3 entry and wakeup retry rules, eSS.Inactive quiet timeout), `usb3_pipe/core.py` (`IdleHandshake`), `test/test_ltssm_pm.py` |
| 4 Status and debug | `usb3_pipe/debug.py` (`USB3PHYDebug`), `AGENTS.md` (no retraining in U3) |
| 5 Gen 1 datapath, SKP, alignment | `usb3_pipe/datapath.py` (`TXDatapath`, `RXDatapath`, `TXSKPInserter`, `RXSKPRemover`, `RXWordAligner`, `RXErrorSubstitution`, `RXIdleFilter` `burst_cycles`), `usb3_pipe/core.py` (`USB3PIPE`), `usb3_pipe/common.py`, `usb3_pipe/serdes.py` (`ALIGN_COMMA_WORD`) |
| 5 Scrambling | `usb3_pipe/scrambling.py` (polynomials, seeds, rules) |
| 5 Gen 2 blocks, datapaths, framing | `usb3_pipe/common.py` (Gen 2 constants), `usb3_pipe/coding.py` (`Gen2BlockGearbox`, `Gen2BlockAligner`), `usb3_pipe/datapath.py` (`Gen2TX`, `Gen2RX`, `Gen2TXDCBalance` saturation), `usb3_pipe/framing.py`, `usb3_pipe/crc.py`, `doc/notes/gen2-x2-roadmap.md` |
| 5 Lanes | `usb3_pipe/lanes.py`, `usb3_pipe/phy.py` |
| 6 Wrapper interface, settings, LFPS | `usb3_pipe/serdes.py` (`USB3SerDes`, `add_transceiver`, `add_xcvr_datapath`, `add_lfps`, per-family classes, `A7USB3SerDes` `rx_idle_min_burst`), `test/test_rx_idle_filter.py` |
| 6 Clocking | `bench/acorn.py`, `kc705.py`, `alibaba_xcku3p.py`, `sipeed_tang_mega_138k_pro.py`, `lambdaconcept_ecpix5.py` (reference clocks, `oob`), `usb3_pipe/gen.py` |
| 6 Gowin GW5 | `usb3_pipe/serdes.py` (`GW5USB3SerDes`, `GW5RateSequencer`, `GW5TXElectricalIdle`), `usb3_pipe/gw5/`, `test/test_gw5*.py` |
| 6 Lattice ECP5, USB-C | `usb3_pipe/serdes.py` (`ECP5USB3SerDes`: LiteICLink `SerDesECP5PLL`, `SerDesECP5`, `rx_settings`, `xcvr_cd`, `tx_cdc`), `bench/lambdaconcept_ecpix5.py` (`TUSB322Reader`, mux pins, `pipe_cd` clocking, `USB3LinkStatus`), `test/test_ecp5.py` |
| 7 Device core | `usb3_core/device.py`, `link.py`, `protocol.py` (adapted endpoints: no 32-bit Bulk OUT on a wide link layer), `dpp.py`, `endpoints.py`, `bulk.py` (`USB3EndpointUpConverter`, `USB3EndpointDownConverter`, `bulk_endpoint_32`, `USB3BulkOUT` `with_zlp`), `slots.py` (packet length from `be`), `iso.py`, `common.py` (`endpoint_layout`, `be_to_bytes`: zero-length Bulk IN packet), `control.py`, `descriptors.py`, `wishbone.py`, `trace.py`, `examples.py` (link layer widths of the examples), `bench/lambdaconcept_ecpix5.py` (83.33 MHz) |
| 8 Device classes | `usb3_core/cdc_acm.py`, `cdc_ecm.py` (descriptors, frames, zero-length packets, notifications), `bridge.py`, `uvc.py` (`UVC_MODES`, `UVC_ISO_MODES`), `msc.py` (LUNs, sense codes, UNIT ATTENTION, START STOP UNIT), `msc_storage.py` (`capacity`, `removable`; Bulk endpoints at the link layer width, `bulk_endpoint_32`), `bench.py` (`USB3Bench` widths), `protocol.py` (no 32-bit Bulk OUT on a wide link layer), `examples.py` (composite), `gud.py` (GUD display: requests table, `USB3GUDFunction`), `control.py` (request fields without handler), `test/test_core_class_wide.py`, `test/test_core_cdc_ecm.py`, `test/test_core_msc_luns.py`, `test/test_core_gud.py` |
| 8 Network adapters | `usb3_core/cdc_ecm.py` (`USB3BulkTransferRX/TX`, notifications), `usb3_core/cdc_ncm.py` (NTB16, `NCMNTBParser`, `NCMNTBBuilder`), `test/test_core_cdc_ecm.py`, `test/test_core_cdc_ncm.py` |
| 9 Supported FPGAs and boards | `bench/*.py` targets (CRG, reference clocks, bridges, options, `pipe_cd` defaults), `bench/usb3_soc.py` (`usb3_soc_build`, `add_usb3_device`), `bench/lambdaconcept_ecpix5.py` (83.33 MHz clocking, `USB3LinkStatus`), `bench/lambdaconcept_ecpix5_bringup.py`, `usb3_pipe/serdes.py`, `usb3_pipe/gw5/`, `README.md` (status, hardware), the former targets and ECPIX-5 pages (results, timing margins), `doc/notes/gen2-x2-roadmap.md` (Gowin Gen 2) |
| 10 Getting started | `README.md` (prerequisites), `setup.py`, `sim.py`, `pytest.ini`, `bench/99-usb3-pipe.rules`, `bench/common.py`, `bench/usb3_bulk.py`, `bench/usb3_bench.py`, `litex_server` (LiteX) options |
| 11 Integration | `bench/usb3_soc.py`, `bench/acorn.py`, `bench/alibaba_xcku3p.py`, `bench/sipeed_tang_mega_138k_pro.py` (`pipe_cd` wrapper), `usb3_pipe/phy.py`, `usb3_pipe/serdes.py` (wrapper parameters, `add_timing_constraints`), `usb3_pipe/debug.py`, `usb3_core/device.py`, `bulk.py` (endpoint converters), `protocol.py` (adapted endpoints), `common.py`, `usb3_pipe/gen.py` (`CONFIG_DEFAULTS`, `get_ports`), `examples/*.yml`, the former integration and standalone core pages |
| 12 Examples | `usb3_core/examples.py` (examples allowed with `link_words` 2 or 4), `bench/usb3_soc.py` (`usb3_soc_args`, `--lean`), docstrings of the `bench/*.py` host tools |
| 13 Simulation | `sim.py` (argument parser, `--host-hot-reset`), `usb3_core/host_model.py`, `test/model/*.py`, `test/common.py`, `test/*.py`, `pytest.ini`, `.github/workflows/ci.yml` |
| 14 Debugging | `bench/usb3_soc.py` (`add_usb3_profile`: throughput profile), `litex_cli` (LiteX) usage with `--csr-csv`, `usb3_pipe/debug.py` (`USB3PHYDebug`, `USB3SerDesDebug`, `usb3_analyzer_groups`), `usb3_core/trace.py`, `bench/usb3_{debug,analyzer,trace,soak,suspend,bulk,bench}.py`, `bench/acorn_run.py`, `bench/usb3_soc.py`, `bench/sipeed_tang_mega_138k_pro.py` (GTR12 bring-up registers), `bench/lambdaconcept_ecpix5.py` (`USB3LinkStatus`), `usb3_core/device.py` (`add_csr`) |
| A Registers | Generated by `tools/csr_doc.py` from the elaborated `bench/acorn.py` and `bench/alibaba_xcku3p.py` (snapshots in `csr/`) |
| B Build options | Argument parsers of `bench/usb3_soc.py`, the `bench/*.py` targets (`set_defaults` of the ECPIX-5 target), `sim.py`, `usb3_pipe/gen.py`; `usb3_core/examples.py` (examples per option) |
| C Glossary | Chapters 1 to 14 |
| D Performance and resources | `README.md` (status table, hardware list: Tang Mega 138K Pro, ECPIX-5, GW5AT-15 results and clocks), `bench/lambdaconcept_ecpix5.py` (83.33 MHz, max burst 16), `bench/usb3_bench.py`, `bench/usb3_resources.py`, the measurements of the former device core, examples and ECPIX-5 pages |
