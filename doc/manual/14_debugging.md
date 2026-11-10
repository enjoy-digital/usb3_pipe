# Debugging on hardware {#debugging}

This chapter explains how to debug a USB 3 link and a device on a board, without a USB protocol
analyzer. It describes the method, the debug cores built into the bench targets, the host tools
that read them, and ends with a troubleshooting table. Read it before touching hardware.

## Method {#debug-method}

The approach is to observe before capturing. Small always-on counters and histories answer most
questions; a logic analyzer capture is only worth taking once the question is narrow. The host side
is read from the Linux kernel log, and every finding becomes a simulation test.

1. Check the connection to the board and the design first: the bridge answers and the identifier
   matches the build.
2. Read the registers: link state, state history, counters, measurements.
3. Capture a narrow LiteScope trace only when the question is about timing.
4. Reproduce the finding in simulation, fix it there, then check again on the board (see
   [Simulation and verification](#simulation)).

The figure below shows where each debug core taps the design. All of them are read from the host
through the CSR bus, over JTAG (JTAGBone) or a UART (UARTBone), with `litex_server`.

![Debug infrastructure. The dotted arrows show which datapath block each debug core observes,
from the transceiver on the left to the protocol layer on the right.](debug_taps.svg)

| Core | Taps | Built | Host tool |
|------------------|-------------------|---------------------------------------|---------------------------------|
| PHY debug | LTSSM, PIPE events | Except `--lean` and on the ECPIX-5 | `usb3_debug.py` |
| Transceiver debug | Transceiver, clocks, DRP | `--with-serdes-debug` | `usb3_debug.py` |
| LiteScope | PHY signals, by group | `--with-analyzer` | `usb3_analyzer.py` |
| Packet and link traces | Link and protocol layers | Except `--lean`, see below | `usb3_trace.py` |
| Core status | Link layer errors | With the device core | `usb3_soak.py` |

The host tools are in `bench/` and share the `--csr-csv`, `--host` and `--port` options. The
register names they use are listed in [Register reference](#registers).

## Setup {#debug-setup}

The reference setup is a LiteX Acorn Baseboard Mini with an Acorn CLE-215+, an SFP2USB adapter in
the SFP cage and a USB 3 cable to a Linux host. JTAG goes through the FT4232H of the baseboard.
The debug options are the same on every target; the bridge and the loading of the other boards are
in [Target scripts](#boards-targets).

- **Select the board explicitly.** The programmer of a platform picks the first FTDI adapter it
  finds. When several are connected, give the USB location of the right one; `bench/acorn_run.py`
  refuses to guess and reads it from the `USB3_PIPE_FTDI` environment variable (a sysfs location
  such as `5-4`).
- **Use a dedicated `litex_server` port.** The host tools default to the LiteX port 1234. Another
  bridge may already listen there, and a client would then talk silently to another board.
- **Check the identifier first.** It must start with `USB3SoC`, the identifier of the bench SoC.
  `litex_cli` needs the `csr.csv` of the build to find it.

```sh
litex_server --jtag --bind-port 1240 &
litex_cli --port 1240 --csr-csv build/sqrl_acorn/csr.csv --ident
```

A debug build adds JTAGBone, the transceiver debug core and LiteScope probes. `bench/acorn_run.py`
then loads it and records a series of link training attempts in `build/hw_runs/<date>/`: LTSSM
state, history and counters, transceiver measurements and the host view of each attempt. It
starts its own `litex_server` on port 1240 (`--port`): stop any other server on that port first.

```sh
./bench/acorn.py --build --with-jtagbone --with-serdes-debug \
    --with-analyzer --analyzer-group link --output-dir build/acorn_dbg
./bench/acorn_run.py --build-dir build/acorn_dbg --attempts 20
```

## The host side {#debug-host}

Without root access, the host view comes from the kernel log and from sysfs. The kernel log is
`/var/log/kern.log`, readable by the `adm` group; the tools fall back to `journalctl -k` when the
file is missing (`--kern-log`). `lsusb -t` and the port states in
`/sys/bus/usb/devices/*/usb*-port*/state` complete it.

```sh
./bench/usb3_debug.py host
```

This command lists the SuperSpeed devices (identifiers, strings, speed, configuration) and the last
USB kernel messages. Enumeration messages arrive seconds after a link training: wait before reading
them. The table below lists the useful ones.

| Kernel message | Meaning |
|------------------------------------------------|---------------------------------------------------|
| "Cannot enable. Maybe the USB cable is bad?" | The link does not reach U0 for the host; it retries with a Warm Reset every 4 s or so |
| "can't read configurations, error -61" | Wrong descriptor type returned: check the descriptor tables |
| "can't set config #1, error -71" | Transaction error on a control transfer: check the Status stage ACK sequence number |

## PHY debug registers {#debug-phy}

The PHY debug core (`USB3PHYDebug`, in `usb3_pipe/debug.py`) is light and built by default. It gives
the LTSSM state, a history of its last 32 transitions with microsecond timestamps, and event
counters. The history answers the first question of any link problem: how far did the training go,
and how long did each state last.

```sh
D="--port 1240 --csr-csv build/acorn_dbg/csr.csv"
# State and counters
./bench/usb3_debug.py $D status
# Last 32 LTSSM transitions
./bench/usb3_debug.py $D history
# PHY reset and counters clear
./bench/usb3_debug.py $D retrain
# Print the changes
./bench/usb3_debug.py $D watch
```

| Information | Content |
|--------------------|---------------------------------------------------------------------|
| Status | LTSSM state, transceiver ready, receive electrical idle, receive polarity |
| History | Last 32 transitions (or the first ones after a clear, when frozen), state and time |
| Counters | TS1, inverted TS1 and TS2 detections; LFPS by kind; idle edges; receive error cycles; substituted K28.4 words; U0, Recovery, U1, U2 and U3 entries |
| PHY reset | Holds the PHY in reset with the transmitter in electrical idle |
| Receiver reset | Re-initializes the receiver of the transceiver |

Two targets build less. With `--lean` and a separate PHY clock domain (Tang Mega 138K Pro), a
minimal `usb3_phy_status` register keeps the LTSSM state, U0 and the transceiver ready. The ECPIX-5
target has no PHY debug core: its link status block (`usb3_link`) gives the LTSSM state and counts
the U0 and Recovery entries, the receive errors and idles in U0, and the cycles in U0.

Holding the PHY in reset shows what the link partner sends on its own. The receiver reset restarts
its equalization, clock recovery and offset calibration. The PHY also does it on each training
failure, because the receiver initialization does not always end in a good state: about one time
in twelve on the Artix-7 GTP with a host using spread spectrum clocking.

## Transceiver debug registers {#debug-serdes}

The transceiver debug core (`USB3SerDesDebug`) is added by `--with-serdes-debug`. It gives
controls, clock measurements, receive statistics and access to the transceiver attributes.

| Information | Content |
|--------------------|---------------------------------------------------------------------|
| Controls | Transmit polarity, transmit inhibit (always or in one LTSSM state), loopback mode |
| Clocks | Transmit and receive clock frequencies over 100 ms; receive versus transmit phase |
| Receive statistics | Symbols, decoder and disparity errors, error bursts, longest error-free run |
| DRP | Read and write of the transceiver attributes, without rebuilding |

The receive clock is the recovered clock, so its offset from the transmit clock shows the clock
recovery tracking the host. A host with spread spectrum clocking averages about -2500 ppm. An
offset of exactly 0 ppm means the receiver is not tracking the host, for example when it decodes
its own transmitter through crosstalk.

The longest error-free run over a training is a good figure of receive quality. It reaches millions
of symbols on a healthy link and about a thousand when the clock recovery slips bits.

The DRP commands try transceiver settings without a rebuild; apply them with a receiver reset. The
DRP addresses are those of the transceiver user guide (UG482 for the GTP). The build-time
equivalents of the Acorn target are `--rxcdr-cfg`, `--gtp-param` and `--lpm-hold`.

```sh
D="--port 1240 --csr-csv build/acorn_dbg/csr.csv"
# Clock recovery setting: read, then write
./bench/usb3_debug.py $D rxcdr-cfg
./bench/usb3_debug.py $D rxcdr-cfg 0x87fe206024481010
# Any attribute, and a dump to compare two builds
./bench/usb3_debug.py $D drp-read 0x2a
./bench/usb3_debug.py $D drp-dump drp.json
# Receiver initialization quality over 12 resets
./bench/usb3_debug.py $D rx-check --count 12
# Transmitter silent in one LTSSM state
./bench/usb3_debug.py $D tx-inhibit-state Polling.Active
```

Qualify each new bitstream with `rx-check`. A `no_partner` result is normal while the host backs
off between enumeration retries. Reference clock issues specific to a board, such as the Acorn
receiver quality depending on how the oscillator reaches the transceiver, are in
[Transceivers](#transceivers).

## LiteScope captures {#debug-litescope}

LiteScope captures a group of signals around a trigger. Keep captures narrow, well below 128 bits,
and in the system clock domain: captures in the 250 MHz receive clock domain do not meet timing.
The groups come from `usb3_analyzer_groups` and are selected with `--analyzer-group`.

| Group | Signals |
|------------|-----------------------------------------------------------------------------------|
| `link` | LTSSM, TS and polling LFPS detections and generation, receive idle, errors, polarity |
| `lfps` | LTSSM, LFPS transmit and receive of every kind |
| `ts` | LTSSM, TS detections and generation, transceiver streams |
| `datapath` | LTSSM, transceiver and link layer streams, SKP removal |
| `phase` | LTSSM, receive versus transmit clock phase (needs the transceiver debug core) |
| `u3` | LTSSM, U3 exit: LFPS, TS1, receive stream, clock phase (needs the transceiver debug core) |

`bench/usb3_analyzer.py` arms the capture, retrains the PHY until the trigger fires, then analyzes
the result. The analysis gives the LTSSM states, the receive errors per state, the longest
error-free runs, the reception of the partner's TSEQ (a known pattern, so errors per symbol
position), the link commands and headers, and the clock offset profile.

```sh
A="--port 1240 --csr-csv build/acorn_dbg/csr.csv"
A="$A --analyzer-csv build/acorn_dbg/analyzer.csv"
# Link initialization after the U0 entry (datapath group)
./bench/usb3_analyzer.py $A capture --state U0 --offset 256
# The partner's TSEQ (it is in Polling.RxEQ, the device in Polling.Active)
./bench/usb3_analyzer.py $A capture --state Polling.RxEQ:Polling.Active
# A control request data word, without retraining
./bench/usb3_analyzer.py $A capture --data 0x00090100 --no-retrain \
    --offset 1024
# Recovered clock offset over about 2 ms (phase group)
./bench/usb3_analyzer.py $A capture --state Polling.LFPS:Polling.RxEQ \
    --subsampling 64
# Analyze a capture again
./bench/usb3_analyzer.py analyze analyzer.vcd
```

Subsampling covers millisecond sequences on level signals: 4096 samples of 512 cycles at 8 ns make
16.8 ms. The LTSSM state encodings are given by `usb3_pipe.gen.get_ltssm_states()`. CSR writes
through `litex_server` and JTAGBone are buffered: read a register after arming the analyzer and
before causing the event to capture.

## Bulk data and stability {#debug-bulk}

The `loopback` example checks the data path with `usb3_bulk.py`, as in
[First transfers](#gs-transfers); `usb3_bulk.py info` prints the device and its endpoints. A
single run of a few seconds misses rare errors, so a link is qualified with a soak run.

`bench/usb3_soak.py` runs a long test and logs one JSON line per period: PHY counters (U0,
Recovery, U1, U2 and U3 entries), the enumeration state, the USB kernel messages, Bulk loopback
batches or a full-duplex benchmark, and periodic host reconnects.

```sh
S="--port 1240 --csr-csv build/sqrl_acorn/csr.csv"
# Loopback example: Bulk batches, reconnect every 150 s
./bench/usb3_soak.py $S --duration 300 --period 10 --bulk 64 \
    --reconnect-every 150 --log soak.jsonl
# Bench example: full duplex, data checked on both sides
./bench/usb3_bench.py both $S
./bench/usb3_soak.py $S --duration 300 --period 30 --bench 20
```

The device core status registers count the link layer errors: LBAD sent and received, Recovery
requests and their causes, link commands and data packets rejected. The soak logs them. A healthy
run keeps them at zero.

| Register | Content |
|--------------------------------------|-----------------------------------------------------|
| `usb3_core_status` | Ready, configured, address, LBAD and retry counts, Recovery requests |
| `usb3_core_recovery_cause` | Last cause and saturating counts per cause |
| `usb3_core_rx_errors` | Link command words rejected, data packets with a CRC-32 error |
| `usb3_core_recovery_request` | Write: direct the link to Recovery |

The causes are a header sequence number error, an LGOOD or LCRD out of order, and the pending
header and credit timeouts. Each Recovery request is counted once.

## Packet and link traces {#debug-traces}

The packet trace (`USB3CoreTrace`) records the header packets the protocol layer receives and sends,
with a timestamp; isochronous timestamp packets are skipped. Control transfers, bursts, NRDY and
ERDY flow control and retries become visible without an analyzer. All the examples enable it,
except with `--lean`, on the ECPIX-5, and on the Tang Mega 138K Pro at 62.5 MHz without
`--with-trace`.

```console
$ S="--port 1240 --csr-csv build/sqrl_acorn/csr.csv"
$ ./bench/usb3_trace.py $S clear
$ ./bench/usb3_trace.py $S dump
 523164.672us TX TP   ERDY      ept 1 dir 1 nump  2 seq  0
 523165.952us RX TP   ACK       ept 1 dir 1 nump  2 seq  0
 523165.952us TX DPH  ept 1 dir 1 seq  0 len     1
 523166.720us RX TP   ACK       ept 1 dir 1 nump  0 seq  1
```

The register prefix is `usb3_core` in the vendor Bulk and composite examples, and
`usb3_<class>_core` in the single-class examples; `usb3_trace.py` detects it (`--prefix`).
`clear-ept --ept N` records only endpoint `N` in a circular buffer and `stop --ept N` freezes it,
for example around a stalled transfer.

The link trace (`USB3LinkTrace`) goes one layer down. It records link commands, header packets,
U0 entries and exits, and Recovery requests with their causes. The link state register adds the
sequence numbers, the credits and the transmit header buffers. A circular link trace stops on a U0
exit or a Recovery request, so it keeps the last events before the problem.

```sh
X="--port 1240 --csr-csv build/alibaba_xcku3p/csr.csv"
./bench/usb3_trace.py $X link-clear --circular --filter
./bench/usb3_trace.py $X link
```

## Gen 2 {#debug-gen2}

With `--gen2-capable`, the XCKU3P target adds Gen 2 bring-up registers, described in
[Register reference](#registers). They belong to the SoC, so their names start with `main_` in
`csr.csv`. With a separate PHY clock domain, as on the Tang Mega 138K Pro, only
`usb3_gen2_capable` and `usb3_gen2_status` are added. They give the negotiated rate and let software withdraw the
Gen 2 capability for the next training. A receive monitor counts blocks, corrected headers,
alignment losses and Ordered Sets. Link command counters along the receive path freeze on an
out-of-order LCRD, and maxima record the longest pending LGOOD, LCRD and data packet stall.

Four snapshot buffers capture raw transceiver words and data blocks, before or after the
scrambler, in both directions. They stop on a U0 exit, or on an LBAD received; the transmit
snapshot then holds the units sent before it, to check the CRCs and the framing of the header
packets. A register also selects the GTY receive equalizer: LPM (the default) or DFE.

### Gowin GTR12 bring-up {#debug-gtr12}

The Tang Mega 138K Pro target has its own bring-up registers, selected by the options of
[Tang Mega 138K Pro options](#opt-tang). The wide ones are read 32 bits at a time: write the word
index to `<name>_sel`, then read `<name>_data`. Their layouts are in the `add_*` docstrings of
`bench/sipeed_tang_mega_138k_pro.py`.

| Option | Registers | Content |
|------------------------------------|------------------------------------------|----------------------|
| Always | `usb3_serdes_gtr12_status` | GTR12 lane status |
| See below | `usb3_serdes_gtr12_upar_*` | Runtime access to the GTR12 registers (UPAR) |
| `--gen2-capable` | `usb3_gen2_capable`, `usb3_rx_flow` | Gen 2 capability and status, Type 2 credits, receive throttle level |
| `--with-gen2-debug` | `usb3_gen2_rx_*` | Gen 2 receive monitor on the raw words |
| `--with-tx-check` | `usb3_tx_check_*` | Transmitted header and link command CRCs |
| `--with-tx-raw-check` | `usb3_tx_raw_check_*` | Same check after the scrambler, block counts |
| `--with-ltssm-history` | `usb3_tx_ffe`, `usb3_rx_eq` | Equalizer settings, LGOOD latency, Gen 2 receive probes |
| `--with-gtr12-debug` | `gtr12_transitions_*` | Lane status transitions, snapshots, polling state |
| `--with-gtr12-analyzer` | `analyzer.csv` | LiteScope on the raw receive words |

The UPAR registers are built with `--with-gtr12-debug`, with `--gen2-capable` or with `sys` at
125 MHz. The Gen 2 receive monitor keeps its counts across the rate changes.

The transmit checks tell bad packets made by the transmit logic from packets corrupted on the line:
the first check runs before the scrambler, the second after it. Both keep a snapshot of the first
bad header packet.

`--with-ltssm-history` adds the transmit equalizer (FFE), receive equalizer (CTLE) and transmitter
re-initialization registers with `--gen2-capable`; the settings apply at the next 10 Gbps rate
change. Without `--with-gtr12-debug`, it also adds the LGOOD latency histogram and the Gen 2
receive backlog, counter and framing probes.

`--with-gtr12-debug` records the transitions of the lane status bits, which identify the LFPS and
electrical idle indicators. It also takes receive snapshots from a K28.5, a transmit snapshot at
the end of the training sequences, the last 64 transmitted words before a U0 exit, and the
SuperSpeedPlus polling state.

## Throughput profile {#debug-profile}

The `bench` example adds a throughput profile, unless the target is built with `--lean`. It
counts, in system clock cycles, where the link layer transmitter and the Bulk IN endpoint spend
their time. A throughput below the figures of [Performance](#performance) is explained by the
largest counter.

| Counters | What they count |
|------------------------------|-------------------------------------------------------------|
| `cycles`, `u0` | Measurement window, cycles in U0 |
| `link_tx_words`, `link_tx_stall` | Words sent to the PHY, cycles the PHY held them |
| `link_tx_full`, `link_tx_credit` | Transmit header buffers full, no credit from the host |
| `in_no_data`, `in_no_win` | Bulk IN idle: no user data, no window opened by the host |
| `in_fc`, `in_hp_wait` | Bulk IN in flow control, waiting for a header acknowledgement |
| `lgood_count`, `lgood_rtt_sum` | Header packets acknowledged, sum of their round trips |

The counters are 32-bit fields of the wide `usb3_profile` register, in the order of
`USB3_PROFILE_EVENTS` in `bench/usb3_soc.py`, `cycles` in the least significant bits. The
`usb3_profile_control` register clears them (`clear`) and freezes them (`run` at 0), so a window
can be measured during a `usb3_bench.py` run. The command below prints the register as one wide
value, to split into 32-bit words.

```sh
litex_cli --port 1240 --csr-csv build/sqrl_acorn/csr.csv \
    --regs --filter usb3_profile
```

## Suspend and resume {#debug-u3}

Linux suspends a device with a driver after a few seconds without activity: the link goes to U3,
and it resumes when the device is used again. The UVC, CDC-ACM and Mass Storage examples do this.

> Never retrain a link from the debug registers while it is in U3. The host still sees its port in
> U3: the device ends in a Polling.LFPS timeout, then in Compliance. `bench/acorn_run.py` reports
> such attempts as `suspended`.

`bench/usb3_suspend.py` waits for the suspend, resumes the device through its device node, and
reports each resume as `clean` (U3 exit) or `reset` (the exit failed and the host reset the
device). It arms the LTSSM history to restart on the U3 exit, so even a long failed exit is
captured from its first transition; times are relative to the U3 exit.

```console
$ ./bench/usb3_suspend.py $S --dev /dev/video0 --cycles 8
cycle 2: clean (0.6s), LTSSM: Recovery.Active@0.0ms ...
cycle 0: reset (1.7s), LTSSM: Recovery.Active@0.0ms eSS.Inactive@12.0ms ...
```

To capture a failed exit with LiteScope, trigger on eSS.Inactive, which is only reached when
Recovery.Active times out, with a large offset and `--no-retrain`. Desktop camera services also
open and resume UVC devices on their own; keep them in mind when counting resumes.

On the Acorn with an AMD xHCI host, all resumes are clean: 72 out of 72, in about 0.6 s.

### Case study: the U3 exit comma alignment {#debug-u3-case}

Before a fix, about half of the resumes ended with a host reset. The investigation shows the
method at work.

1. The registers showed the wakeup handshake completed: host LFPS detected, device response sent,
   then Recovery.Active, which timed out after 12 ms.
2. The `ts` and `phase` groups showed no TS1 decoded, 30 to 65% of the symbols in error, and a
   clock offset of exactly 0 ppm at the end. This looked like a receiver that never locked.
3. Changing the receiver settings, resetting the receiver, or the time spent in U3 did not help.
4. A capture taken right after the device response (`u3` group) told otherwise. The clock recovery
   was locked and tracking the host, but every Ordered Set was received misaligned: the comma
   aligner slid on each one, about 1100 times in 32 µs.

The cause was the GTP comma alignment on even bytes only. With two symbols per word, it conflicts
with the consecutive COM symbols of the USB 3 Ordered Sets when the first one arrives on an odd
byte, which is random after electrical idle. Commas are now aligned on any byte, and the word
aligner of the PHY handles the byte rotation.

## Useful experiments {#debug-experiments}

| Experiment | What it shows |
|--------------------------|-------------------------------------------------------------------------|
| Near-end PMA loopback | The LTSSM trains against itself to U0: transceiver at 5 Gbps, 8b/10b, alignment, TS detection and LTSSM checked without a partner |
| PHY held in reset | Receive activity then only comes from the partner |
| Transmit polarity inverted | If the receiver decodes its own transmitter (crosstalk), TS1 become inverted TS1 |
| Transmit inhibit in U0 | If the receiver goes idle or the recovered clock drifts, the partner was not transmitting |
| Host disconnect | Break the link for more than 10 s (`usb3_debug.py reconnect`): the host enumerates again, where a retrain only causes a Warm Reset |

The loopback mode is the near-end PMA loopback, value `0b010`, set before a retrain. One trap: the
scrambled logical idle of the partner looks like an echo of the device's own transmitter. Both
scramblers restart from the same seed after TS2, so the two idle streams match with a delay.

## Investigation loop {#debug-loop}

1. Reproduce with `bench/acorn_run.py`; note the build command, the bitstream and its timing.
2. Answer the next question with registers: history, counters, measurements.
3. Capture a narrow LiteScope trace when the question is about timing.
4. Reproduce in simulation (a Migen unit test or `sim.py`), fix, run the test suite including the
   slow tests, rebuild and check again on hardware.
5. Keep reusable probes and scripts in the tree, exploratory ones out of the commits.

> A bitstream that loops in Recovery can make the host xHCI controller give up and stop serving
> all its ports, until the host is rebooted or the controller reset. Simulate a change to the
> LTSSM or the link layer before loading it on a board connected to a workstation.

## Troubleshooting {#debug-troubleshooting}

| Symptom | Likely cause | Check | Chapter |
|--------------------------|--------------------------|--------------------------------|------------------|
| Wrong or no identifier from `litex_cli` | Another bridge on the port, wrong `csr.csv` | Server port, `--csr-csv` of the build | Debugging |
| Transceiver ready LED off | No reference clock, wrong frequency option | Clock meters, reference clock option | Transceivers |
| No device at all, no kernel message | Cable on a USB 2 port, bitstream not loaded | SuperSpeed port, `usb3_debug.py status` | Boards |
| "Access denied" from the host scripts | udev rule not installed | udev rule of `bench/` | Getting started |
| No device, "Cannot enable" in the kernel log | Link does not reach U0 | `usb3_debug.py` history: last state reached | Link training |
| Training stops in Polling.LFPS | No LFPS received, receive idle detection | LFPS counters, `lfps` group | Transceivers |
| Training stops in Polling.RxEQ or Polling.Active | Receiver not locked or misaligned | Error-free run, TS counters, `ts` group | Transceivers |
| Only inverted TS1 counted | Own transmitter received (crosstalk) or polarity | Transmit polarity experiment | Datapath |
| Receive clock offset exactly 0 ppm | Clock recovery not tracking the host | Clock meters, `phase` group | Transceivers |
| Link unstable on some builds only | Reference clock path or receiver settings | `drp-dump` between builds, `rx-check` | Transceivers |
| Repeated Recovery in U0 | Receive errors, credit or sequence timeouts | `recovery_cause`, link trace | Core architecture |
| Error -61 at enumeration | Wrong descriptor returned | Descriptors, packet trace | Device classes |
| Error -71 at enumeration | Control transfer error | Packet trace, `datapath` capture | Core architecture |
| LBAD or retries counted | Header CRC errors on the link | `rx_lbad`, `tx_retry`, soak | Datapath |
| Throughput below the expected figure | Burst size, transfer size, credits | Throughput profile, `--max-burst` | Performance |
| Resume ends with a host reset | U3 exit fails in Recovery | `usb3_suspend.py`, `u3` group | Link training |
| Host stops serving all ports | Recovery storm | Reboot the host, simulate the change | Simulation |

Work through the rows from the top: each one assumes the checks above it pass. The chapters named
in the last column are [Getting started](#getting-started), [Boards](#boards),
[Link training](#link-training), [Transceivers](#transceivers), [Datapath](#datapath),
[Core architecture](#core-architecture), [Device classes](#classes), [Performance](#performance)
and [Simulation and verification](#simulation); "Debugging" points to [Setup](#debug-setup).

Where to go next: [Register reference](#registers) for every register named here, and
[Build options](#build-options) for the debug options of the bench targets.
