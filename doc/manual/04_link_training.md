# Link training {#link-training}

This chapter describes how the PHY brings the link up and keeps it up: the LFPS signaling, the
training sequences and the LTSSM that sequences them. It also covers Recovery, the low power
states and the resets. Read it to understand a link that does not reach U0, together with
[Debugging](#debugging). The concepts are introduced in [LFPS](#primer-lfps),
[Training](#primer-training) and [The LTSSM](#primer-ltssm) of the primer; this chapter
describes how the PHY implements them.

## At a glance {#lt-glance}

| Item                    | Value                                                              |
|-------------------------|--------------------------------------------------------------------|
| LFPS                    | `LFPSUnit` (`usb3_pipe/lfps.py`), USB 3.2 section 6.9              |
| Training sequences      | `TSUnit` (`usb3_pipe/training.py`), TSEQ, TS1 and TS2              |
| LTSSM                   | `USB3LTSSM` (`usb3_pipe/ltssm.py`), USB 3.2 section 7.5            |
| SuperSpeedPlus Polling  | `SSPPolling` (`usb3_pipe/ltssm.py`), SCD and LBPM                  |
| States                  | Polling, U0, Recovery, U1, U2, U3, Hot Reset, Warm Reset, Compliance, Loopback, eSS.Inactive |
| Timers                  | Specification values, divided by `timer_scale` in simulation       |
| Not modelled            | Rx.Detect (receiver detection), electrical parameters              |

## LFPS {#lt-lfps}

Before the link carries any symbol, the two ports talk with LFPS: bursts of a 10 to 50 MHz
square wave separated by electrical idle. The figure below shows the two durations that identify
a pattern.

![LFPS signaling: bursts of a slow clock separated by electrical idle. Look at the burst length
(tBurst) and the repeat period (tRepeat): together they identify the pattern.](lfps_waveform.svg)

LFPS needs no trained receiver, only a transmitter that can idle and a receiver that detects
idle. The PHY generates the square wave as a pattern of transceiver words, at 25 MHz by default
(`LFPSUnit`). On receive, it measures the bursts on the idle signal of the transceiver after a
two-sample deglitcher (`LFPSIdleDeglitcher`). [Electrical idle and LFPS](#gt-lfps) describes the
transceiver side.

### Patterns {#lt-lfps-patterns}

The table below gives the burst the PHY transmits for each pattern, its accepted range or its
detection threshold, and its repeat period. The values come from USB 3.2 tables 6-30 and 6-31.

| Pattern            | Burst (typical)    | Repeat         | Use                                      |
|--------------------|--------------------|----------------|------------------------------------------|
| Polling.LFPS       | 1 µs (0.6-1.4 µs)  | 10 µs (6-14 µs)| Start of the link training               |
| Ping.LFPS          | 190 ns (40-200 ns) | 200 ms         | Keep-alive in U1                         |
| U1 exit            | 900 ns             | single         | Leave U1, detected from 300 ns           |
| U2 or Loopback exit| 100 µs             | single         | Leave U2 or Loopback, detected from 80 µs|
| U3 wakeup          | 1 ms               | single         | Leave U3, detected from 80 µs            |
| Warm Reset         | 100 ms (80-120 ms) | single         | Reset the link, detected after 25 ms     |

The checkers measure both the burst and the gap. A Polling.LFPS burst is accepted only when its
length and its period both fall in the specification window. Ping.LFPS and U1 exit are single
bursts, told apart by their length alone.

Warm Reset is detected differently. During a long LFPS burst, the 8b/10b decoder sees only
errors. A burst of decode errors longer than 25 ms, the Warm Reset detection delay, is a Warm
Reset. This also keeps the longest wakeup burst, 10 ms, from being taken for one.

### Transmission priority {#lt-lfps-tx}

The LTSSM asks for one pattern at a time, but the unit enforces a priority: Warm Reset, U3 wakeup,
U2 exit and U1 exit first, then Ping, then Polling. With SuperSpeedPlus Polling, the SCD and LBPM
signaling below has priority over all of them on the configuration lane.

### SuperSpeedPlus signaling {#lt-lfps-ssp}

A Gen 2 capable port adds information to the Polling.LFPS bursts. The repeat period encodes bits:
6 to 9 µs is a 0, 11 to 14 µs is a 1. Four bursts form a SCD, a SuperSpeedPlus Capability
Declaration: SCD1 is `0010b` and SCD2 is `1101b`, sent LSb first (`SCDGenerator`, `SCDChecker`).

Once both ports have exchanged SCDs, they send LBPM messages, LFPS Based PWM Messages. Each bit is
a 2.2 µs period: a 0 is one third LFPS, a 1 is two thirds. The PHY Capability LBPM advertises Gen 2
(bit 2) and two lanes (bit 5); the PHY Ready LBPM confirms the configuration (`LBPMGenerator`,
`LBPMChecker`).

## Training sequences {#lt-ts}

After LFPS, the ports switch to the 5 Gbps signaling and exchange Ordered Sets: fixed patterns
that the receiver recognizes even before the link is configured. The table below gives the three
training sequences.

| Ordered Set | Length     | Content                                                   | Sent in                      |
|-------------|------------|------------------------------------------------|------------------------------------|
| TSEQ        | 32 symbols | COM, a fixed equalization pattern, then 16 x D10.2        | Polling.RxEQ, 65536 times    |
| TS1         | 16 symbols | 4 x COM, a zero, link configuration, 10 x D10.2           | Polling.Active, Recovery     |
| TS2         | 16 symbols | 4 x COM, a zero, link configuration, 10 x D5.2            | Polling.Configuration, Recovery |

The role of each sequence is explained in [Training](#primer-training). The link configuration
symbol of TS1 and TS2 has three flags: Hot Reset (bit 0), Loopback (bit 2) and Disable Scrambling
(bit 3). The PHY decodes them from the received TS2 and sets them in its own TS2 on request of the
LTSSM.

A receiver whose differential pair is swapped sees an inverted TS1: the D10.2 symbols become
D21.5. The PHY detects both forms (`TS1`, `TS1_INV`), and the LTSSM inverts the receive polarity
in Polling.Active when it sees the inverted one.

### In the code {#lt-ts-code}

`TSUnit` holds a checker per Ordered Set, each reporting a detection after 8 consecutive valid
Ordered Sets, and a generator per Ordered Set: 65536 TSEQ, or bursts of 16 TS1 or TS2. Requests
take effect on Ordered Set boundaries, so an Ordered Set is never truncated. The checkers run all
the time, because a TS1 in U0 means the partner entered Recovery.

## The LTSSM {#lt-ltssm}

The Link Training and Status State Machine sequences everything above. It asks the LFPS unit and
the training unit for patterns, watches their detections and timers, and reports the link state
to the link layer (`USB3LTSSM`). It implements USB 3.2 section 7.5 with a few simplifications,
listed in [Simplifications](#lt-simplifications).

### Polling {#lt-polling}

Polling is the training from scratch, after power-on, a reset or a failure. The figure below
shows its substates, from the first LFPS to U0, and the timeouts that restart it.

![LTSSM Polling: each substate waits for an exchange with the partner. A timeout sends the LTSSM
to Rx.Detect, which restarts the training.](ltssm_polling.svg)

The list below gives what each substate does and when it leaves.

- **Polling.LFPS**: both ports send Polling.LFPS. The LTSSM waits for at least 16 bursts sent and
  one received, then sends 4 more bursts.
- **Polling.RxEQ**: the PHY sends 65536 TSEQ while the receiver aligns on the words of the
  partner. The partner sends TSEQ too.
- **Polling.Active**: the PHY sends TS1 until it has received 8 consecutive TS1 or TS2, then
  finishes its burst of 16. An inverted TS1 flips the receive polarity here.
- **Polling.Configuration**: the PHY sends TS2 until it has received 8 consecutive TS2 and sent 16
  TS2 after the first one.
- **Polling.Idle**: the PHY sends Idle symbols, scrambled zeros, and runs the Idle handshake:
  8 Idle symbols received and 16 sent after the first one. Then the link is in U0.

Polling.Idle also checks the TS2 flags: the Loopback flag leads to Loopback, the Reset flag to
Hot Reset. The Disable Scrambling flag is decoded here.

A timeout in Polling.LFPS on the very first training leads to Compliance, the electrical test
mode, instead of Rx.Detect. This only applies before any Polling.LFPS handshake has succeeded
since power-on or Warm Reset; later, a timeout is a training failure.

### SuperSpeedPlus Polling {#lt-ssp-polling}

With `gen2_capable` or two lanes, Polling.LFPS runs the SuperSpeedPlus substates first
(`SSPPolling`). The ports declare their capabilities with SCDs, then agree on the operation
with LBPMs, as the figure below shows.

![SuperSpeedPlus Polling: SCD1 and SCD2 declare SuperSpeedPlus, the PHY Capability and PHY Ready
LBPMs agree on the operation, then the training continues at the negotiated rate. A partner that
sends no SCD is a SuperSpeed port.](ltssm_ssp_polling.svg)

The negotiated operation is the highest common one: Gen 1x1, Gen 1x2, Gen 2x1 or Gen 2x2 (USB 3.2
table 7-14). After 16 plain Polling.LFPS bursts received and 4 SCD1 sent, the partner is a
SuperSpeed port: the LTSSM falls back to the Gen 1 Polling.LFPS handshake. Without matching LBPMs
within 12 ms, SuperSpeedPlus Polling fails.

When Gen 2 is negotiated, the transceiver switches to 10 Gbps before Polling.RxEQ. The rest of the
training uses the Gen 2 TSEQ, TS1 and TS2 inside 128b/132b blocks (`Gen2PIPE`).

### U0 and Recovery {#lt-recovery}

U0 is the working state: the link layer exchanges packets and the PHY passes them through. Three
events send the LTSSM from U0 to Recovery, a short retraining that keeps the link configuration.
The list below gives them.

- TS1 received: the partner has entered Recovery.
- No link activity for 1 ms: the link layer has seen nothing from the partner.
- Directed: the link layer asks for it with `directed_recovery`, after repeated errors.

Recovery runs the end of the Polling sequence again: Recovery.Active sends TS1,
Recovery.Configuration sends TS2, Recovery.Idle runs the Idle handshake. No LFPS and no TSEQ are
needed. A timeout in Recovery leads to eSS.Inactive. The figure below places Recovery among the
other link states.

![LTSSM link states around U0. Follow the arrows out of U0: the low power states are entered
with the LGO_Ux handshake and always left through Recovery. Recovery returns to U0 after its Idle
handshake, or goes to Hot Reset or, on a timeout, to eSS.Inactive.](ltssm_states.svg)

During Recovery.Idle, the partner can already be in U0 and send its first messages. The PHY keeps
these received words for the link layer (`rx_keep`) so they are not lost.

### Power management {#lt-pm}

The low power states save energy when the link is idle. U1 and U2 are quick to leave; U3 is
suspend. A port enters them through a handshake of link commands, sent by the link layer on
request of the LTSSM. The figure below shows a U1 entry asked by the host, then a U1 exit started
by the device.

![U1 entry and exit: LGO_U1, LAU and LPMA link commands, then the U1 exit LFPS handshake before
Recovery and U0.](ltssm_pm.svg)

The steps are the same for all three states. The initiator sends LGO_Ux. The acceptor answers LAU
to accept or LXU to reject, and the initiator confirms with LPMA. The acceptor enters Ux on LPMA,
or when its PM_ENTRY_TIMER (8 µs) expires without one.

- Without an answer within PM_LC_TIMER (4 µs), the initiator goes to Recovery.
- An upstream port never rejects U3. A device only requests U1 or U2, never U3.
- When both ports send LGO_Ux at once, the downstream port rejects the request of the upstream
  port and keeps its own.

To leave U1, U2 or U3, one port sends an exit LFPS burst and the other answers with its own; both
then go to Recovery. Without an answer within tNoLFPSResponseTimeout, the outcome depends on the
state:

- In U1 and U2 (2 ms), the port goes to eSS.Inactive.
- In U3 (10 ms), the port stays in U3 and retries the wakeup after tU3WakeupRetryDelay (100 ms).

U3 entry has its own failure rule: a downstream port whose LGO_U3 times out three times in a row
goes to eSS.Inactive instead of Recovery.

Three timers run in U1. A downstream port expects a Ping.LFPS at least every 300 ms. U1 changes
to U2 when the `u2_timeout` set by the host expires, in steps of 256 µs. A device leaves U1 only
after the minimum residency of 3 µs.

### Resets {#lt-resets}

USB 3 has two resets. Hot Reset is in-band: the downstream port sets the Reset flag of its TS2, and
both ports go to HotReset.Active, then HotReset.Exit, and back to U0 after an Idle handshake. The
link configuration is kept.

A port leaves HotReset.Active when its own reset is done and the partner's Reset flag has fallen,
at the end of a full burst of 16 TS2 with the flag cleared, started after that fall (USB 3.2
section 7.5.12.2). Without
that burst, a partner that entered HotReset.Active later never saw a cleared flag, and both ports
timed out to eSS.Inactive.

Warm Reset is an LFPS burst of 100 ms. A port detects it in any state and goes back to Rx.Detect,
then trains from Polling. Only a downstream port starts one, on `warm_reset_req`; an upstream port
answers for as long as the burst lasts. Warm Reset is also the only exit from Compliance.

### eSS.Inactive and Rx.Detect {#lt-inactive}

eSS.Inactive is the state of a failed link. The PHY stays quiet for 12 ms, then goes to Rx.Detect
and retrains. In the specification, a port leaves this state only on a Warm Reset or on a
disconnect, which it checks every 12 ms; the PHY cannot see a disconnect, so it retrains instead.

The PHY does not model Rx.Detect, the detection of a receiver termination on the cable. The
transient state Polling.ExitToRxDetect waits 64 cycles and restarts Polling. It also triggers the
transceiver receiver reset of [Resets](#phy-resets).

### Loopback and Compliance {#lt-test-modes}

Loopback and Compliance are test modes. In Loopback, the PHY sends back what it receives; it is
entered when the TS2 Loopback flag is set, as slave, or when directed with `directed_loopback`
and `loopback_capable`, as master. In Compliance, the PHY sends test patterns for electrical
measurements, and each Ping.LFPS received is counted in `compliance_ping_count`.

### Timers {#lt-timers}

The table below lists the LTSSM timers. All are specification values, scaled by `timer_scale`
only in simulation.

| Timer                         | Value  | State                                  | On expiry               |
|----------------------------------------|--------|--------------------------------|--------------------|
| tPollingLFPSTimeout           | 360 ms | Polling.LFPS                           | Rx.Detect or Compliance |
| tPollingActiveTimeout         | 12 ms  | Polling.Active                         | Rx.Detect               |
| tPollingConfigurationTimeout  | 12 ms  | Polling.Configuration                  | Rx.Detect               |
| tPollingIdleTimeout           | 2 ms   | Polling.Idle                           | Rx.Detect               |
| tU0RecoveryTimeout            | 1 ms   | U0, without link activity              | Recovery                |
| tRecoveryActiveTimeout        | 12 ms  | Recovery.Active                        | eSS.Inactive            |
| tRecoveryConfigurationTimeout | 6 ms   | Recovery.Configuration                 | eSS.Inactive            |
| tRecoveryIdleTimeout          | 2 ms   | Recovery.Idle                          | eSS.Inactive            |
| tHotResetActiveTimeout        | 12 ms  | HotReset.Active                        | eSS.Inactive            |
| tHotResetExitTimeout          | 2 ms   | HotReset.Exit                          | eSS.Inactive            |
| PM_LC_TIMER                   | 4 µs   | U0, LGO_Ux sent                        | Recovery                |
| PM_ENTRY_TIMER                | 8 µs   | U0, LAU sent                           | Enter Ux                |
| Ux_EXIT_TIMER                 | 6 ms   | Recovery, after a Ux exit              | eSS.Inactive            |
| tU1PingTimeout                | 300 ms | U1, downstream port                    | Rx.Detect               |
| tU3WakeupRetryDelay           | 100 ms | U3, after a failed wakeup              | Retry the wakeup        |
| teSSInactiveQuietTimeout      | 12 ms  | eSS.Inactive                           | Rx.Detect               |

### Simplifications {#lt-simplifications}

The LTSSM leaves out parts of the specification that an FPGA device does not need. The list below
gives them.

- Rx.Detect is not modelled: Polling.ExitToRxDetect restarts the training at once, and
  eSS.Inactive retrains after its 12 ms quiet period instead of waiting for a Warm Reset.
- In U1, U2 and U3, a Polling.LFPS from the partner means it restarted: the LTSSM goes to
  Polling.Entry.
- The TS2 flags and the Idle handshake are handled by the PIPE, not by the LTSSM.
- Electrical parameters and the 20 ns gap rule are not modelled.
- SuperSpeedPlus Polling has no tPollingSCDLFPSTimeout, no re-timer support and no capability
  fallback after a training failure.

## Status and debug {#lt-debug}

The LTSSM state is visible on `ltssm_state` and through the status outputs of
[Link interface](#phy-link-interface). The debug module `USB3PHYDebug` keeps a history of the
last LTSSM transitions with timestamps and counts the events: training sequences and LFPS
detected, receive errors, entries in U0, Recovery, U1, U2 and U3. Its registers are listed in
[Registers](#registers); how to read them on a board is in [Debugging](#debugging).

Never retrain the link from the debug registers while the device is in U3: the host considers the
device suspended and may not recover the link.

Where to go next: [Datapath](#datapath) for the symbols after training,
[Core architecture](#core-architecture) for the link layer side of power management.
