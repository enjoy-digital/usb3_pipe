#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3LTSSM drivers/harnesses for Migen simulations (LTSSM inputs driven as the PIPE/LFPS/TS units
and the link partner would)."""

import unittest

from migen import *

from usb3_pipe.ltssm import USB3LTSSM

# Helpers ------------------------------------------------------------------------------------------

def wait_for(dut, signal, timeout=64):
    """Wait (up to timeout cycles) for signal, return True if it was asserted."""
    for _ in range(timeout):
        if (yield signal):
            return True
        yield
    return False

def pulse(signal):
    """Pulse signal for one cycle, then one more cycle."""
    yield signal.eq(1)
    yield
    yield signal.eq(0)
    yield

# LTSSM Test Case ----------------------------------------------------------------------------------

class LTSSMTestCase(unittest.TestCase):
    """Single USB3LTSSM (with_timers=False) driven to U0: LFPS/TS units and partner inputs pulsed
    (Polling.LFPS -> Polling.RxEQ -> Polling.Active -> Polling.Configuration -> Polling.Idle -> U0)."""
    def wait_for(self, dut, signal, timeout=64):
        return (yield from wait_for(dut, signal, timeout))

    def drive_to_u0(self, dut, inverted=False):
        yield dut.lfps_tx_count.eq(20)
        yield dut.lfps_rx_polling.eq(1)
        yield
        yield dut.lfps_tx_count.eq(24)
        yield

        self.assertEqual((yield from self.wait_for(dut, dut.ts_tx_tseq)), True)
        yield dut.ts_tx_done.eq(1)
        yield
        yield dut.ts_tx_done.eq(0)

        self.assertEqual((yield from self.wait_for(dut, dut.ts_tx_ts1)), True)
        if inverted:
            yield dut.ts_rx_ts1_inv.eq(1)
            yield
            yield dut.ts_rx_ts1_inv.eq(0)
        else:
            yield dut.ts_rx_ts1.eq(1)
            yield
            yield dut.ts_rx_ts1.eq(0)
        yield
        yield dut.ts_tx_done.eq(1)
        yield
        yield dut.ts_tx_done.eq(0)

        self.assertEqual((yield from self.wait_for(dut, dut.ts_tx_ts2)), True)
        yield dut.ts_rx_ts2.eq(1)
        yield
        yield dut.ts_rx_ts2.eq(0)
        yield
        # 16 TS2 sent after receiving the TS2s (7.5.4.9.2): end of the current burst, then a full one.
        for _ in range(2):
            yield dut.ts_tx_done.eq(1)
            yield
            yield dut.ts_tx_done.eq(0)
            yield

        self.assertEqual((yield from self.wait_for(dut, dut.u0)), True)
        yield

# LTSSM Link Harness -------------------------------------------------------------------------------

class LTSSMLinkHarness(Module):
    """Two USB3LTSSMs (host/dev, with_timers=False) back-to-back: Polling.LFPS bursts counted, TS
    bursts of 4 cycles, TS1/TS2 received when the partner's burst is done."""
    def __init__(self, sys_clk_freq=int(125e6)):
        self.submodules.host = host = USB3LTSSM(sys_clk_freq, with_timers=False)
        self.submodules.dev  = dev  = USB3LTSSM(sys_clk_freq, with_timers=False)

        self.host_force_ts1 = Signal()

        self.host_lfps_count = Signal(16)
        self.dev_lfps_count  = Signal(16)

        host_ts_counter = Signal(3)
        dev_ts_counter  = Signal(3)
        host_ts_active  = Signal()
        dev_ts_active   = Signal()
        host_ts_done    = Signal()
        dev_ts_done     = Signal()

        self.comb += [
            host_ts_active.eq(host.ts_tx_enable & (host.ts_tx_tseq | host.ts_tx_ts1 | host.ts_tx_ts2)),
            dev_ts_active.eq(dev.ts_tx_enable & (dev.ts_tx_tseq | dev.ts_tx_ts1 | dev.ts_tx_ts2)),
            host.lfps_tx_count.eq(self.host_lfps_count),
            dev.lfps_tx_count.eq(self.dev_lfps_count),
            host.lfps_rx_polling.eq(dev.lfps_tx_polling & (self.dev_lfps_count >= 20)),
            dev.lfps_rx_polling.eq(host.lfps_tx_polling & (self.host_lfps_count >= 20)),
            host.ts_tx_done.eq(host_ts_done),
            dev.ts_tx_done.eq(dev_ts_done),
            host.ts_rx_ts1.eq(self.host_force_ts1 | (dev_ts_done & dev.ts_tx_ts1)),
            host.ts_rx_ts1_inv.eq(0),
            host.ts_rx_ts2.eq(dev_ts_done & dev.ts_tx_ts2),
            dev.ts_rx_ts1.eq(host_ts_done & host.ts_tx_ts1),
            dev.ts_rx_ts1_inv.eq(0),
            dev.ts_rx_ts2.eq(host_ts_done & host.ts_tx_ts2),
        ]

        self.sync += [
            If(host.lfps_tx_polling,
                self.host_lfps_count.eq(self.host_lfps_count + 1)
            ).Else(
                self.host_lfps_count.eq(0)
            ),
            If(dev.lfps_tx_polling,
                self.dev_lfps_count.eq(self.dev_lfps_count + 1)
            ).Else(
                self.dev_lfps_count.eq(0)
            ),
            host_ts_done.eq(0),
            If(host_ts_active,
                host_ts_counter.eq(host_ts_counter + 1),
                If(host_ts_counter == 3,
                    host_ts_done.eq(1),
                    host_ts_counter.eq(0)
                )
            ).Else(
                host_ts_counter.eq(0)
            ),
            dev_ts_done.eq(0),
            If(dev_ts_active,
                dev_ts_counter.eq(dev_ts_counter + 1),
                If(dev_ts_counter == 3,
                    dev_ts_done.eq(1),
                    dev_ts_counter.eq(0)
                )
            ).Else(
                dev_ts_counter.eq(0)
            ),
        ]
