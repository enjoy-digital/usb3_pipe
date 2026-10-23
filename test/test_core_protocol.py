#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.fhdl.specials import Instance

from usb3_core.daisho_mod.usb3.usb3_protocol import USB3Protocol

LT_U0           = 16
LP_TP_SUB_ACK    = 0b0001
LP_TP_SUB_STATUS = 0b0100

# Daisho Protocol (daisho_mod) ---------------------------------------------------------------------

class TestDaishoProtocol(unittest.TestCase):
    def test_status_ack_seq_after_set_configuration(self):
        # Control transfer without Data stage (ex: SET_CONFIGURATION): SETUP DP (seq 0), ACK TP, then
        # STATUS TP. The Status stage ACK TP shall carry the next expected sequence number (1), even
        # if EP0 requested a data packet sequence reset when decoding the request (8.12.2).
        dut = USB3Protocol()
        # Remove the Verilog endpoints (not simulated) and drive EP0's reset_dp_seq directly.
        ep0 = [s for s in dut._fragment.specials if isinstance(s, Instance) and s.of == "usb3_ep0"][0]
        reset_dp_seq = [i.expr for i in ep0.items if i.name == "reset_dp_seq"][0]
        dut._fragment.specials = {s for s in dut._fragment.specials if not isinstance(s, Instance)}
        dut.clock_domains.cd_sys = ClockDomain()

        def generator(dut):
            yield dut.reset_n.eq(1)
            yield dut.ltssm_state.eq(LT_U0)
            yield dut.tx_tp_a_ack.eq(1)
            yield dut.tx_tp_b_ack.eq(1)
            for _ in range(4):
                yield
            # SETUP DP (DPH seq 0 + DPP, CRC good).
            yield dut.rx_dph.eq(1)
            yield dut.rx_dph_setup.eq(1)
            yield dut.rx_dph_seq.eq(0)
            yield
            yield dut.rx_dph.eq(0)
            yield dut.rx_dpp_start.eq(1)
            yield
            yield dut.rx_dpp_start.eq(0)
            yield dut.rx_dpp_crcgood.eq(1)
            yield dut.rx_dpp_done.eq(1)
            yield
            yield dut.rx_dpp_done.eq(0)
            for _ in range(8):
                yield
            # EP0 decodes the request and requests a data packet sequence reset.
            yield reset_dp_seq.eq(1)
            yield
            yield reset_dp_seq.eq(0)
            yield
            # STATUS TP.
            yield dut.rx_tp.eq(1)
            yield dut.rx_tp_subtype.eq(LP_TP_SUB_STATUS)
            yield
            yield dut.rx_tp.eq(0)
            dut.status_acks = []
            for _ in range(8):
                yield
                if (yield dut.tx_tp_b):
                    dut.status_acks.append(((yield dut.tx_tp_b_subtype), (yield dut.tx_tp_b_nump), (yield dut.tx_tp_b_seq)))

        run_simulation(dut, generator(dut), clocks={"sys": 10})
        self.assertEqual(dut.status_acks[:1], [(LP_TP_SUB_ACK, 0, 1)])

if __name__ == "__main__":
    unittest.main()
