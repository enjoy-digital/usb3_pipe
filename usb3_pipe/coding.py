#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 line coding (USB 3.2 6.3): 8b/10b (Gen 1, 6.3.1) Python model/code tables and RX checker."""

from migen import *

from litex.gen import *

from litex.soc.cores import code_8b10b

# 8b/10b Code Tables (6.3.1, Appendix A) -----------------------------------------------------------

# Valid K-codes (Table 6-2 / Appendix A).
K_CODES = [(28, 0), (28, 1), (28, 2), (28, 3), (28, 4), (28, 5), (28, 6), (28, 7),
           (23, 7), (27, 7), (29, 7), (30, 7)]

def encode_8b10b(d, k, rd):
    """Python model of LiteX's SingleEncoder (code_8b10b): returns the 10-bit code (MSB first,
    abcdei fghj) and the running disparity (0: RD-, 1: RD+) after the symbol."""
    t = code_8b10b
    code5b, code3b = d & 0x1f, d >> 5
    if k and code5b == 28:
        code6b, unb6, flip6 = 0b110000, True, True
    else:
        code6b, unb6, flip6 = t.table_5b6b[code5b], t.table_5b6b_unbalanced[code5b], t.table_5b6b_flip[code5b]
    code4b = t.table_3b4b[code3b]
    unb4   = t.table_3b4b_unbalanced[code3b]
    flip4  = True if k else t.table_3b4b_flip[code3b]
    alt7_rd0 = (code3b == 7) and ((code5b in [17, 18, 20]) or k)
    alt7_rd1 = (code3b == 7) and ((code5b in [11, 13, 14]) or k)
    disp_inter = rd ^ unb6
    out6 = (~code6b & 0x3f) if (not rd and flip6) else code6b
    if not disp_inter and alt7_rd0:
        rd_out, out4 = 1, 0b0111
    elif disp_inter and alt7_rd1:
        rd_out, out4 = 0, 0b1000
    else:
        rd_out = disp_inter ^ unb4
        out4   = (~code4b & 0xf) if (not disp_inter and flip4) else code4b
    return (out6 << 4) | out4, int(rd_out)

def code_8b10b_table(lsb_first=True):
    """1024 entries (indexed by the received 10-bit code): (valid with RD-, valid with RD+, RD after
    the symbol for RD-, RD after the symbol for RD+)."""
    table = [[0, 0, 0, 0] for _ in range(1024)]
    symbols = [(d, 0) for d in range(256)] + [(x | (y << 5), 1) for x, y in K_CODES]
    for d, k in symbols:
        for rd in [0, 1]:
            code, rd_out = encode_8b10b(d, k, rd)
            if lsb_first:
                code = int("{:010b}".format(code)[::-1], 2)
            table[code][rd]     = 1
            table[code][2 + rd] = rd_out
    return table

# RX 8b/10b Checker (6.3.1.2, 6.3.1.4) -------------------------------------------------------------

class RX8b10bChecker(LiteXModule):
    """RX 8b/10b Checker

    Checks the received 10-bit symbols (decoders inputs) against the valid 8b/10b codes, with the
    same latency as the decoders outputs:
    - invalid:         Symbol not matching any valid 8b/10b code (decode error, 6.3.1.4).
    - disparity_error: Valid code received in the wrong running disparity column (6.3.1.2).

    The running disparity is tracked over the symbols and re-initialized from the received code on
    errors (6.3.1.2: initial disparity is the disparity of the first received symbol).
    """
    def __init__(self, decoders, lsb_first=True):
        n = len(decoders)
        self.invalid         = Signal(n) # o
        self.disparity_error = Signal(n) # o

        # # #

        table = code_8b10b_table(lsb_first)
        rom   = Memory(4, 1024, init=[e[0] | (e[1] << 1) | (e[2] << 2) | (e[3] << 3) for e in table])
        self.specials += rom

        # Code lookup, registered (as the decoders outputs).
        lookups = []
        for decoder in decoders:
            port = rom.get_port(async_read=True)
            self.specials += port
            lookup = Signal(4)
            self.comb += port.adr.eq(decoder.input)
            self.sync += lookup.eq(port.dat_r)
            lookups.append(lookup)

        # Errors and running disparity.
        rd = Signal()
        rd_lane = rd
        for i, lookup in enumerate(lookups):
            valid_m, valid_p, rd_m, rd_p = [lookup[j] for j in range(4)]
            rd_next = Signal()
            self.comb += [
                self.invalid[i].eq(~valid_m & ~valid_p),
                self.disparity_error[i].eq(Mux(rd_lane, ~valid_p & valid_m, ~valid_m & valid_p)),
                If(rd_lane,
                    If(valid_p,  rd_next.eq(rd_p)).Elif(valid_m, rd_next.eq(rd_m)).Else(rd_next.eq(rd_lane))
                ).Else(
                    If(valid_m,  rd_next.eq(rd_m)).Elif(valid_p, rd_next.eq(rd_p)).Else(rd_next.eq(rd_lane))
                )
            ]
            rd_lane = rd_next
        self.sync += rd.eq(rd_lane)
