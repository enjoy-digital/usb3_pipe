#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PHY common definitions: Gen 1 symbols (6.3.5), Ordered Sets (TS1/TS2/TSEQ), Link
Configuration and Gen 2 Block Headers (6.3.2.2), Symbols/Ordered Sets identifiers and payloads
(6.4.1.2, 6.4.3.3).
"""

from migen import *

from litex.gen import *

# Helpers ------------------------------------------------------------------------------------------

def K(x, y):
    """K code generator ex: K(28, 5) is COM Symbol"""
    return (y << 5) | x

def D(x, y):
    """D code generator"""
    return (y << 5) | x

def LinkConfig(reset=0, loopback=0, scrambling=1):
    """Link Configuration of TS1/TS2 Ordered Sets."""
    value  = (      reset   << 0)
    value |= (   loopback   << 2)
    value |= ((not scrambling) << 3)
    return value

# Symbols (6.3.5) ----------------------------------------------------------------------------------

class Symbol:
    """Symbol definition with name, 8-bit value and description"""
    def __init__(self, name, value, description=""):
        self.name        = name
        self.value       = value
        self.description = description

SKP =  Symbol("SKP", K(28, 1), "Skip")
SDP =  Symbol("SDP", K(28, 2), "Start Data Packet")
EDB =  Symbol("EDB", K(28, 3), "End Bad")
SUB =  Symbol("SUB", K(28, 4), "Decode Error Substitution")
COM =  Symbol("COM", K(28, 5), "Comma")
RSD =  Symbol("RSD", K(28, 6), "Reserved")
SHP =  Symbol("SHP", K(27, 7), "Start Header Packet")
END =  Symbol("END", K(29, 7), "End")
SLC =  Symbol("SLC", K(30, 7), "Start Link Command")
EPF =  Symbol("EPF", K(23, 7), "End Packet Framing")

# Training Sequence Ordered Sets (6.4.1.2) ---------------------------------------------------------

class OrderedSet(list):
    """Ordered Set definition with name, 8-bit values and description"""
    def __init__(self, name, values, description=""):
        self.name        = name
        self.description = description
        list.__init__(self, values)

    @property
    def has_link_config(self):
        """TS1/TS2 Ordered Sets carry the Link Configuration (Symbol 5, Table 6-4/6-5)."""
        return self.name in ["TS1", "TS1_INV", "TS2"]

    def to_bytes(self):
        r = bytes()
        for e in self:
            if isinstance(e, Symbol):
                r += bytes([e.value])
            else:
                r += bytes([e])
        return r

TSEQ = OrderedSet("TSEQ",
    [COM,      D(31, 7), D(23, 0), D( 0, 6)] +
    [D(20, 0), D(18, 5), D( 7, 7), D( 2, 0)] +
    [D( 2, 4), D(18, 3), D(14, 3), D( 8, 1)] +
    [D( 6, 5), D(30, 5), D(13, 3), D(31, 5)] +
    [D(10, 2) for i in range(16)])

# TSEQ (32 bytes):
# [0] : 0xc017ffbc
# [1] : 0x02e7b214
# [2] : 0x286e7282
# [3] : 0xbf6dbea6
# [4] : 0x4a4a4a4a
# [5] : 0x4a4a4a4a
# [6] : 0x4a4a4a4a
# [7] : 0x4a4a4a4a

TS1 = OrderedSet("TS1",
    [COM for i in range(4)] +
    [D( 0, 0), LinkConfig(reset=0, loopback=0, scrambling=1)] +
    [D(10, 2) for i in range(10)])

# TS1 (16 bytes):
# [0] : 0xbcbcbcbc
# [1] : 0x4a4a0000
# [2] : 0x4a4a4a4a
# [3] : 0x4a4a4a4a

TS1_INV = OrderedSet("TS1_INV",
    [COM for i in range(4)] +
    [D( 0, 0), LinkConfig(reset=0, loopback=0, scrambling=1)] +
    [D(21, 5) for i in range(10)])

# TS1_INV (16 bytes):
# [0] : 0xbcbcbcbc
# [1] : 0xb5b50000
# [2] : 0xb5b5b5b5
# [3] : 0xb5b5b5b5

TS2 = OrderedSet("TS2",
    [COM for i in range(4)] +
    [D( 0, 0), LinkConfig(reset=0, loopback=0, scrambling=1)] +
    [D(5, 2) for i in range(10)])

# TS2 (16 bytes):
# [0] : 0xbcbcbcbc
# [1] : 0x45450000
# [2] : 0x45454545
# [3] : 0x45454545

# Endianness Swap ----------------------------------------------------------------------------------

class EndiannessSwap(LiteXModule):
    """Swap the data bytes/ctrl bits of stream"""
    def __init__(self, sink, source):
        assert len(sink.data) == len(source.data)
        assert len(sink.ctrl) == len(source.ctrl)
        self.comb += sink.connect(source, omit={"data", "ctrl"})
        n = len(sink.ctrl)
        for i in range(n):
            self.comb += source.data[8*i:8*(i+1)].eq(sink.data[8*(n-1-i):8*(n-1-i+1)])
            self.comb += source.ctrl[i].eq(sink.ctrl[n-1-i])

# LTSSM <-> PIPE Interface -------------------------------------------------------------------------

# Signals between the USB3LTSSM and the USB3PIPE (LFPS, Training Sequences, Idle Symbol handshake,
# SerDes/Datapath controls): same name on both sides, declared by both from this list and connected
# by USB3PHY (connect_ltssm_pipe). (name, width, direction from the LTSSM: "o": LTSSM -> PIPE, "i":
# PIPE -> LTSSM).
LTSSM_PIPE_INTERFACE = [
    # LFPS (6.9).
    ("lfps_tx_polling",      1, "o"), # Polling.LFPS bursts transmission.
    ("lfps_tx_idle",         1, "o"), # Electrical idle (when no LFPS burst).
    ("lfps_tx_ping",         1, "o"), # Ping.LFPS transmission (U1).
    ("lfps_tx_exit_u1",      1, "o"), # U1 exit LFPS transmission.
    ("lfps_tx_exit_u2",      1, "o"), # U2/Loopback exit LFPS transmission.
    ("lfps_tx_wakeup_u3",    1, "o"), # U3 wakeup LFPS transmission.
    ("lfps_tx_reset",        1, "o"), # Warm Reset LFPS transmission.
    ("lfps_tx_count",       16, "i"), # Polling.LFPS bursts transmitted.
    ("lfps_rx_polling",      1, "i"), # Polling.LFPS received.
    ("lfps_rx_ping",         1, "i"), # Ping.LFPS received.
    ("lfps_rx_exit_u1",      1, "i"), # U1 exit LFPS received.
    ("lfps_rx_exit_u2",      1, "i"), # U2/Loopback exit LFPS received.
    ("lfps_rx_wakeup_u3",    1, "i"), # U3 wakeup LFPS received.
    ("lfps_rx_reset",        1, "i"), # Warm Reset LFPS received.

    # Training Sequences (6.4.1, 7.5.4).
    ("ts_tx_enable",         1, "o"), # Training Sequences transmission.
    ("ts_tx_tseq",           1, "o"), # TSEQ transmission.
    ("ts_tx_ts1",            1, "o"), # TS1 transmission.
    ("ts_tx_ts2",            1, "o"), # TS2 transmission.
    ("ts_tx_ts2_reset",      1, "o"), # Reset bit of the transmitted TS2 (Hot Reset, 7.4.2).
    ("ts_tx_ts2_loopback",   1, "o"), # Loopback bit of the transmitted TS2 (7.5.4.9.1).
    ("ts_tx_done",           1, "i"), # Training Sequences burst transmitted.
    ("ts_rx_ts1",            1, "i"), # TS1 received.
    ("ts_rx_ts1_inv",        1, "i"), # Inverted TS1 received (polarity).
    ("ts_rx_ts2",            1, "i"), # TS2 received.
    ("ts_rx_ts2_reset",      1, "i"), # Reset bit of the received TS2 (Hot Reset).
    ("ts_rx_ts2_loopback",   1, "i"), # Loopback bit of the received TS2 (Loopback slave).
    ("ts2_decode",           1, "o"), # Decode the received TS2 Disable Scrambling bit.

    # Idle Symbol handshake (7.5.4.10.2, 7.5.10.4.2, 7.5.12.4.2).
    ("idle_handshake",       1, "o"), # Idle Symbols transmission and handshake.
    ("idle_handshake_done",  1, "i"), # 8 Idle Symbols received and 16 sent after the first one.

    # SerDes/Datapath.
    ("serdes_rx_align",      1, "o"), # RX word alignment (re-)acquisition.
    ("serdes_rx_polarity",   1, "o"), # RX polarity inversion (7.5.4.5).
    ("rx_ready",             1, "o"), # RX data to the Link Layer (U0).
    ("tx_ready",             1, "o"), # TX data from the Link Layer (U0).
]

# Gen 2 Block Headers (6.3.2.2) --------------------------------------------------------------------

BLOCK_HEADER_DATA    = 0b0011
BLOCK_HEADER_CONTROL = 0b1100

# Gen 2 Symbols/Ordered Sets identifiers (Table 6-2, Tables 6-7 to 6-11, 6-13) ---------------------

GEN2_SKP    = 0xcc
GEN2_SKPEND = 0x33
GEN2_SDS    = 0xe1
GEN2_TS1    = 0x1e
GEN2_TS2    = 0x2d
GEN2_TSEQ   = 0x87
GEN2_IDLE   = 0x5a # Idle Symbol in Gen 2 operation (Logical Idle, 7.2.3, Table 7-6).

# Gen 2 framing Symbols (Table 6-2: scrambled, in Data Blocks only).
GEN2_SHP  = 0x9a # Start Header Packet.
GEN2_DPHP = 0x95 # Start Data Packet Header (non-deferred DPH, Gen 2 only).
GEN2_SDP  = 0x96 # Start Data Packet Payload.
GEN2_EDB  = 0x69 # End Bad.
GEN2_END  = 0x65 # End.
GEN2_SLC  = 0x4b # Start Link Command.
GEN2_EPF  = 0x36 # End Packet Framing.

GEN2_SKP_TX_SYMBOLS = 20 # SKP Symbols in a transmitted SKP Ordered Set (24 Symbols, 6.4.3.3).

# Gen 2 Ordered Sets payloads (16 Symbols, Symbol 0 first) -----------------------------------------

def gen2_ts_payload(identifier, link_config=0):
    """TS1/TS2 (Tables 6-7/6-8) or TSEQ (Table 6-9) payload, Symbols 14/15 = identifier."""
    payload = [identifier]*16
    payload[4] = 0x00
    payload[5] = link_config if identifier in [GEN2_TS1, GEN2_TS2] else 0x00
    return payload

def gen2_sync_payload():
    """SYNC Ordered Set (Table 6-10)."""
    return [0x00, 0xff]*8

def gen2_sds_payload():
    """SDS Ordered Set (Table 6-11)."""
    return [GEN2_SDS]*4 + [0x55]*12

def gen2_skp_lfsr_symbols(lfsr):
    """SKP Ordered Set LFSR Symbols (Table 6-13): Bit[7] = ~LFSR[22], Bit[6:0] = LFSR[22:16],
    LFSR[15:8], LFSR[7:0]."""
    return [(((~lfsr >> 22) & 0x1) << 7) | ((lfsr >> 16) & 0x7f), (lfsr >> 8) & 0xff, lfsr & 0xff]

def gen2_skp_payload(lfsr, n_skp=GEN2_SKP_TX_SYMBOLS):
    """SKP Ordered Set (Table 6-13): n_skp SKP (multiple of 4), SKPEND, LFSR state."""
    assert n_skp % 4 == 0
    return [GEN2_SKP]*n_skp + [GEN2_SKPEND] + gen2_skp_lfsr_symbols(lfsr)

def symbols_to_int(symbols):
    return sum(s << (8*i) for i, s in enumerate(symbols))

GEN2_SYNC_BITS = BLOCK_HEADER_CONTROL | (symbols_to_int(gen2_sync_payload()) << 4) # 132-bit.
