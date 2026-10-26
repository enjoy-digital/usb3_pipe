#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab device constants: the host side of the host/gateware contract (doc/agentlab.md).

Everything that must match the gateware (USB IDs, composite layout, events record format, CSR
names) lives here and only here.
"""

# USB IDs ------------------------------------------------------------------------------------------

USB_VID = 0x1d50
USB_PID = 0x605a

# USB Composite Layout -----------------------------------------------------------------------------

# Control CDC-ACM (UARTBone): communication interface number (data interface is +1).
CONTROL_INTERFACE = 0

# DUT UARTs CDC-ACM: communication interface number of each DUT UART (data interface is +1).
DUT_UART_INTERFACES = [2, 4, 6, 8]
DUT_UART_COUNT      = len(DUT_UART_INTERFACES)

# Events (Vendor class, bulk IN).
EVENTS_INTERFACE = 10
EVENTS_ENDPOINT  = 0x8b # EP11 IN.

# Logic analyzer (Vendor class, bulk IN).
LA_INTERFACE = 11
LA_ENDPOINT  = 0x8c # EP12 IN.

# USB interface classes.
USB_CLASS_CDC        = 0x02
USB_CLASS_CDC_DATA   = 0x0a
USB_CLASS_VENDOR     = 0xff

# Timebase -----------------------------------------------------------------------------------------

DEFAULT_SYS_CLK_FREQ = 125e6 # Used when the CSR map does not provide config_clock_frequency.

# Events Records -----------------------------------------------------------------------------------

EVENT_RECORD_SIZE = 16 # Bytes, little-endian: type(1), source(1), value(6), timestamp(8).

EVENT_UART_RX    = 0x01
EVENT_UART_TX    = 0x02
EVENT_GPIO_EDGE  = 0x03
EVENT_POWER      = 0x04
EVENT_LA_TRIGGER = 0x05
EVENT_MARKER     = 0x06
EVENT_OVERFLOW   = 0xff

EVENT_NAMES = {
    EVENT_UART_RX    : "uart_rx",
    EVENT_UART_TX    : "uart_tx",
    EVENT_GPIO_EDGE  : "gpio",
    EVENT_POWER      : "power",
    EVENT_LA_TRIGGER : "la_trigger",
    EVENT_MARKER     : "marker",
    EVENT_OVERFLOW   : "overflow",
}
EVENT_TYPES = {name: code for code, name in EVENT_NAMES.items()}

# CSRs ---------------------------------------------------------------------------------------------

CSR_TIMESTAMP       = "agentlab_timestamp_value" # 64-bit, read (after the latch).
CSR_TIMESTAMP_LATCH = "agentlab_timestamp_latch" # Written (1) before reading the timestamp.
CSR_MARKER          = "agentlab_marker"          # Write: emits a marker event (value = written).
CSR_POWER_OUT       = "agentlab_power_out"       # Bit per power switch (1: on).
CSR_POWER_STATUS    = "agentlab_power_status"    # Optional: bit per power switch (sense).
CSR_GPIO_OE         = "agentlab_gpio_oe"         # Bit per GPIO (1: output).
CSR_GPIO_OUT        = "agentlab_gpio_out"        # Bit per GPIO (output level).
CSR_GPIO_IN         = "agentlab_gpio_in"         # Bit per GPIO (input level).
CSR_GPIO_EDGE_MASK  = "agentlab_gpio_edge_mask"  # Bit per GPIO (1: emit edge events).

def csr_uart_baudrate(n):
    """DUT UART n baudrate CSR (fallback when SET_LINE_CODING is not used)."""
    return "agentlab_uart{}_baudrate".format(n)

# Logic Analyzer -----------------------------------------------------------------------------------

# Stream format (agentlab/gateware/la.py, doc/agentlab.md): header then blocks, 32-bit LE words.
LA_MAGIC           = 0x31414c41 # "ALA1".
LA_VERSION         = 1
LA_HEADER_WORDS    = 12
LA_HEADER_OVERFLOW = 0x01       # Header flags.
LA_BLOCK_MAGIC     = 0xb1       # Block word: count (bits 0-15), flags (16-23), magic (24-31).
LA_BLOCK_LAST      = 0x01
LA_BLOCK_OVERFLOW  = 0x02

LA_MODES = {"capture": 0, "stream": 1}

LA_TRIGGER_IMMEDIATE = 0
LA_TRIGGER_PATTERN   = 1
LA_TRIGGER_EXTERNAL  = 2
LA_EDGES             = {"rising": 0, "falling": 1, "edge": 2}
LA_EXT_SOURCES       = {"marker": 0, "gpio": 1, "power": 2} # Bits of CSR_LA_TRIGGER_EXT.

LA_STATES = {0: "idle", 1: "armed", 2: "triggered", 3: "readout", 4: "done", 5: "flush"}

CSR_LA_MODE              = "agentlab_la_mode"              # 0: capture, 1: stream.
CSR_LA_DIVIDER           = "agentlab_la_divider"           # samplerate = sys_clk/divider.
CSR_LA_TRIGGER_SOURCE    = "agentlab_la_trigger_source"    # 0: immediate, 1: pattern, 2: ext.
CSR_LA_TRIGGER_MASK      = "agentlab_la_trigger_mask"
CSR_LA_TRIGGER_VALUE     = "agentlab_la_trigger_value"
CSR_LA_TRIGGER_EDGE      = "agentlab_la_trigger_edge"      # enable(0), channel(1-5), polarity(6-7).
CSR_LA_TRIGGER_EXT       = "agentlab_la_trigger_ext"       # Bits: LA_EXT_SOURCES.
CSR_LA_PRE_TRIGGER       = "agentlab_la_pre_trigger"       # Samples before the trigger.
CSR_LA_LENGTH            = "agentlab_la_length"            # Samples (stream mode: 0, until stop).
CSR_LA_CONTROL           = "agentlab_la_control"           # arm(0), stop(1) (pulses).
CSR_LA_STATUS            = "agentlab_la_status"            # state(0-2), triggered(3),
                                                           # overflow(4), dram_ready(5).
CSR_LA_LEVEL             = "agentlab_la_level"             # DRAM words buffered.
CSR_LA_COUNT             = "agentlab_la_count"             # Samples captured.
CSR_LA_TRIGGER_OFFSET    = "agentlab_la_trigger_offset"
CSR_LA_START_TIMESTAMP   = "agentlab_la_start_timestamp"
CSR_LA_TRIGGER_TIMESTAMP = "agentlab_la_trigger_timestamp"

# Constants (csr.csv).
CONST_LA_CHANNELS     = "agentlab_la_channels"     # Number of channels (sample bits).
CONST_LA_RING_SAMPLES = "agentlab_la_ring_samples"
