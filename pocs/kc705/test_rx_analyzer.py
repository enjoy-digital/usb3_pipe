#!/usr/bin/env python3

import sys
import time

from litex import RemoteClient
from litescope import LiteScopeAnalyzerDriver

from usb3_pipe.common import TSEQ, TS1

wb = RemoteClient()
wb.open()

# # #

TSEQ_FIRST_WORD = int.from_bytes(TSEQ.to_bytes()[0:4], byteorder="little")
TS1_FIRST_WORD  = int.from_bytes(TS1.to_bytes()[0:4], byteorder="little")
print("%08x" %TS1_FIRST_WORD)

# FPGA ID ------------------------------------------------------------------------------------------
fpga_id = ""
for i in range(256):
    c = chr(wb.read(wb.bases.identifier_mem + 4*i) & 0xff)
    fpga_id += c
    if c == "\0":
        break
print("FPGA: " + fpga_id)

# Enable Capture -----------------------------------------------------------------------------------
wb.regs.gtx_rx_polarity.write(1)
wb.regs.gtx_tx_enable.write(1)
while (wb.regs.gtx_tx_ready.read() == 0):
    pass
wb.regs.gtx_rx_enable.write(1)
while (wb.regs.gtx_rx_ready.read() == 0):
    pass

# Analyzer dump ------------------------------------------------------------------------------------
analyzer = LiteScopeAnalyzerDriver(wb.regs, "rx_analyzer", debug=True)
analyzer.configure_subsampler(1)
analyzer.configure_trigger(cond={
    "soc_gtx0_source_payload_ctrl": 0b0001,
    "soc_gtx0_source_payload_data": TSEQ_FIRST_WORD})
analyzer.configure_trigger(cond={
    "soc_gtx0_source_payload_ctrl": 0b1111,
    "soc_gtx0_source_payload_data": TS1_FIRST_WORD})
#analyzer.configure_trigger(cond={"soc_tseq_receiver_detected": 1})
#analyzer.configure_trigger(cond={"soc_ts1_receiver_detected": 1})
analyzer.run(offset=32, length=4096)
analyzer.wait_done()
analyzer.upload()
analyzer.save("analyzer.vcd")

# # #

wb.close()
