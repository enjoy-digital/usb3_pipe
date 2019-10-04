#!/usr/bin/env python3

import sys
import time

from litex import RemoteClient
from litescope import LiteScopeAnalyzerDriver

wb = RemoteClient()
wb.open()

# # #

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
analyzer = LiteScopeAnalyzerDriver(wb.regs, "fsm_analyzer", debug=True)
analyzer.configure_subsampler(1)
analyzer.configure_trigger(cond={"soc_ts2_det_sync_o": 0b1})
analyzer.run(offset=32, length=4096)
analyzer.wait_done()
analyzer.upload()
analyzer.save("analyzer.vcd")

# # #

wb.close()
