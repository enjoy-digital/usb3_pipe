#!/usr/bin/env python3

import sys
import time

from litex import RemoteClient
from litescope import LiteScopeAnalyzerDriver

wb = RemoteClient()
wb.open()

# # #

def help():
    print("Supported triggers:")
    print(" - rx_polling")
    print(" - tx_polling")
    print("")
    print(" - rx_tseq")
    print(" - rx_ts1")
    print(" - rx_ts2")
    print(" - tx_ts2")
    print("")
    print(" - now")
    exit()

if len(sys.argv) < 2:
    help()

if len(sys.argv) < 3:
    length = 4096
else:
    length = int(sys.argv[2])

# FPGA ID ------------------------------------------------------------------------------------------
fpga_id = ""
for i in range(256):
    c = chr(wb.read(wb.bases.identifier_mem + 4*i) & 0xff)
    fpga_id += c
    if c == "\0":
        break
print("FPGA: " + fpga_id)

# Analyzer dump ------------------------------------------------------------------------------------
analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
if sys.argv[1] == "rx_polling":
    analyzer.configure_trigger(cond={"soc_usb3_phy_lfps_rx_polling" : 1})
elif sys.argv[1] == "tx_polling":
    analyzer.configure_trigger(cond={"soc_usb3_phy_lfps_tx_polling" : 1})
elif sys.argv[1] == "rx_tseq":
    analyzer.configure_trigger(cond={"soc_usb3_phy_ts_rx_tseq" : 1})
elif sys.argv[1] == "rx_ts1":
     analyzer.configure_trigger(cond={"soc_usb3_phy_ts_rx_ts1" : 1})
elif sys.argv[1] == "rx_ts2":
     analyzer.configure_trigger(cond={"soc_usb3_phy_ts_rx_ts2" : 1})
elif sys.argv[1] == "tx_ts2":
     analyzer.configure_trigger(cond={"soc_usb3_phy_ts_tx_ts2" : 1})
elif sys.argv[1] == "now":
	analyzer.configure_trigger(cond={})
else:
	raise ValueError
analyzer.configure_trigger(cond={})
analyzer.run(offset=32, length=length)
analyzer.wait_done()
analyzer.upload()
analyzer.save("analyzer.vcd")

# # #

wb.close()
