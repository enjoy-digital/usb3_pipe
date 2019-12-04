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
    print(" - rx_tseq_first_word")
    print(" - rx_tseq")
    print(" - rx_ts1")
    print(" - rx_ts2")
    print(" - tx_ts2")
    print("")
    print(" - ready")
    print(" - skip")
    print("")
    print(" - sink_ready")
    print(" - source_valid")
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
analyzer  = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
usb3_name = "usb3"
for s in analyzer.layouts[0]:
    if "soc_usb3" in s[0]:
        usb3_name = "soc_usb3"
if sys.argv[1] == "rx_polling":
    analyzer.add_rising_edge_trigger(usb3_name + "_pipe_rx_polling")
elif sys.argv[1] == "tx_polling":
    analyzer.add_rising_edge_trigger(usb3_name + "_pipe_tx_polling")
elif sys.argv[1] == "rx_tseq_first_word":
    from usb3_pipe.common import TSEQ
    TSEQ_FIRST_WORD = int.from_bytes(TSEQ.to_bytes()[0:4], byteorder="little")
    analyzer.configure_trigger(cond={
        usb3_name + "_serdes_source_source_valid" :       1,
        usb3_name + "_serdes_source_source_payload_data": TSEQ_FIRST_WORD})
elif sys.argv[1] == "rx_tseq":
    analyzer.add_rising_edge_trigger(usb3_name + "_pipe_rx_tseq")
elif sys.argv[1] == "rx_ts1":
     analyzer.add_rising_edge_trigger(usb3_name + "_pipe_rx_ts1")
elif sys.argv[1] == "rx_ts2":
     analyzer.add_rising_edge_trigger(usb3_name + "_pipe_rx_ts2")
elif sys.argv[1] == "tx_ts2":
     analyzer.add_rising_edge_trigger(usb3_name + "_pipe_tx_ts2")
elif sys.argv[1] == "ready":
     analyzer.add_rising_edge_trigger(usb3_name + "_pipe_ready")
elif sys.argv[1] == "skip":
     analyzer.add_rising_edge_trigger(usb3_name + "_serdes_rx_skip")
elif sys.argv[1] == "sink_ready":
     analyzer.add_rising_edge_trigger(usb3_name + "_pipe_sink_ready")
elif sys.argv[1] == "source_valid":
     analyzer.add_rising_edge_trigger(usb3_name + "_pipe_source_valid")
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
