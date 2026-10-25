#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core example designs, shared by the targets (acorn.py, kc705.py, alibaba_xcku3p.py).

- loopback     : Vendor Bulk OUT (EP2) -> FIFO -> Bulk IN (EP1) (scripts/usb3_bulk.py).
- bench        : Vendor Bulk IN counter source, Bulk OUT counter checker (scripts/usb3_bench.py).
- cdc_uartbone : CDC-ACM serial port carrying UARTBone (litex_server --uart --uart-port /dev/ttyACMx).
- console      : CDC-ACM SoC console, BIOS (litex_term /dev/ttyACMx), requires a CPU.
- uvc          : UVC camera, YUY2 test pattern (ffplay/VLC /dev/videoX).
- uvc_iso      : UVC camera, YUY2 test pattern, Isochronous streaming (not yet validated on hardware).
- msc          : Mass Storage 512KB FAT RAM disk (/dev/sdX).
- msc_dram     : Mass Storage DRAM disk (upper half of the DRAM, /dev/sdX, mkfs.vfat needed), requires
                 a DRAM controller (soc.sdram) initialized by the BIOS (CPU).
- composite    : UVC camera + CDC-ACM SoC console in one device, requires a CPU.
"""

from migen import *

from litex.soc.cores.uart import UART, Stream2Wishbone
from litex.soc.interconnect import stream

from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import bulk_descriptors, cdc_acm_interfaces, add_cdc_acm_requests
from usb3_core.descriptors import USB3Descriptors, DEVICE_CLASS_IAD
from usb3_core.cdc_acm     import USB3CDCACM, USB3CDCACMFunction
from usb3_core.bench       import USB3Bench
from usb3_core.uvc         import USB3UVC, USB3UVCFunction, uvc_interfaces, UVC_MODES
from usb3_core.msc         import USB3MSC, USB3MSCDRAM, msc_fat_image

USB3_EXAMPLES = ["loopback", "bench", "cdc_uartbone", "console", "uvc", "uvc_iso", "msc", "msc_dram", "composite"]

# Helpers ------------------------------------------------------------------------------------------

def add_usb3_console(soc, usb3_acm):
    """SoC console (BIOS) on a USB3 CDC-ACM: litex_term /dev/ttyACMx."""
    soc.uart = UART(usb3_acm, tx_fifo_depth=64, rx_fifo_depth=64)
    if soc.irq.enabled:
        soc.irq.add("uart", use_loc_if_exists=True)
    else:
        soc.add_constant("UART_POLLING")

# USB3 Example -------------------------------------------------------------------------------------

def add_usb3_example(soc, usb3_phy, sys_clk_freq, example="loopback", with_lpm=False):
    """Add a USB3 Device Core example design to soc (on usb3_phy), with_lpm: U1/U2 Link Power
    Management (vendor Bulk examples)."""
    assert example in USB3_EXAMPLES
    if example in ["console", "composite"]:
        assert soc.cpu_type is not None, f"{example} example requires a CPU (--cpu-type)."
    if example in ["msc_dram"]:
        assert hasattr(soc, "sdram"), f"{example} example requires a DRAM controller (soc.sdram)."

    # Composite device: UVC camera (interfaces 0-1, EP1) + CDC-ACM SoC console (interfaces 2-3,
    # EP2-4), one USB3 device: ffplay /dev/videoX while using litex_term /dev/ttyACMx.
    if example == "composite":
        descriptors = USB3Descriptors(
            product      = "USB3-PIPE Camera + Console",
            device_class = DEVICE_CLASS_IAD,
            interfaces   = uvc_interfaces(UVC_MODES, vc_interface=0, ept=1, max_burst=16, clk_freq=sys_clk_freq) +
                cdc_acm_interfaces(comm_interface=2, notif_ept=4, in_ept=2, out_ept=3, max_burst=2))
        add_cdc_acm_requests(descriptors)
        soc.usb3_core = usb3_core = USB3DeviceCore(usb3_phy, sys_clk_freq, descriptors,
            endpoints = [
                USB3BulkIN(ept=1,  max_burst=16), # UVC video.
                USB3BulkIN(ept=2,  max_burst=2),  # CDC-ACM data IN.
                USB3BulkOUT(ept=3, max_burst=2),  # CDC-ACM data OUT.
                USB3BulkIN(ept=4,  max_burst=2),  # CDC-ACM notifications (unused).
            ],
            with_ep0_handler = True)
        usb3_core.add_csr(with_trace=True)
        ep0 = usb3_core.ep0
        soc.usb3_uvc = USB3UVCFunction(ep0, usb3_core.endpoints[0], usb3_core.link.reset, sys_clk_freq)
        soc.usb3_acm = usb3_acm = USB3CDCACMFunction(ep0, usb3_core.endpoints[1], usb3_core.endpoints[2])
        add_usb3_console(soc, usb3_acm)

    # Mass Storage RAM disk (512KB, FAT with a README): /dev/sdX.
    elif example == "msc":
        readme = "USB3-PIPE RAM disk: USB3.0 SuperSpeed Mass Storage from an FPGA (LiteX/usb3_pipe).\n"
        image  = msc_fat_image(512*1024, {"README.TXT": readme})
        soc.usb3_msc = usb3_msc = USB3MSC(usb3_phy, sys_clk_freq, size=512*1024, image=image)
        usb3_msc.core.add_csr(with_trace=True)

    # Mass Storage DRAM disk (upper half of the DRAM, not formatted: mkfs.vfat /dev/sdX): /dev/sdX.
    elif example == "msc_dram":
        rd_port   = soc.sdram.crossbar.get_port(mode="read")
        wr_port   = soc.sdram.crossbar.get_port(mode="write")
        dram_size = (2**rd_port.address_width)*(rd_port.data_width//8)
        # Ready once the DRAM is initialized by the BIOS: DFII back to hardware control after the
        # software control of the initialization/calibration, no initialization error reported
        # (ddrctrl, when available).
        dfii_sel = soc.sdram.dfii._control.fields.sel
        dfii_sw  = Signal() # DFII has been under software control.
        ready    = Signal()
        soc.sync += If(~dfii_sel, dfii_sw.eq(1))
        soc.comb += ready.eq(dfii_sw & dfii_sel)
        if hasattr(soc, "ddrctrl"):
            soc.comb += If(soc.ddrctrl.init_error.storage, ready.eq(0))
        usb3_msc_dram = USB3MSCDRAM(rd_port, wr_port,
            size  = dram_size//2,
            base  = dram_size//2, # Upper half: lower half left to the CPU (BIOS memtest, software).
            ready = ready)
        soc.usb3_msc = usb3_msc = USB3MSC(usb3_phy, sys_clk_freq,
            storage         = usb3_msc_dram,
            product         = "USB3-PIPE DRAM Disk",
            inquiry_product = "USB3-PIPE DRAM")
        usb3_msc.core.add_csr(with_trace=True)

    # UVC camera (test pattern): ffplay -f v4l2 -input_format yuyv422 -video_size 1280x720 /dev/videoX.
    elif example == "uvc":
        soc.usb3_uvc = usb3_uvc = USB3UVC(usb3_phy, sys_clk_freq)
        usb3_uvc.core.add_csr(with_trace=True)

    # UVC camera (test pattern), Isochronous streaming (16KB per 125us service interval, ~131MB/s):
    # ffplay -f v4l2 -input_format yuyv422 -video_size 1280x720 -framerate 60 /dev/videoX.
    elif example == "uvc_iso":
        soc.usb3_uvc = usb3_uvc = USB3UVC(usb3_phy, sys_clk_freq, transfer="iso", max_burst=16,
            with_debug=True) # EP0 IN responses debug CSRs (not yet validated on hardware).
        usb3_uvc.core.add_csr(with_trace=True)

    # USB3 CDC-ACM serial port: SoC console or Wishbone bridge.
    elif example in ["console", "cdc_uartbone"]:
        soc.usb3_acm = usb3_acm = USB3CDCACM(usb3_phy, sys_clk_freq)
        usb3_acm.core.add_csr(with_trace=True)
        if example == "console":
            add_usb3_console(soc, usb3_acm)
        if example == "cdc_uartbone":
            # Wishbone bridge over the USB3 CDC-ACM: litex_server --uart --uart-port /dev/ttyACMx.
            soc.usb3_uartbone = usb3_uartbone = Stream2Wishbone(usb3_acm, clk_freq=sys_clk_freq)
            soc.bus.add_master(name="usb3_uartbone", master=usb3_uartbone.wishbone)

    # Vendor Bulk examples.
    else:
        descriptors = bulk_descriptors(max_burst=8)
        endpoints   = [USB3BulkIN(ept=1, max_burst=8), USB3BulkOUT(ept=2, max_burst=8)]
        soc.usb3_core = usb3_core = USB3DeviceCore(usb3_phy, sys_clk_freq, descriptors, endpoints,
            with_lpm = with_lpm)
        usb3_core.add_csr()

        # Bulk loopback: EP2 OUT -> FIFO -> EP1 IN (scripts/usb3_bulk.py).
        if example == "loopback":
            soc.usb3_loopback = usb3_loopback = stream.SyncFIFO([("data", 32), ("be", 4)], 2*256, buffered=True)
            soc.comb += [
                usb3_core.endpoints[1].source.connect(usb3_loopback.sink),
                usb3_loopback.source.connect(usb3_core.endpoints[0].sink),
            ]

        # Bandwidth benchmark: EP1 IN counter source, EP2 OUT counter checker (scripts/usb3_bench.py).
        if example == "bench":
            soc.usb3_bench = usb3_bench = USB3Bench()
            soc.comb += [
                usb3_bench.source.connect(usb3_core.endpoints[0].sink),
                usb3_core.endpoints[1].source.connect(usb3_bench.sink),
            ]
