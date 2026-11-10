#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 GUD (Generic USB Display) function: a USB display for the Linux gud driver
(drivers/gpu/drm/gud, include/drm/gud.h), device VID:PID 1d50:614d (gud driver match).

Fixed display, all the requests answered from the Control Endpoint requests table (no request
handler):
- one mode (preferred), one connector (HDMI, connected), one pixel format, no EDID/properties;
- GUD_DISPLAY_FLAG_FULL_UPDATE: the host sends full frames (frame_size bytes) on the Bulk OUT
  endpoint, SET_BUFFER only after a failed transfer (resynchronization), no compression.

USB3GUDFunction: frame data stream from the Bulk OUT endpoint (source) with a start pulse on
SET_BUFFER (next byte: first byte of a frame): to a framebuffer DMA writer (frame bytes in order,
host pixel format: ex RGB565 little-endian, as the LiteX VideoFrameBuffer rgb565).
"""

import struct

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common      import endpoint_layout
from usb3_core.descriptors import USB3InterfaceDescriptor, USB3EndpointDescriptor

# Constants (include/drm/gud.h) --------------------------------------------------------------------

GUD_VID = 0x1d50
GUD_PID = 0x614d

GUD_DISPLAY_MAGIC              = 0x1d50614d
GUD_DISPLAY_FLAG_STATUS_ON_SET = (1 << 0)
GUD_DISPLAY_FLAG_FULL_UPDATE   = (1 << 1)

GUD_DISPLAY_MODE_FLAG_PHSYNC    = (1 << 0)
GUD_DISPLAY_MODE_FLAG_NHSYNC    = (1 << 1)
GUD_DISPLAY_MODE_FLAG_PVSYNC    = (1 << 2)
GUD_DISPLAY_MODE_FLAG_NVSYNC    = (1 << 3)
GUD_DISPLAY_MODE_FLAG_PREFERRED = (1 << 10)

GUD_CONNECTOR_TYPE_HDMI        = 7
GUD_CONNECTOR_STATUS_CONNECTED = 0x01

GUD_STATUS_OK = 0x00

GUD_PIXEL_FORMAT_RGB565   = 0x40
GUD_PIXEL_FORMAT_XRGB8888 = 0x80

GUD_REQ_GET_STATUS                   = 0x00
GUD_REQ_GET_DESCRIPTOR               = 0x01
GUD_REQ_GET_FORMATS                  = 0x40
GUD_REQ_GET_PROPERTIES               = 0x41
GUD_REQ_GET_CONNECTORS               = 0x50
GUD_REQ_GET_CONNECTOR_PROPERTIES     = 0x51
GUD_REQ_GET_CONNECTOR_TV_MODE_VALUES = 0x52
GUD_REQ_SET_CONNECTOR_FORCE_DETECT   = 0x53
GUD_REQ_GET_CONNECTOR_STATUS         = 0x54
GUD_REQ_GET_CONNECTOR_MODES          = 0x55
GUD_REQ_GET_CONNECTOR_EDID           = 0x56
GUD_REQ_SET_BUFFER                   = 0x60
GUD_REQ_SET_STATE_CHECK              = 0x61
GUD_REQ_SET_STATE_COMMIT             = 0x62
GUD_REQ_SET_CONTROLLER_ENABLE        = 0x63
GUD_REQ_SET_DISPLAY_ENABLE           = 0x64

GUD_REQ_TYPE_IN  = 0xc1 # Vendor, Interface, Device to Host.
GUD_REQ_TYPE_OUT = 0x41 # Vendor, Interface, Host to Device.

GUD_FORMATS_BPP = {
    GUD_PIXEL_FORMAT_RGB565   : 2,
    GUD_PIXEL_FORMAT_XRGB8888 : 4,
}

# Mode ---------------------------------------------------------------------------------------------

def gud_mode(video_timings, flags=GUD_DISPLAY_MODE_FLAG_PHSYNC | GUD_DISPLAY_MODE_FLAG_PVSYNC):
    """struct gud_display_mode_req (preferred) from LiteX video timings (dict: pix_clk, h_active,
    h_blanking, h_sync_offset, h_sync_width, v_...)."""
    vt = video_timings
    h, v = vt["h_active"], vt["v_active"]
    return struct.pack("<IHHHHHHHHI",
        int(vt["pix_clk"]//1000),
        h, h + vt["h_sync_offset"], h + vt["h_sync_offset"] + vt["h_sync_width"],
        h + vt["h_blanking"],
        v, v + vt["v_sync_offset"], v + vt["v_sync_offset"] + vt["v_sync_width"],
        v + vt["v_blanking"],
        flags | GUD_DISPLAY_MODE_FLAG_PREFERRED)

# Descriptors --------------------------------------------------------------------------------------

def gud_interfaces(interface=0, out_ept=1, max_burst=16):
    """GUD interface: vendor class, Bulk OUT endpoint (frames)."""
    return [USB3InterfaceDescriptor(interface, cls=0xff, endpoints=[
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])]

def add_gud_requests(descriptors, video_timings, pixel_format=GUD_PIXEL_FORMAT_RGB565):
    """GUD requests (constant responses): fixed mode/connector/format, full frame updates."""
    width, height = video_timings["h_active"], video_timings["v_active"]
    frame_size    = width*height*GUD_FORMATS_BPP[pixel_format]
    display_descriptor = struct.pack("<IBIBIIIII",
        GUD_DISPLAY_MAGIC,
        1,                            # Version.
        GUD_DISPLAY_FLAG_FULL_UPDATE, # Flags.
        0,                            # Compression.
        frame_size,                   # Max buffer size: one frame.
        width, width, height, height) # Min/max width/height.
    connector_descriptor = struct.pack("<BI", GUD_CONNECTOR_TYPE_HDMI, 0)
    responses = {
        GUD_REQ_GET_STATUS               : [GUD_STATUS_OK],
        GUD_REQ_GET_DESCRIPTOR           : list(display_descriptor),
        GUD_REQ_GET_FORMATS              : [pixel_format],
        GUD_REQ_GET_PROPERTIES           : [],
        GUD_REQ_GET_CONNECTORS           : list(connector_descriptor),
        GUD_REQ_GET_CONNECTOR_PROPERTIES : [],
        GUD_REQ_GET_CONNECTOR_STATUS     : [GUD_CONNECTOR_STATUS_CONNECTED],
        GUD_REQ_GET_CONNECTOR_MODES      : list(gud_mode(video_timings)),
        GUD_REQ_GET_CONNECTOR_EDID       : [],
    }
    for request, response in responses.items():
        descriptors.add_class_request(GUD_REQ_TYPE_IN, request, response)
    for request in [GUD_REQ_SET_CONNECTOR_FORCE_DETECT, GUD_REQ_SET_BUFFER,
        GUD_REQ_SET_STATE_CHECK, GUD_REQ_SET_STATE_COMMIT, GUD_REQ_SET_CONTROLLER_ENABLE,
        GUD_REQ_SET_DISPLAY_ENABLE]:
        descriptors.add_class_request(GUD_REQ_TYPE_OUT, request)
    return frame_size

# USB3 GUD Function --------------------------------------------------------------------------------

class USB3GUDFunction(LiteXModule):
    """GUD function of a USB3 device (EP0 requests from the table, Bulk OUT frames endpoint).

    source: frames data (Bulk OUT endpoint stream, data_width of the endpoint).
    start : pulse on SET_BUFFER (frames resynchronization: the next data is the start of a frame).
    """
    def __init__(self, ep0, bulk_out):
        self.source = source = stream.Endpoint(endpoint_layout(bulk_out.data_width)) # o
        self.start  = start  = Signal()                                              # o

        # # #

        self.comb += [
            bulk_out.source.connect(source),
            start.eq(ep0.req_done & (ep0.req_type == GUD_REQ_TYPE_OUT) &
                (ep0.req_request == GUD_REQ_SET_BUFFER)),
        ]
