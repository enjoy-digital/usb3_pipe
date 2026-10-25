#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab device discovery (Linux sysfs) and CDC-ACM interface to /dev/ttyACMx mapping.

sysfs layout: /sys/bus/usb/devices/<dev>/idVendor, ... and the CDC-ACM ttys under
/sys/bus/usb/devices/<dev>:<cfg>.<intf>/tty/ttyACMx.
"""

import os
import glob
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from agentlab.host import constants as c

SYSFS_USB_DEVICES = "/sys/bus/usb/devices"

# Device Info --------------------------------------------------------------------------------------

@dataclass
class DeviceInfo:
    sysfs_name : str                 # e.g. "2-1".
    busnum     : int
    devnum     : int
    serial     : Optional[str]  = None
    product    : Optional[str]  = None
    speed      : Optional[str]  = None # Mbps, as reported by sysfs ("5000" for SuperSpeed).
    ttys       : Dict[int, str] = field(default_factory=dict) # Interface number -> /dev/ttyACMx.

    @property
    def control_tty(self) -> Optional[str]:
        return self.ttys.get(c.CONTROL_INTERFACE, None)

    def uart_tty(self, index) -> Optional[str]:
        if not (0 <= index < c.DUT_UART_COUNT):
            raise IndexError("DUT UART index {} out of range (0-{}).".format(
                index, c.DUT_UART_COUNT - 1))
        return self.ttys.get(c.DUT_UART_INTERFACES[index], None)

    def describe(self) -> Dict[str, object]:
        return {
            "sysfs"   : self.sysfs_name,
            "bus"     : self.busnum,
            "device"  : self.devnum,
            "serial"  : self.serial,
            "product" : self.product,
            "speed"   : self.speed,
            "control" : self.control_tty,
            "uarts"   : [self.uart_tty(i) for i in range(c.DUT_UART_COUNT)],
        }

# Discovery ----------------------------------------------------------------------------------------

def _read(path) -> Optional[str]:
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except OSError:
        return None

def _tty_map(sysfs_root, name) -> Dict[int, str]:
    ttys = {}
    for intf_dir in glob.glob(os.path.join(sysfs_root, name + ":*.*")):
        try:
            intf = int(intf_dir.rsplit(".", 1)[1])
        except ValueError:
            continue
        for tty in glob.glob(os.path.join(intf_dir, "tty", "tty*")):
            ttys[intf] = "/dev/" + os.path.basename(tty)
    return ttys

def _intf_class(sysfs_root, name, intf) -> Optional[int]:
    for intf_dir in glob.glob(os.path.join(sysfs_root, "{}:*.{}".format(name, intf))):
        value = _read(os.path.join(intf_dir, "bInterfaceClass"))
        if value is not None:
            return int(value, 16)
    return None

def has_agentlab_layout(sysfs_root, name) -> bool:
    """Check the composite layout (other USB3-PIPE designs share the same VID/PID)."""
    layout = {
        c.CONTROL_INTERFACE        : c.USB_CLASS_CDC,
        c.DUT_UART_INTERFACES[0]   : c.USB_CLASS_CDC,
        c.EVENTS_INTERFACE         : c.USB_CLASS_VENDOR,
    }
    return all(_intf_class(sysfs_root, name, i) == cls for i, cls in layout.items())

def find_devices(vid=c.USB_VID, pid=c.USB_PID, sysfs_root=SYSFS_USB_DEVICES,
    check_layout=True) -> List[DeviceInfo]:
    """Return the AgentLab devices found in sysfs (sorted by sysfs name).

    With check_layout, devices with the VID/PID but another interface layout are ignored.
    """
    devices = []
    for path in sorted(glob.glob(os.path.join(sysfs_root, "*"))):
        name = os.path.basename(path)
        if ":" in name:
            continue # Interface, not a device.
        if _read(os.path.join(path, "idVendor")) != "{:04x}".format(vid):
            continue
        if _read(os.path.join(path, "idProduct")) != "{:04x}".format(pid):
            continue
        if check_layout and not has_agentlab_layout(sysfs_root, name):
            continue
        devices.append(DeviceInfo(
            sysfs_name = name,
            busnum     = int(_read(os.path.join(path, "busnum")) or 0),
            devnum     = int(_read(os.path.join(path, "devnum")) or 0),
            serial     = _read(os.path.join(path, "serial")),
            product    = _read(os.path.join(path, "product")),
            speed      = _read(os.path.join(path, "speed")),
            ttys       = _tty_map(sysfs_root, name),
        ))
    return devices

def find_device(serial=None, **kwargs) -> DeviceInfo:
    """Return the AgentLab device (selected by serial number when several are connected)."""
    devices = find_devices(**kwargs)
    if serial is not None:
        devices = [d for d in devices if d.serial == serial]
    if not devices:
        raise RuntimeError("No AgentLab device found ({:04x}:{:04x}{}).".format(
            c.USB_VID, c.USB_PID, "" if serial is None else ", serial " + serial))
    if len(devices) > 1:
        raise RuntimeError("Several AgentLab devices found, select one by serial: {}.".format(
            ", ".join(str(d.serial) for d in devices)))
    return devices[0]

def find_usb_device(info: DeviceInfo):
    """Return the pyusb device matching a DeviceInfo (for the vendor bulk interfaces)."""
    import usb.core
    dev = usb.core.find(idVendor=c.USB_VID, idProduct=c.USB_PID,
        custom_match=lambda d: d.bus == info.busnum and d.address == info.devnum)
    if dev is None:
        raise RuntimeError("AgentLab device {} not found with pyusb.".format(info.sysfs_name))
    return dev
