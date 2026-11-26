#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from usb3_core.descriptors import bulk_descriptors

class TestDescriptors(unittest.TestCase):
    def ssp_sublink_speeds(self, descriptors):
        """(LSE, LSM) of each Sublink Speed Attribute of the SuperSpeedPlus USB Device Capability."""
        bos = descriptors.bos()
        i   = bos[0]
        while i < len(bos):
            length, dtype, cap = bos[i], bos[i + 1], bos[i + 2]
            if (dtype, cap) == (0x10, 0x0a):
                attrs = bos[i + 12:i + length]
                words = [int.from_bytes(bytes(attrs[k:k + 4]), "little") for k in range(0, len(attrs), 4)]
                return [((w >> 4) & 0x3, w >> 16) for w in words]
            i += length
        return None

    def test_bos_ssp_lane_speed(self):
        # SuperSpeedPlus USB Device Capability: lane speed 10Gbps (Gen 2x1, default) or 5Gbps (Gen
        # 1x2), Gb/s exponent, RX and TX attributes; absent without SuperSpeedPlus.
        self.assertIsNone(self.ssp_sublink_speeds(bulk_descriptors()))
        for lane_speed in [10, 5]:
            d = bulk_descriptors()
            d.enable_ssp(lane_speed=lane_speed)
            self.assertEqual(self.ssp_sublink_speeds(d), [(3, lane_speed)]*2)
        d = bulk_descriptors()
        d.enable_ssp()
        self.assertEqual(self.ssp_sublink_speeds(d), [(3, 10)]*2)

if __name__ == "__main__":
    unittest.main()
