#!/usr/bin/env python3

def K(x, y):
    return (y << 5) | x

symbols = {
    "K28.1": (K(28, 1), "SKP", "Skip"),
    "K28.2": (K(28, 2), "SDP", "Start Data Packet"),
    "K28.3": (K(28, 3), "EDB", "End Bad"),
    "K28.4": (K(28, 4), "SUB", "Decode Error Substitution"),
    "K28.5": (K(28, 5), "COM", "Comma"),
    "K28.6": (K(28, 6), "RSD", "Reserved"),
    "K27.7": (K(27, 7), "SHP", "Start Header Packet"),
    "K29.7": (K(29, 7), "END", "End"),
    "K30.7": (K(30, 7), "SLC", "Start Link Command"),
    "K23.7": (K(23, 7), "EPF", "End Packet Framing"),
}

for k, v in symbols.items():
    print("{} : 0x{:02x}".format(k, v[0]))
