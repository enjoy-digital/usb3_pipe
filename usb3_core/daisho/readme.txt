

USB 3.0 Device Controller Core
------------------------------

Copyright (c) 2012-2014 Marshall H. All rights reserved.
Code in this folder is released under the terms of the simplified BSD license. 
See LICENSE.TXT for details.

Part of the Daisho project, by Michael Ossmann. http://greatscottgadgets.com/


Revision 0.1, 06/01/14
--------------------------------------------------------------------------------



Overview
--------

This is a device-side controller. This means you can use it to implement a
device talking to a host, whether it be a PC, smartphone, embedded platform...

The controller supports both USB 3.0 (5.0Gbps) and USB 2.0 (480Mbps) operation.
During enumeration if a SuperSpeed receiver is not detected, it will fall
back to 2.0 operation. 

Written in Verilog-2001, it aims to be somewhat easily adaptable to other
devices. It handles PHY strapping and initialization, LFPS with partner, link
training and receiver equalization, link layer data scrambling, elastic buffer
management, device enumeration, and block data transfer.

If you don't know anything about USB, you will have a rough time. If you have 
no experience with FPGAs, you will suffer even more. Read the USB 2.0 spec 
overview, then read about USB 3.0 functionality and how they are incorporated
into each other. Be familiar with the PHY you will use.



Capabilities
------------

The following transfer modes are supported in USB 2.0:
 - Control (intrinsic)
 - Bulk
 - Interrupt
 - Isochronous

And in USB 3.0 mode:
 - Control, Link management
 - Bulk

By default, both controllers implement one BULK IN and one BULK OUT endpoint
in addition to the default Endpoint0 for control transfers.
Additional endpoints may be added with some rework of the protocol layers.

Setting the endpoints to different transfer modes requires editing of the RTL
and also the device descriptors. Consult the USB specifications or your 
favorite search engine for details on the descriptors.



Limitations
-----------

Does not implement the following USB 2.0 functionality:
 - Power saving via SuspendM bit in ULPI
 - Low speed (1.5Mbps) is unimplemented
 - Full speed (11Mbps) is unsupported, but may work

Does not implement the following USB 3.0 functionality:
 - Compliance testing mode
 - Loopback BERT
 - Assertion of disabled data scrambling
 - Tolerance of link command glitches
 - Stream transfer mode

Details:
1. USB devices operating via the 2.0 connection should enter suspend when 
   not transmitting any data on the bus for over 3.0ms. After a total of 10ms 
   have passed, it may not draw more than the rated suspend-mode 
   current instead of the normal current draw.
   This is normally entered when the host stops sending StartOfFrame packets 
   and allows this idle condition. Due to the target platform's fixed current 
   draw this is not an option that can be supported.
2. For compliance testing, the USB 3.0 LTSSM should support a special 
   Compliance state that is entered under certain conditions detailed in the 
   USB specification, for PHY characterization. This mode is not implemented.
3. Also, USB 3.0 Loopback mode is used for benchmarking the bit error rate of
   a connection. As this is not used under normal operation, it is omitted to
   save device resources.
4. During link training, one partner may choose to signal it wishes to disable
   data scrambling. The other partner should signal its acknowledgement in the
   next training sequence. This is supported but basically untested.
5. Per the specification, link commands (which are 4 consecutive, redundant
   K-symbols) can have up to 1 invalid symbol and still be considered valid.
   To simplify functionality the controller will simply ignore these and 
   cause a sequence error. Because the protocol is robust anyway, these will
   at most cause a small performance blip in adverse environments (poor
   cabling, bad connectors, etc)
6. Streams are unsupported, if you need them in your application you are
   exceeding the expected use case


Implementation Results
----------------------

Logic utilization: 
    Approx. 1100 LEs for USB 2.0
    Approx. 6300 LEs for USB 3.0
    1 PLL
    ~40 DDR I/O
    8-10Kbyte distributed block RAM (flexible)

Targeted PHY:
    Texas Instruments TUSB1310A SuperSpeed PHY

Targeted FPGA:
    Altera Cyclone IV EP4CE30 speed grade 8

Build system:
    Altera Quartus II 13.1sp1



Porting Notes
-------------

This controller relies upon several vendor and family-specific features such as
PLLs, pipelined DDR I/O, distributed block RAM, and so on. 
Altera calls these "Megafunctions" and they are denoted by "mf_*.v" filenames.

If you are going to port this to Xilinx, Lattice, silicon then you will want to
blow these away and use the vendor's equivalent tools to get the same result.

I would recommend that you download Quartus and use the Megawizard Plug-in 
Manager to open these files so that you can see how they are supposed to work,
and how you would write wrappers for slightly different device functionality.

You're on your own here, this is not something I would recommend to an FPGA
newcomer (though arguably using this core itself is not trivial.)



Usage Considerations
--------------------

There are a number of design decisions and small things that may affect your
success using the code.

1. Device descriptors
   When running the core with only USB 2.0, edit the device descriptors so that
   the "bcdDevice" entry denotes 200h instead of 210h. The latter indicates the
   device has latent SuperSpeed functionality.
   And obviously, if you want to change the enumerated display name you'll want
   to change these. They must always match the RTL endpoint configuration.

2. I/O constraints
   It's your responsibility to constrain the top level I/O to meet your PCB
   layout. Consult your PHY datasheet, and read FPGA vendor datasheets. 
   PIPE bus I/O is fast enough that timing driven synthesis is recommended

3. SSRX/TX polarity
   The core can tell when the RX data pair has its polarity swapped and will
   configure the PHY accordingly during link training. You can swap either TX
   or RX pairs for more convenient PCB routing. Consult your PHY documentation.

4. Reset
   To reset the core to its power-on state, assert reset for at least a few
   hundred cycles (around 10 uS would be a safe bet.). Additionally, your PHY
   chip may have its own requirements for this. Reset for whichever duration is
   longest.



Troubleshooting
---------------

It's highly unlikely that things will work properly the first time. You would
be well advised to buy/lease/borrow a competent protocol analyzer. Units are
available from the major vendors, which include:

  - Teledyne Lecroy
  - Agilent
  - Tektronix
  - Total Phase

The Beagle 5000v2 from Total Phase was used for the core's development. It
runs about $5-6k. Additionally, the Lecroy Voyagyer M3x was used to test
compliance cases to identify rare problems.
Other vendors provide add-on packages for their higher end scopes, but if 
you could afford those, you'd be licensing a USB core from DesignWare 
anyhow. :)
If you have access to proper verification IP you'd probably want to test this
core against it. I welcome any changes that will make the core more robust.

If you have some knowledge of the protocols you can trigger on the RTL with
Chipscope, Signaltap, Reveal, et al. This will only give you a very narrow
window but it's better than nothing.



Contact
-------

I do not provide support! I will not answer STUPID questions or ones that
make it clear you are going to be wasting not only your own time but mine too.

Bugfixes are welcome if you can clearly demonstrate what is fixed and that no
regressions are caused.

If you successfully implement the core and ship a product I would appreciate
sending an email to let me know it worked.

usb3ipcore, at outlookdotcom



EOF