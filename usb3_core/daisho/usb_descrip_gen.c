
//
// USB 3.0/2.0 IP core descriptor generator
//
// Copyright (c) 2013 Marshall H.
// Copyright (c) 2017 Florent Kermarrec
// All rights reserved.
// This code is released under the terms of the simplified BSD license.
// See LICENSE.TXT for details.
//

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#define WIN32_LEAN_AND_MEAN
//#include <windows.h>	// just to ruffle jared's feathers

#define u8	unsigned char
#define u16	unsigned short
#define u32	unsigned int

// to add endpoints, change strings etc, just skip to main() at bottom
//
// warning: objectionable C code ahead
// proceed with extreme prejudice

char filename_usb2_init[] = "./usb2/usb2_descrip_rom.init";
char filename_usb3_init[] = "./usb3/usb3_descrip_rom.init";
char filename_usb2_bin[] = "./usb2/usb2_descrip.bin";
char filename_usb3_bin[] = "./usb3/usb3_descrip.bin";
char filename_descrip_vh[] = "usb_descrip.vh";


FILE *bin_2, *bin_3, *init_2, *init_3, *descrip_vh;
u32 i_2, i_3;
u32 c_2, c_3;

u8	*a;
u8	*z;
u8	buf_2[1024] = {0, };
u8	buf_3[1024] = {0, };
u32 bytes_pending_2;
u32 bytes_pending_3;
u8	bitwidth_2;
u8	bitwidth_3;
u32 bytes_written_2 = 0;
u32 bytes_written_3 = 0;

void fail(void *ret, char *str)
{
	if(ret == NULL){
		printf("\n* FAIL: %s\n", str);
		exit(-1);
	}
}

void write_buf8(u8 *buf, u32 size)
{
	u32 i;
	for(i = 0; i < size; i++){
		fprintf(init_2, "%02X\n", *buf++);
		fwrite(buf-1, 1, 1, bin_2);
		bytes_written_2 ++;
	}
}

void write_buf32(u32 *buf, u32 size)
{
	u32 i, w;
	for(i = 0; i < size; i+=4){
		w = *buf;
		w = (w & 0xff) << 24 | (w & 0xff00) << 8 | (w & 0xff0000) >> 8 | (w & 0xff000000) >> 24 ;
		fprintf(init_3, "%08X\n", w);
		fwrite(buf, 4, 1, bin_3);
		*buf++;
		bytes_written_3 += 4;
	}
}

void write_buf(void *buf, u32 size)
{
	write_buf8(buf, size);
	write_buf32(buf, size);
}

void print_offsets(char *name, u8 for_usb2, u8 for_usb3)
{
	if(for_usb2) fprintf(descrip_vh, "parameter\t[%d:0]\tDESCR_USB2_%s	= 'd%d;\n", bitwidth_2-1, name, i_2);
	if(for_usb3) fprintf(descrip_vh, "parameter\t[%d:0]\tDESCR_USB3_%s	= 'd%d;\n", bitwidth_3-1, name, i_3);
}

void add_device_qual(u16 usb_spec, u8 class_code, u8 subclass, u8 protocol_code, u8 max_size_ep0,
					u8 num_possible_config)
{
	a = buf_2;
	print_offsets("DEVICE_QUAL", 1, 0);

	// usb spec 2.10
	// this is not included in USB3 descriptors
	*a++ = 0x0A;
	*a++ = 0x06;
	*a++ = (usb_spec == 0x300) ? (0x210&0xff) : (0x200&0xff);
	*a++ = (usb_spec == 0x300) ? (0x210>>8) : (0x200>>8);
	*a++ = class_code;
	*a++ = subclass;
	*a++ = protocol_code;
	*a++ = max_size_ep0;
	*a++ = num_possible_config;
	*a++ = 0;
	write_buf8(buf_2, 0xA);
}

void add_device_descr(u16 usb_spec, u8 class_code, u8 subclass, u8 protocol_code, u8 max_size_ep0,
					 u16 vid, u16 pid, u16 dev_num,
					 u8 idx_mfg, u8 idx_prod, u8 idx_serial, u8 num_possible_config)
{
	a = buf_2;
	print_offsets("DEVICE", 1, 1);

	// usb spec 2.10
	*a++ = 0x12;					// length
	*a++ = 0x01;					// descriptor ID
	*a++ = (usb_spec == 0x300) ? (0x210&0xff) : (0x200&0xff); // bcdUSB
	*a++ = (usb_spec == 0x300) ? (0x210>>8) : (0x200>>8); // bcdUSB
	write_buf8(buf_2, 4); a = buf_2+2;
	// usb spec 3.00
	*a++ = (0x300&0xff); // bcdUSB
	*a++ = (0x300>>8); // bcdUSB
	write_buf32((u32 *)buf_2, 4); a = buf_2;

	*a++ = class_code;				// bDeviceClass
	*a++ = subclass;				// bDeviceSubClass
	*a++ = protocol_code;			// bDeviceProtocol
	*a++ = max_size_ep0;			// bMaxPacketSize0
	write_buf8(buf_2, 4); a--;
	*a++ = 0x09;					// USB3: 512 bytes fixed EP0
	write_buf32((u32 *)buf_2, 4); a = buf_2;

	*a++ = (vid&0xff);
	*a++ = (vid>>8);
	*a++ = (pid&0xff);
	*a++ = (pid>>8);
	*a++ = (dev_num&0xff);
	*a++ = (dev_num>>8);
	*a++ = idx_mfg;
	*a++ = idx_prod;
	*a++ = idx_serial;
	*a++ = num_possible_config;		// bNumConfigurations
	write_buf(buf_2, 10);

	add_device_qual(	usb_spec,		// USB spec number (2.xx only)
						class_code,		// Device Class
						subclass,		// Device subclass
						protocol_code,	// Device protocol code
						max_size_ep0,	// Endpoint0 Max packet
						num_possible_config
					);
}

void add_config_start(u16 usb_spec, u8 attrib, u32 power_ma, u8 num_endpoints)
{
	a = buf_2;
	z = buf_3;
	c_2 = i_2;
	c_3 = i_3;
	print_offsets("CONFIG", 1, 1);
	bytes_pending_2 = 0;
	bytes_pending_3 = 0;

	// write Config descriptor
	*a++ = 0x09;				// length
	*a++ = 0x02;				// descriptor ID
	*a++ = (0xffff&0xff);       // total length, must be overwritten later
	*a++ = (0xffff>>8);   		// total length, must be overwritten later
	*a++ = 0x01;				// bNumInterfaces
	*a++ = 0x01;				// bConfigurationValue
	*a++ = 0x00;				// iConfiguration
	*a++ = attrib;				// bmAttributes
	*a++ = (usb_spec == 0x300) ? power_ma / 8 : power_ma / 2;
	bytes_pending_2 += 0x9;
	bytes_pending_3 += 0x9;

	// write Interface descriptor
	*a++ = 0x09;				// length
	*a++ = 0x04;				// descriptor ID
	*a++ = 0x00;				// bInterfaceNumber
	*a++ = 0x00;				// bAlternateSetting
	*a++ = num_endpoints;		// bNumEndpoints
	*a++ = 0xFF;				// bInterfaceClass
	*a++ = 0xFF;				// bInterfaceSubClass
	*a++ = 0xFF;				// bInterfaceProtocol
	*a++ = 0x02;				// iInterface
	bytes_pending_2 += 0x9;
	bytes_pending_3 += 0x9;

	// now here's the annoying part...
	// create a second buffer just for usb3
	// since it has companion descriptors
	memcpy(buf_3, buf_2, bytes_pending_2);
	z += bytes_pending_3;
}

void add_endpoint(u8 idx, u8 dir, u8 attrib, u16 max_pkt, u8 interval, u8 max_burst,
				 u8 attrib_3, u16 bytes_per_interval )
{
	u8 *c = a;

	// write Endpoint descriptor
	*a++ = 0x07;				// length
	*a++ = 0x05;				// descriptor ID
	*a++ = idx | (dir ? 0x80 : 0x0);
	*a++ = attrib;				// bmAttributes
	*a++ = (max_pkt&0xff);	    // max packet size
	*a++ = (max_pkt>>8);	    // max packet size
	*a++ = interval;			// bInterval
	bytes_pending_2 += 0x7;
	bytes_pending_3 += 0x7;

	// copy this endpoint over to USB3
	memcpy(z, c, 0x7);
	z += 0x7;
	// patch max packet size to 1024
	*(z-2) = 0x04;
	*(z-3) = 0x00;

	// add companion descriptor only to USB3 buffer
	*z++ = 0x06;				// length
	*z++ = 0x30;				// descriptor ID
	*z++ = max_burst-1;
	*z++ = attrib_3;			// bmAttributes
	*z++ = (bytes_per_interval&0xff); // wBytesPerInterval
	*z++ = (bytes_per_interval>>8);	  // wBytesPerInterval
	bytes_pending_3 += 0x6;

}


void add_config_end()
{
	u8 *c = a;
	// add alternate interface
	// this is needed so that windows can select it if isoch. bandwidth
	// is not able to be reserved
	*a++ = 0x09;				// length
	*a++ = 0x04;				// descriptor ID
	*a++ = 0x00;				// bInterfaceNumber
	*a++ = 0x01;				// bAlternateSetting
	*a++ = 0x0;					// bNumEndpoints
	*a++ = 0xFF;				// bInterfaceClass
	*a++ = 0xFF;				// bInterfaceSubClass
	*a++ = 0xFF;				// bInterfaceProtocol
	*a++ = 0x02;				// iInterface
	bytes_pending_2 += 0x9;
	bytes_pending_3 += 0x9;

	// copy this interface over to USB3
	memcpy(z, c, 0x9);
	z += 0x9;

	// patch total descriptor lengths
	memcpy(&buf_2[2], &bytes_pending_2, 2);
	memcpy(&buf_3[2], &bytes_pending_3, 2);

	write_buf8(buf_2, bytes_pending_2);
	write_buf32((u32 *)buf_3, bytes_pending_3);
}

void add_bos()
{
	z = buf_3;
	print_offsets("BOS    ", 0, 1);

	*z++ = 0x05;				// bLength
	*z++ = 0x0F;				// bDescriptorType
	*z++ = 0x16;				// wTotalLength
	*z++ = 0x00;				//

	*z++ = 0x02;				// bNumDeviceCaps

	*z++ = 0x07;				// bLength
	*z++ = 0x10;				// bDescriptorType
	*z++ = 0x02;				// bDevCapabilityType (USB 2.0 EXTENSION)

	*z++ = 0x02;                // 32bit field: LPM Supported (SS Required)
	*z++ = 0x00;                // 32bit field: LPM Supported (SS Required)
	*z++ = 0x00;                // 32bit field: LPM Supported (SS Required)
	*z++ = 0x00;                // 32bit field: LPM Supported (SS Required)

	*z++ = 0x0A;				// bLength
	*z++ = 0x10;				// bDescriptorType
	*z++ = 0x03;				// bDevCapabilityType (SUPERSPEED_USB)
	*z++ = 0x00;				// bmAttributes (8bit) (LTM Generation Incapable)
	*z++ = 0x0E;		        // wSpeedsSupported (Operation supported FS, HS, SS)
	*z++ = 0x00;	         	// wSpeedsSupported (Operation supported FS, HS, SS)
	*z++ = 0x02;				// bFunctionalitySupported (Valid operation starts with HS)
	*z++ = 0x08;				// bU1DevExitLat (Less than 8uS)
	*z++ = 0x64;				// wU2DevExitLat (Less than 100uS)
	*z++ = 0x00;	         	// wU2DevExitLat (Less than 100uS)


	write_buf32((u32 *)buf_3, z - buf_3);
}

void add_string(u8 idx, char* str)
{
	char temp[32];
	wchar_t w_str[128];
	u32 len;
	if(idx == 0)
		len = 4;		// string0 is fixed at 4 bytes for language code
	else
		len = strlen(str)*2+2;

	memset(buf_2, 0, sizeof(buf_2));
	a = buf_2;

	sprintf(temp, "STRING%d", idx);
	print_offsets(temp, 1, 1);

	if(idx > 0)
		mbstowcs(w_str, str, 128);
	else
		memcpy(&w_str[0], str, 2);

	*a++ = len;
	*a++ = 0x03;
	memcpy(a, w_str, len-2);
	write_buf(buf_2, len);
}

void add_set()
{
	print_offsets("CONFUNSET", 1, 1);
	write_buf("\x00", 1);
	print_offsets("CONFSET", 1, 1);
	write_buf("\x01", 1);
}

int main(int argc, char *argv[])
{
	u32 temp1, temp2;
	i_2 = 0;
	i_3 = 0;

	printf("\n* Daisho USB 3.0 / USB 2.0 descriptor export tool\n  by marshallh, 2013\n");

	fail(init_2 = fopen(filename_usb2_init, "w"), "Failed opening USB2.0 INIT");
	fail(init_3 = fopen(filename_usb3_init, "w"), "Failed opening USB3.0 INIT");
	fail(bin_2 = fopen(filename_usb2_bin, "wb"), "Failed opening USB2.0 BIN");
	fail(bin_3 = fopen(filename_usb3_bin, "wb"), "Failed opening USB3.0 BIN");
	fail(descrip_vh = fopen(filename_descrip_vh, "w"), "Failed opening descriptor include");

	// set your BRAM address bit widths
	bitwidth_2 = 8;	// [7:0]
	bitwidth_3 = 7;	// [6:0]

	printf ("* Generating...\n");

	add_device_descr(	0x300,		// USB spec number (auto fixed 2.10 for 2.0)
									// put 0x200 if you only use USB 2.0 core
						0xFF,		// Class Code
						0xFF,		// Subclass
						0xFF,		// Protocol Code
						64,			// Endpoint0 Max packet (ignored for 3.0)
						0x1D50,		// Vendor ID
						0x605A,		// Product ID
						0x1,		// Device release number
						1,			// Index of Manufacturer Str Descriptor
						2,			// Index of Product Str Descriptor
						3,			// Index of Serial Number Str Descriptor
						1			// Number of Possible Configs
					);

	add_config_start(	0x300,		// USB spec number (auto fixed 2.10 for 2.0),
						0x80,		// TODO attributes (self powered)
						500,		// Power draw in mA
						2			// Number of endpoints
					);

	add_endpoint(		1,			// EP1
						1,			// IN
						2,			// BULK
						512,		// Max packet size (autofixed to 1024 for USB3)
						0x1,		// Interval for isoch. endpoints
						16,			// Max burst packets for usb 3.0
						0x00,		// No stream support
						0			// 0bytes per interval (BULK)
					);

	add_endpoint(		2,			// EP2
						0,			// OUT
						2,			// BULK
						512,		// Max packet size (autofixed to 1024 for USB3)
						0x1,		// Interval for isoch. endpoints
						16,			// Max burst packets for usb 3.0
						0x00,		// No stream support
						0			// 0bytes per interval (BULK)
					);

	add_config_end();

	// hack to print out CONFIG length
	temp1 = i_2;
	temp2 = i_3;
	i_2 = bytes_pending_2;
	i_3 = bytes_pending_3;
	print_offsets("CONFIG_LEN", 1, 1);
	i_2 = temp1;
	i_3 = temp2;

	add_bos();

	// hack to print out BOS length
	temp2 = i_3;
	i_3 = z - buf_3;
	print_offsets("BOS_LEN", 0, 1);
	i_3 = temp2;

	add_string(0, "\x09\x04\0");
	add_string(1, "Great Scott Gadgets");
	add_string(2, "Daisho USB test");
	add_string(3, "DAISHOUSB000");

	add_set();

	print_offsets("EOF     ", 1, 1);

	// pad out files to their max address space
	while(bytes_written_2 < pow(2, bitwidth_2)) write_buf8("\0", 1);
	while(bytes_written_3 < pow(2, bitwidth_3)*4) write_buf32((u32 *)"\0\0\0\0", 1);

	printf("* Finished\n");
	fclose(bin_2);
	fclose(bin_3);
	fclose(init_2);
	fclose(init_3);
	fclose(descrip_vh);
	return 0;
}


