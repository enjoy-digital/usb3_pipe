// 1024x32b ram (1 write / 1 read / 2 clocks)
// read latency: 2 clocks

module usb3_ep_ram(
	input wr_clk,
	input wr_rst,
	input wr_we,
	input [9:0] wr_adr,
	input [31:0] wr_dat_w,

	input rd_clk,
	input rd_rst,
	input [9:0] rd_adr,
	output reg [31:0] rd_dat_r
);

reg [31:0] mem[0:1023];
reg [9:0] rd_adr_i;
always @(posedge wr_clk) begin
	if (wr_we)
		mem[wr_adr] <= wr_dat_w;
end

always @(posedge rd_clk) begin
	rd_adr_i <= rd_adr;
	rd_dat_r = mem[rd_adr_i];
end

endmodule
