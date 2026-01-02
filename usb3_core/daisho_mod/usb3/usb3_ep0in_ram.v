// 16x32b ram (1 write / 1 read / 1 clock)
// read latency: 2 clocks

module usb3_ep0in_ram(
	input clk,
	input wr_we,
	input [3:0] wr_adr,
	input [31:0] wr_dat_w,
	input [3:0] rd_adr,
	output reg [31:0] rd_dat_r

);

reg [31:0] mem[0:15];
reg [3:0] rd_adr_i;
always @(posedge clk) begin
	if (wr_we)
		mem[wr_adr] <= wr_dat_w;
end

always @(posedge clk) begin
	rd_adr_i <= rd_adr;
	rd_dat_r = mem[rd_adr_i];
end

endmodule
