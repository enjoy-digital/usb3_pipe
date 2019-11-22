// 1024x8b ram (1 write / 1 read / 2 clocks)
// read latency: 1 clock

module usb2_ep_ram(
	input wr_clk,
	input wr_we,
	input [9:0] wr_adr,
	input [7:0] wr_dat_w,

	input rd_clk,
	input [9:0] rd_adr,
	output [7:0] rd_dat_r
);

reg [7:0] mem[0:1023];
reg [9:0] rd_adr_i;
always @(posedge wr_clk) begin
	if (wr_we)
		mem[wr_adr] <= wr_dat_w;
end

always @(posedge rd_clk) begin
	rd_adr_i <= rd_adr;
end

assign rd_dat_r = mem[rd_adr_i];

endmodule
