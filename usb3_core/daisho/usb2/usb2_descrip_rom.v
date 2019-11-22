// 256x8b rom
// read latency: 1 clock

module usb2_descrip_rom(
	input clk,
	input [7:0] adr,
	output [7:0] dat_r
);

reg [7:0] mem[0:255];
reg [7:0] memadr;
always @(posedge clk) begin
	memadr <= adr;
end

assign dat_r = mem[memadr];

initial begin
	$readmemh("usb2_descrip_rom.init", mem);
end

endmodule
