module mkTop (
  input CLK,
  input RST_N,

  output [7:0] led,
  output ftdi_rxd,
  input ftdi_txd,

  output [3:0] gpdi_dp,
  //output [3:0] gpdi_dn,

  input [11:11] gp,
  inout [11:11] gn
);
  parameter C_ddr = 1'b1;

  // DVI output clock
  wire clkdvi;
  wire clkvga;

  // VGA signals
  wire vga_hsync, vga_vsync, vga_blank;
  // wire video;
  wire [7:0] r_video;
  wire [7:0] g_video;
  wire [7:0] b_video;


  // // converter from VGA to DVI
  wire [1:0] tmds[3:0];

  `ifndef __ICARUS__
  vga2dvid
  #(
    .C_ddr(C_ddr),
    .C_shift_clock_synchronizer(1'b1)
  )
  vga2dvid_instance
  (
    .clk_pixel(CLK),
    .clk_shift(clkdvi),
    .in_red(r_video),
    .in_green(g_video),
    .in_blue(b_video),
    .in_hsync(vga_hsync),
    .in_vsync(vga_vsync),
    .in_blank(vga_blank),
    .out_clock(tmds[3]),
    .out_red(tmds[2]),
    .out_green(tmds[1]),
    .out_blue(tmds[0])
  );

  fake_differential
  #(
    .C_ddr(C_ddr)
  )
  fake_differential_instance
  (
    .clk_shift(clkdvi),
    .in_clock(tmds[3]),
    .in_red(tmds[2]),
    .in_green(tmds[1]),
    .in_blue(tmds[0]),
    .out_p(gpdi_dp),
    .out_n(gpdi_dn)
  );

  // clock generation for the DVI output
  clk_25_system
  clk_25_system_inst
  (
    .clk_in(CLK),
    .pll_125(clkdvi), // 125 Mhz, DDR bit rate
    .pll_25(clkvga)   //  25 Mhz, VGA pixel rate
  );
  `endif

  mkSOC soc(
    .CLK(CLK),
    .RST_N(RST_N),

    .led(led),
    .ftdi_txd(ftdi_txd),
    .ftdi_rxd(ftdi_rxd),

    .vga_hsync(vga_hsync),
    .vga_vsync(vga_vsync),
    .vga_blank(vga_blank),
    .vga_red(r_video),
    .vga_green(g_video),
    .vga_blue(b_video)
  );

endmodule
