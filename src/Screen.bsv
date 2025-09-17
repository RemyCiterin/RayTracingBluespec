import SpecialFIFOs :: *;
import RegFile :: *;
import GetPut :: *;
import FIFOF :: *;
import Ehr :: *;

import BRAMCore :: *;

typedef struct {
  Bit#(TMul#(n,3)) r;
  Bit#(TMul#(n,3)) g;
  Bit#(TMul#(n,2)) b;
} ColorN#(numeric type n)
deriving(Bits, FShow, Eq);

function ColorN#(n) colorN(Bit#(TMul#(n,3)) r, Bit#(TMul#(n,3)) g, Bit#(TMul#(n,2)) b) =
  ColorN{r:r, g:g, b:b};

function ColorN#(n) mulColorN(ColorN#(n) c1, ColorN#(n) c2);
  Bit#(TMul#(n, 6)) red = 0;
  Bit#(TMul#(n, 6)) green = 0;
  Bit#(TMul#(n, 4)) blue = 0;

  for (Integer i=0; i < valueOf(n)*3; i = i + 1) begin
    red = red + ((c1.r[i] == 1 ? zeroExtend(c2.r) : 0) << i);
    green = green + ((c1.g[i] == 1 ? zeroExtend(c2.g) : 0) << i);
  end

  for (Integer i=0; i < valueOf(n)*2; i = i + 1) begin
    blue = blue + ((c1.b[i] == 1 ? zeroExtend(c2.b) : 0) << i);
  end

  return colorN(
    red[valueOf(n)*6-1:valueOf(n)*3],
    green[valueOf(n)*6-1:valueOf(n)*3],
    blue[valueOf(n)*4-1:valueOf(n)*2]
  );
endfunction

typedef struct {
  Bit#(6) r;
  Bit#(6) g;
  Bit#(4) b;
} Color16
deriving(Bits, FShow, Eq);

function Color16 color16(Bit#(6) r, Bit#(6) g, Bit#(4) b) = Color16{r:r, g:g, b:b};

function Color16 mulColor16(Color16 c1, Color16 c2);
  function Bit#(6) approx12(Bit#(12) n) = truncateLSB(n);
  function Bit#(4) approx8(Bit#(8) n) = truncateLSB(n);

  Bit#(12) r0 = c1.r[0] == 1 ? zeroExtend(c2.r) : 0;
  Bit#(12) r1 = c1.r[1] == 1 ? zeroExtend(c2.r) : 0;
  Bit#(12) r2 = c1.r[2] == 1 ? zeroExtend(c2.r) : 0;
  Bit#(12) r3 = c1.r[3] == 1 ? zeroExtend(c2.r) : 0;
  Bit#(12) r4 = c1.r[4] == 1 ? zeroExtend(c2.r) : 0;
  Bit#(12) r5 = c1.r[5] == 1 ? zeroExtend(c2.r) : 0;

  Bit#(12) g0 = c1.g[0] == 1 ? zeroExtend(c2.g) : 0;
  Bit#(12) g1 = c1.g[1] == 1 ? zeroExtend(c2.g) : 0;
  Bit#(12) g2 = c1.g[2] == 1 ? zeroExtend(c2.g) : 0;
  Bit#(12) g3 = c1.g[3] == 1 ? zeroExtend(c2.g) : 0;
  Bit#(12) g4 = c1.g[4] == 1 ? zeroExtend(c2.g) : 0;
  Bit#(12) g5 = c1.g[5] == 1 ? zeroExtend(c2.g) : 0;

  Bit#(8) b0 = c1.b[0] == 1 ? zeroExtend(c2.b) : 0;
  Bit#(8) b1 = c1.b[1] == 1 ? zeroExtend(c2.b) : 0;
  Bit#(8) b2 = c1.b[2] == 1 ? zeroExtend(c2.b) : 0;
  Bit#(8) b3 = c1.b[3] == 1 ? zeroExtend(c2.b) : 0;

  return color16(
    approx12( r0 + (r1 << 1) + (r2 << 2) + (r3 << 3) + (r4 << 4) + (r5 << 5) ),
    approx12( g0 + (g1 << 1) + (g2 << 2) + (g3 << 3) + (g4 << 4) + (g5 << 5) ),
    approx8( b0 + (b1 << 1) + (b2 << 2) + (b3 << 3) )
  );
endfunction

instance Arith#(Color16);
  function Color16 \+ (Color16 c1, Color16 c2) = color16(c1.r + c2.r, c1.g + c2.g, c1.b + c2.b);
  function Color16 \- (Color16 c1, Color16 c2) = color16(c1.r - c2.r, c1.g - c2.g, c1.b - c2.b);
  function Color16 \* (Color16 c1, Color16 c2) = mulColor16(c1,c2);
  function Color16 \/ (Color16 c1, Color16 c2) = error("/ is undefined for Color16");
  function Color16 \% (Color16 c1, Color16 c2) = error("% is undefined for Color16");
  function Color16 negate(Color16 c) = color16(-c.r, -c.g, -c.b);
  function Color16 abs(Color16 c) = error("abs is undefined for Color16");
  function Color16 signum(Color16 c) = error("signum is undefined for Color16");
  function Color16 \** (Color16 c1, Color16 c2) = error("** is undefined for Color16");
  function Color16 exp_e(Color16 c) = error("** is undefined for Color16");
  function Color16 log(Color16 c) = error("log is undefined for Color16");
  function Color16 log2(Color16 c) = error("log2 is undefined for Color16");
  function Color16 log10(Color16 c) = error("log10 is undefined for Color16");
  function Color16 logb(Color16 b, Color16 c) = error("logb is undefined for Color16");
endinstance

instance Literal#(Color16);
  function Color16 fromInteger(Integer n);
    return color16(fromInteger(n), fromInteger(n), fromInteger(n));
  endfunction
endinstance

typedef struct {
  Bit#(3) r;
  Bit#(3) g;
  Bit#(2) b;
} Color8
deriving(Bits, FShow, Eq);

function Color8 color8(Bit#(3) r, Bit#(3) g, Bit#(2) b) = Color8{r:r, g:g, b:b};

function Color8 mulColor8(Color8 c1, Color8 c2);
  function Bit#(3) approx6(Bit#(6) n) = n[2:0] >= 3'b100 ? n[5:3]+1 : n[5:3];
  function Bit#(2) approx4(Bit#(4) n) = n[1:0] >= 2'b10 ? n[3:2]+1 : n[3:2];

  Bit#(6) r0 = c1.r[0] == 1 ? zeroExtend(c2.r) : 0;
  Bit#(6) r1 = c1.r[1] == 1 ? zeroExtend(c2.r) : 0;
  Bit#(6) r2 = c1.r[2] == 1 ? zeroExtend(c2.r) : 0;

  Bit#(6) g0 = c1.g[0] == 1 ? zeroExtend(c2.g) : 0;
  Bit#(6) g1 = c1.g[1] == 1 ? zeroExtend(c2.g) : 0;
  Bit#(6) g2 = c1.g[2] == 1 ? zeroExtend(c2.g) : 0;

  Bit#(4) b0 = c1.b[0] == 1 ? zeroExtend(c2.b) : 0;
  Bit#(4) b1 = c1.b[1] == 1 ? zeroExtend(c2.b) : 0;

  return color8(
    approx6( r0 + (r1 << 1) + (r2 << 2) ),
    approx6( g0 + (g1 << 1) + (g2 << 2) ),
    approx4( b0 + (b1 << 1) )
  );
endfunction

instance Literal#(Color8);
  function Color8 fromInteger(Integer n);
    return color8(fromInteger(n), fromInteger(n), fromInteger(n));
  endfunction
endinstance

instance Arith#(Color8);
  function Color8 \+ (Color8 c1, Color8 c2) = color8(c1.r + c2.r, c1.g + c2.g, c1.b + c2.b);
  function Color8 \- (Color8 c1, Color8 c2) = color8(c1.r - c2.r, c1.g - c2.g, c1.b - c2.b);
  function Color8 \* (Color8 c1, Color8 c2) = mulColor8(c1,c2);
  function Color8 \/ (Color8 c1, Color8 c2) = error("/ is undefined for Color8");
  function Color8 \% (Color8 c1, Color8 c2) = error("% is undefined for Color8");
  function Color8 negate(Color8 c) = color8(-c.r, -c.g, -c.b);
  function Color8 abs(Color8 c) = error("abs is undefined for Color8");
  function Color8 signum(Color8 c) = error("signum is undefined for Color8");
  function Color8 \** (Color8 c1, Color8 c2) = error("** is undefined for Color8");
  function Color8 exp_e(Color8 c) = error("** is undefined for Color8");
  function Color8 log(Color8 c) = error("log is undefined for Color8");
  function Color8 log2(Color8 c) = error("log2 is undefined for Color8");
  function Color8 log10(Color8 c) = error("log10 is undefined for Color8");
  function Color8 logb(Color8 b, Color8 c) = error("logb is undefined for Color8");
endinstance

function Color8 fromRGB(Bit#(8) r, Bit#(8) g, Bit#(8) b);
  return Color8{r: r[7:5], g: g[7:5], b: b[7:6]};
endfunction

function Color8 color16to8(Color16 c) =
  color8(truncateLSB(c.r), truncateLSB(c.g), truncateLSB(c.b));

function Tuple3#(Bit#(8),Bit#(8),Bit#(8)) toRGB(Color8 color);
  function Bit#(8) match3(Bit#(3) x) =
    case (x) matches
      3'b000 : 8'h00;
      3'b001 : 8'h24;
      3'b010 : 8'h49;
      3'b011 : 8'h6d;
      3'b100 : 8'h92;
      3'b101 : 8'hb6;
      3'b110 : 8'hdb;
      3'b111 : 8'hff;
    endcase;

  function Bit#(8) match2(Bit#(2) x) =
    case (x) matches
      2'b00 : 8'h00;
      2'b01 : 8'h55;
      2'b10 : 8'haa;
      2'b11 : 8'hff;
    endcase;

  return tuple3(
    match3(color.r),
    match3(color.g),
    match2(color.b)
  );
endfunction

function Action write_one_rgb
  (VGA vga, Bit#(32) x, Bit#(32) y, Bit#(8) r, Bit#(8) g, Bit#(8) b) =
  vga.write((x >> 2) + y * 80, zeroExtend(pack(fromRGB(r,g,b))) << {x[1:0], 3'b000}, 1 << x[1:0]);

function Action write_one_color8
  (VGA vga, Bit#(32) x, Bit#(32) y, Color8 c) =
  vga.write((x >> 2) + y * 80, zeroExtend(pack(c)) << {x[1:0], 3'b000}, 1 << x[1:0]);

function Action write_one_color16
  (VGA vga, Bit#(32) x, Bit#(32) y, Color16 c) =
  vga.write((x >> 2) + y * 80, zeroExtend(pack(color16to8(c))) << {x[1:0], 3'b000}, 1 << x[1:0]);

// Fabric interface (system verilog size) of the screen buffer
interface VGAFabric;
  (* always_ready, always_enabled, result= "vga_hsync" *)
  method Bool hsync;

  (* always_ready, always_enabled, result= "vga_vsync" *)
  method Bool vsync;

  (* always_ready, always_enabled, result= "vga_blank" *)
  method Bool blank;

  (* always_ready, always_enabled, result= "vga_red" *)
  method Bit#(8) red;

  (* always_ready, always_enabled, result= "vga_blue" *)
  method Bit#(8) blue;

  (* always_ready, always_enabled, result= "vga_green" *)
  method Bit#(8) green;
endinterface

interface VGA;
  (* prefix = "" *)
  interface VGAFabric fabric;

  // CPU interface to write data into the VGA frame buffer
  method Action write(Bit#(32) addr, Bit#(32) data, Bit#(4) mask);
endinterface

module mkVGA(VGA);
  // Fabric parameters of the vga interface
  Integer hwidth = 640;
  Integer vwidth = 480;

  Integer hsync_front_porch = 16;
  Integer hsync_pulse_width = 96;
  Integer hsync_back_porch = 48;

  Integer vsync_front_porch = 11;
  Integer vsync_pulse_width = 2;
  Integer vsync_back_porch = 31;

  Integer hframe = hwidth + hsync_front_porch + hsync_pulse_width + hsync_back_porch;
  Integer vframe = vwidth + vsync_front_porch + vsync_pulse_width + vsync_back_porch;

  // CPU parameters of the vga interface
  Integer xmax = hwidth;
  Integer ymax = vwidth;

  // Frame buffer
  // Represent a 320 * 240 pixel screen
  BRAM_DUAL_PORT_BE#(Bit#(32), Bit#(32), 4) bram <- mkBRAMCore2BE(xmax * ymax / 16, False);

  // Fabric registers
  Reg#(Bit#(32)) fabric_addr <- mkReg(0);
  Reg#(Bit#(10)) hpos <- mkReg(0);
  Reg#(Bit#(10)) vpos <- mkReg(0);

  Reg#(File) file <- mkReg(InvalidFile);
  Reg#(Bool) started <- mkReg(False);

  function Bit#(32) getFabricAddr;
    Bit#(20) h = zeroExtend(hpos >> 1);
    Bit#(20) v = zeroExtend(vpos >> 1);

    Bit#(32) ret = zeroExtend(h + v * fromInteger(xmax / 2));
    return (ret >= fromInteger(xmax * ymax / 4) ? 0 : ret);
  endfunction

  function Bit#(8) getFabricResponse;
    let x = bram.a.read;

    return case (fabric_addr[1:0]) matches
      2'b00 : x[7:0];
      2'b01 : x[15:8];
      2'b10 : x[23:16];
      2'b11 : x[31:24];
    endcase;
  endfunction

  rule openFile if (!started);
    File f <- $fopen("screen.txt", "w");
    started <= True;
    file <= f;
  endrule

  rule enqBramRead;
    let addr = getFabricAddr;
    bram.a.put(0, zeroExtend(addr[31:2]), ?);
  endrule

  // Write into all the fabric wires
  (* no_implicit_conditions, fire_when_enabled *)
  rule fabric_write;
    let next_hpos = (hpos+1 >= fromInteger(hframe) ? 0 : hpos + 1);
    let next_vpos =
      (hpos+1 >= fromInteger(hframe) ? (vpos+1 >= fromInteger(vframe) ? 0 : vpos+1) : vpos);

    fabric_addr <= getFabricAddr;
    hpos <= next_hpos;
    vpos <= next_vpos;
  endrule

  match {.red_proj, .green_proj, .blue_proj} = toRGB(unpack(getFabricResponse));

  method Action write(Bit#(32) addr, Bit#(32) data, Bit#(4) mask) if (started);
    if (mask != 0)
      bram.b.put(mask, addr, data);
  endmethod

  interface VGAFabric fabric;
    method hsync =
      hpos < fromInteger(hwidth + hsync_front_porch) ||
      hpos >= fromInteger(hframe - hsync_back_porch);

    method vsync =
      vpos < fromInteger(vwidth + vsync_front_porch) ||
      vpos >= fromInteger(vframe - vsync_back_porch);

    method blank =
      hpos >= fromInteger(hwidth) || vpos >= fromInteger(vwidth);

    method red = red_proj;

    method green = green_proj;

    method blue = blue_proj;
  endinterface
endmodule
