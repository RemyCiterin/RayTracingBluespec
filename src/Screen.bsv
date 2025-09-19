import SpecialFIFOs :: *;
import RegFile :: *;
import GetPut :: *;
import FIFOF :: *;
import Ehr :: *;

import BRAMCore :: *;

`ifdef BSIM
import "BDPI" function Action init();
import "BDPI" function Action drawScreen();
import "BDPI" function Action
  setPoint(Bit#(32) x, Bit#(32) y, Bit#(8) r, Bit#(8) g, Bit#(8) b);
`endif

typedef struct {
  Bit#(8) r;
  Bit#(8) g;
  Bit#(8) b;
} Color deriving(Bits, FShow, Eq);

function Color rgb(Bit#(8) r, Bit#(8) g, Bit#(8) b) = Color{r:r, g:g, b:b};

function Color multRgb(Color c1, Color c2);
  Bit#(16) r = 0;//zeroExtend(c1.r)*zeroExtend(c2.r);//0;
  Bit#(16) g = 0;//zeroExtend(c1.g)*zeroExtend(c2.g);//0;
  Bit#(16) b = 0;//zeroExtend(c1.b)*zeroExtend(c2.b);//0;

  for (Integer i=0; i < 8; i = i + 1) begin
    r = r + (c1.r[i] == 1 ? zeroExtend(c2.r) << i : 0);
    g = g + (c1.g[i] == 1 ? zeroExtend(c2.g) << i: 0);
    b = b + (c1.b[i] == 1 ? zeroExtend(c2.b) << i : 0);
  end

  return rgb(
    truncateLSB(r),
    truncateLSB(g),
    truncateLSB(b)
  );
endfunction

instance Literal#(Color);
  function Color fromInteger(Integer n);
    return rgb(fromInteger(n), fromInteger(n), fromInteger(n));
  endfunction
endinstance

instance Arith#(Color);
  function Color \+ (Color v1, Color v2) = rgb(v1.r + v2.r, v1.g + v2.g, v1.b + v2.b);
  function Color \- (Color v1, Color v2) = rgb(v1.r - v2.r, v1.g - v2.g, v1.b - v2.b);
  function Color \* (Color v1, Color v2) = multRgb(v1,v2);
  function Color \/ (Color v1, Color v2) = error("/ is undefined for Color");
  function Color \% (Color v1, Color v2) = error("% is undefined for Color");
  function Color negate(Color v) = rgb(-v.r, -v.g, -v.b);
  function Color abs(Color v) = error("abs is undefined for Color");
  function Color signum(Color v) = error("signum is undefined for Color");
  function Color \** (Color v1, Color v2) = error("** is undefined for Color");
  function Color exp_e(Color v) = error("** is undefined for Color");
  function Color log(Color v) = error("log is undefined for Color");
  function Color log2(Color v) = error("log2 is undefined for Color");
  function Color log10(Color v) = error("log10 is undefined for Color");
  function Color logb(Color b, Color v) = error("logb is undefined for Color");
endinstance

function Action write_one_pixel
  (VGA vga, Bit#(32) x, Bit#(32) y, Bit#(8) r, Bit#(8) g, Bit#(8) b) =
  vga.write(x + y * 320, {0,b,g,r}, 4'b0111);

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
  BRAM_DUAL_PORT_BE#(Bit#(32), Bit#(24), 3) bram <- mkBRAMCore2BE(xmax * ymax / 4, False);

  // Fabric registers
  Reg#(Bit#(10)) hpos <- mkReg(0);
  Reg#(Bit#(10)) vpos <- mkReg(0);

  Reg#(Bit#(16)) cycle <- mkReg(0);

  `ifdef BSIM
  rule step_cycle;
    cycle <= cycle + 1;
    if (cycle == 1) drawScreen();
  endrule
  `endif

  Reg#(File) file <- mkReg(InvalidFile);
  Reg#(Bool) started <- mkReg(False);

  rule openFile if (!started);
    `ifdef BSIM
    init();
    `endif
    File f <- $fopen("screen.txt", "w");
    started <= True;
    file <= f;
  endrule

  rule enqBramRead;
    Bit#(20) h = zeroExtend(hpos >> 1);
    Bit#(20) v = zeroExtend(vpos >> 1);

    Bit#(32) ret = zeroExtend(h + v * fromInteger(xmax / 2));
    let addr = ret >= fromInteger(xmax * ymax / 4) ? 0 : ret;

    //$fdisplay(file, "h: %d v: %d addr: %d", h, v, addr);

    bram.a.put(0, addr, ?);
  endrule

  // Write into all the fabric wires
  (* no_implicit_conditions, fire_when_enabled *)
  rule fabric_write;
    let next_hpos = (hpos+1 >= fromInteger(hframe) ? 0 : hpos + 1);
    let next_vpos =
      (hpos+1 >= fromInteger(hframe) ? (vpos+1 >= fromInteger(vframe) ? 0 : vpos+1) : vpos);

    hpos <= next_hpos;
    vpos <= next_vpos;
  endrule

  // Write into the three first bytes
  method Action write(Bit#(32) addr, Bit#(32) data, Bit#(4) mask) if (started);
    if (mask[2:0] != 0)
      bram.b.put(mask[2:0], addr, data[23:0]);

    `ifdef BSIM
    let x = addr % fromInteger(xmax / 2);
    let y = addr / fromInteger(xmax / 2);
    setPoint(x, y, data[7:0], data[15:8], data[23:16]);
    `endif
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

    method red = bram.a.read[7:0];

    method green = bram.a.read[15:8];

    method blue = bram.a.read[23:16];
  endinterface
endmodule
