import Screen :: *;
import Connectable :: *;
import UART :: *;

import FixedPoint :: *;

typedef FixedPoint#(8,8) FP16;

typedef struct {
  FP16 x;
  FP16 y;
  FP16 z;
} Point3
deriving(Bits, FShow, Eq);

function Point3 point3(FP16 x, FP16 y, FP16 z) =
  Point3{x:x, y:y, z:z};

typedef Point3 Vec3;

function Vec3 vec3(FP16 x, FP16 y, FP16 z) = point3(x, y, z);

typedef struct {
  Point3 origin;
  Vec3 direction;
} Ray deriving(Bits, FShow, Eq);

instance Literal#(Vec3);
  function Vec3 fromInteger(Integer n);
    return vec3(fromInteger(n), fromInteger(n), fromInteger(n));
  endfunction
endinstance

instance RealLiteral#(Vec3);
  function Vec3 fromReal(Real n) = vec3(fromReal(n), fromReal(n), fromReal(n));
endinstance

instance Arith#(Vec3);
  function Vec3 \+ (Vec3 v1, Vec3 v2) = vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
  function Vec3 \- (Vec3 v1, Vec3 v2) = vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
  function Vec3 \* (Vec3 v1, Vec3 v2) = vec3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
  function Vec3 \/ (Vec3 v1, Vec3 v2) = vec3(v1.x / v2.x, v1.y / v2.y, v1.z / v2.z);
  function Vec3 \% (Vec3 v1, Vec3 v2) = error("% is undefined for Vec3");
  function Vec3 negate(Vec3 v) = vec3(-v.x, -v.y, -v.z);
  function Vec3 abs(Vec3 v) = error("abs is undefined for Vec3");
  function Vec3 signum(Vec3 v) = error("signum is undefined for Vec3");
  function Vec3 \** (Vec3 v1, Vec3 v2) = error("** is undefined for Vec3");
  function Vec3 exp_e(Vec3 v) = error("** is undefined for Vec3");
  function Vec3 log(Vec3 v) = error("log is undefined for Vec3");
  function Vec3 log2(Vec3 v) = error("log2 is undefined for Vec3");
  function Vec3 log10(Vec3 v) = error("log10 is undefined for Vec3");
  function Vec3 logb(Vec3 b, Vec3 v) = error("logb is undefined for Vec3");
endinstance

function FP16 dot(Vec3 v1, Vec3 v2) = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

function Vec3 times(FP16 s, Vec3 v) = vec3(s * v.x, s * v.y, s * v.z);

FP16 focal_length = 1.0;

Point3 camera_center = point3(0, 0, 0);

interface Soc_Ifc;
  (* always_ready, always_enabled *)
  method Bit#(8) led;

  (* always_ready, always_enabled *)
  method Bit#(1) ftdi_rxd;

  (* always_ready, always_enabled, prefix="" *)
  method Action ftdi_txd((* port="ftdi_txd" *) Bit#(1) value);

  (* prefix="" *)
  interface VGAFabric vga_fab;
endinterface

(* synthesize *)
module mkCPU(Soc_Ifc);
  let vga <- mkVGA;

  // Write a single pixel on the screen
  function Action write_pixel(Bit#(32) x, Bit#(32) y, Bit#(8) r, Bit#(8) g, Bit#(8) b) =
    vga.write((x >> 2) + y * 80, zeroExtend(pack(fromRGB(r,g,b))) << {x[1:0], 3'b000}, 1 << x[1:0]);

  Reg#(Bit#(32)) x <- mkReg(0);
  Reg#(Bit#(32)) y <- mkReg(0);

  rule write_screen;
    Color16 sky = color16(6'b100000, 6'b110000, 4'b1111);

    Color16 a = color16(y[5:0], y[5:0], y[3:0]);

    Color16 ones = color16(-1, -1, -1);

    write_one_color16(vga, x, y, (ones - a) * ones + a * sky);

    if (x+1 == 320) begin
      y <= y+1 == 240 ? 0 : y+1;
      x <= 0;
    end else
      x <= x + 1;
  endrule

  TxUART tx_uart <- mkTxUART(217);
  RxUART rx_uart <- mkRxUART(217);

  Reg#(Bit#(8)) led_state <- mkReg(0);

  method led = led_state;
  method ftdi_rxd = tx_uart.transmit;
  method ftdi_txd = rx_uart.receive;

  interface vga_fab = vga.fabric;
endmodule
