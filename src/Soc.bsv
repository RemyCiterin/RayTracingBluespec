import Screen :: *;
import Connectable :: *;
import UART :: *;

import Divide :: *;
import SquareRoot :: *;
import ClientServer :: *;
import GetPut :: *;
import Fifo :: *;
import Real :: *;

import Vector :: *;
import BuildVector :: *;

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

function Point3 const3(FP16 x) = point3(x, x, x);

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

function FP16 dot3(Vec3 v1, Vec3 v2) = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

function FP16 det3(Vec3 v1, Vec3 v2, Vec3 v3) = dot3(cross3(v1, v2), v3);

function Vector#(3, Vec3) comatrix3(Vec3 v1, Vec3 v2, Vec3 v3) =
  vec(cross3(v2, v3), cross3(v3, v1), cross3(v1, v2));

function Vec3 apply3(Vec3 m1, Vec3 m2, Vec3 m3, Vec3 x) =
  vec3(
    dot3(vec3(m1.x, m2.x, m3.x), x),
    dot3(vec3(m1.y, m2.y, m3.y), x),
    dot3(vec3(m1.z, m2.z, m3.z), x)
  );

function Vec3 cross3(Vec3 v1, Vec3 v2) =
  vec3(
    v1.y * v2.z - v1.z * v2.y,
    v1.z * v2.x - v1.x * v2.z,
    v1.x * v2.y - v1.y * v2.x
  );

function Vector#(3, Vec3) transpose3(Vec3 v1, Vec3 v2, Vec3 v3) =
  vec(
    vec3(v1.x, v2.x, v3.x),
    vec3(v1.y, v2.y, v3.y),
    vec3(v1.z, v2.z, v3.z)
  );

function Vec3 times(FP16 s, Vec3 v) = vec3(s * v.x, s * v.y, s * v.z);

function Vec3 at(Ray r, FP16 t) = r.origin + times(t, r.direction);

// A module that inverse a 3x3 matrix
module mkInverse3(Server#(Vector#(3, Vec3), Maybe#(Vector#(3, Vec3))));
  Fifo#(4, Bit#(1)) pathQ <- mkFifo;

  Fifo#(4, Vector#(3, Vec3)) comatrixQ <- mkFifo;
  let divider <- mkFP16Divider;

  interface Get response;
    method ActionValue#(Maybe#(Vector#(3,Vec3))) get;
      pathQ.deq;

      if (pathQ.first == 1) begin
        // The matrix is not inversible
        return Invalid;
      end else begin
        comatrixQ.deq;
        let com = comatrixQ.first;
        let m = transpose3(com[0], com[1], com[2]);
        let inv_det <- divider.response.get;

        return Valid(vec(
          m[0] * const3(inv_det),
          m[1] * const3(inv_det),
          m[2] * const3(inv_det)
        ));
      end
    endmethod
  endinterface

  interface Put request;
    method Action put(Vector#(3,Vec3) m);
      action
        let det = det3(m[0], m[1], m[2]);
        let com = comatrix3(m[0], m[1], m[2]);

        if (det == 0) begin
          pathQ.enq(1);
        end else begin
          divider.request.put(tuple2(1, det));
          comatrixQ.enq(com);
          pathQ.enq(0);
        end
      endaction
    endmethod
  endinterface
endmodule

module mkLength(Server#(Vec3, FP16));
  let squareRooter <- mkFixedPointSquareRooter(2);

  interface Get response;
    method ActionValue#(FP16) get;
      match {.x, .*} <- squareRooter.response.get;
      return x;
    endmethod
  endinterface

  interface Put request;
    method Action put(Vec3 v);
      action
        squareRooter.request.put(dot3(v,v));
      endaction
    endmethod
  endinterface
endmodule

module mkFP16Divider(Server#(Tuple2#(FP16,FP16), FP16));
  Server#(Tuple2#(Int#(32), Int#(16)), Tuple2#(Int#(16), Int#(16))) divider <- mkSignedDivider(1);

  interface Put request;
    method Action put(Tuple2#(FP16,FP16) p);
      action
        function Int#(16) toUInt16(FP16 x) = unpack(pack(x));
        function Int#(32) toUInt32(FP16 x) = signExtend(toUInt16(x));
        divider.request.put(tuple2(toUInt32(p.fst) << 8, toUInt16(p.snd)));
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(FP16) get;
      match {.x, .*} <- divider.response.get;
      return unpack(pack(x));
    endmethod
  endinterface
endmodule

typedef struct {
  // True if their is a hit
  Bool found;

  // Normal vector to the surface
  Vec3 normal;

  // The hit position is: ray.center + t * ray.direction
  FP16 t;
} RayHit deriving(Bits, FShow, Eq);

typedef struct {
  //// Center of the triangle
  //Point3 center;

  // Vertex of the triangle
  Vector#(3, Point3) vertex;

  // Momoized normal vector of the triangle
  Vec3 normal;
} Triangle deriving(Bits, FShow, Eq);

module mkIntersectTriangle(Server#(Tuple2#(Ray, Triangle), RayHit));
  let inverse <- mkInverse3;

  Fifo#(4, Tuple2#(Ray, Triangle)) fifo <- mkFifo;

  interface Put request;
    method Action put(Tuple2#(Ray, Triangle) in);
      let edge1 = in.snd.vertex[1] - in.snd.vertex[0];
      let edge2 = in.snd.vertex[2] - in.snd.vertex[0];

      inverse.request.put(vec(edge1,edge2,-in.fst.direction));
      fifo.enq(in);
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(RayHit) get;
      match {.ray, .triangle} = fifo.first;
      fifo.deq;

      let hit = RayHit{
        normal: triangle.normal,
        found: False,
        t: ?
      };

      let inv_opt <- inverse.response.get;

      if (inv_opt matches tagged Valid .inv) begin
        let ret = apply3(inv[0], inv[1], inv[2], ray.origin - triangle.vertex[0]);
        let u = ret.x;
        let v = ret.y;
        hit.t = ret.z;

        hit.found = u >= 0 && v >= 0 && u+v <= 1 && hit.t >= 0;
      end

      return hit;
    endmethod
  endinterface
endmodule

module mkNormalizer(Server#(Vec3, Vec3));
  let divider0 <- mkFP16Divider;
  let divider1 <- mkFP16Divider;
  let divider2 <- mkFP16Divider;
  let length <- mkLength;

  Fifo#(4, Vec3) fifo <- mkFifo;

  rule step;
    fifo.deq;
    let v = fifo.first;
    let len <- length.response.get;

    divider0.request.put(tuple2(v.x, len));
    divider1.request.put(tuple2(v.y, len));
    divider2.request.put(tuple2(v.z, len));
  endrule

  interface Put request;
    method Action put(Vec3 v);
      length.request.put(v);
      fifo.enq(v);
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Vec3) get;
      let x <- divider0.response.get;
      let y <- divider1.response.get;
      let z <- divider2.response.get;
      return vec3(x,y,z);
    endmethod
  endinterface
endmodule

// Return the color of the sky in function of the direction of the rayon
module mkSkyColor(Server#(Ray, Color));
  let normalizer <- mkNormalizer;

  interface Put request;
    method Action put(Ray r);
      normalizer.request.put(r.direction);
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Color) get;
      let n <- normalizer.response.get;

      let a0 = 0.5 * (n.y + 1);
      let a = rgb(a0.f, a0.f, a0.f);
      Color ones = rgb(255, 255, 255);
      return (ones - a) * ones + a * rgb(128, 178, 255);
    endmethod
  endinterface
endmodule

module mkComputeColor(Server#(Ray, Color));
  let sky <- mkSkyColor;
  let hit <- mkIntersectTriangle;

  let triangle = Triangle{
    vertex: vec(vec3(0, 0, -1), vec3(0, 1, -1), vec3(1, 0.1, -1)),
    normal: 0
  };

  interface Put request;
    method Action put(Ray r);
      sky.request.put(r);
      hit.request.put(tuple2(r, triangle));
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Color) get;
      let c <- sky.response.get;
      let h <- hit.response.get;

      if (h.found) return rgb(255,0,0);
      else return c;
    endmethod
  endinterface
endmodule

FP16 focal_length = 1.0;

Point3 camera_center = point3(0, 0, 0);

Real width = 320.0;

Real height = 240.0;

Real viewport_height = 2.0;

Real viewport_width = viewport_height * width / height;

Vec3 viewport_u = vec3(fromReal(viewport_width), 0, 0);
Vec3 viewport_v = vec3(0, -fromReal(viewport_height), 0);

Vec3 pixel_delta_u = vec3(fromReal(viewport_width / width), 0, 0);
Vec3 pixel_delta_v = vec3(0, -fromReal(viewport_height / height), 0);

Point3 viewport_upper_left =
  camera_center - vec3(0, 0, focal_length) - viewport_u/2 - viewport_v/2;
Point3 pixel00_loc =
  viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

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
  TxUART tx_uart <- mkTxUART(217);
  RxUART rx_uart <- mkRxUART(217);
  Reg#(Bit#(8)) led_state <- mkReg(0);
  let vga <- mkVGA;

  Reg#(Bit#(32)) x <- mkReg(0);
  Reg#(Bit#(32)) y <- mkReg(0);

  Fifo#(8, Tuple2#(Bit#(32), Bit#(32))) fifo <- mkFifo;

  let sky <- mkComputeColor;

  Reg#(Bit#(32)) cycle <- mkReg(0);

  rule incr_cycle;
    cycle <= cycle + 1;
  endrule

  rule enq_request;
    // x and y are too big to fit into an FP16 so we divide them first by 16
    FP16 u = FP16{i: truncate(x >> 4), f: {x[3:0], 0}};
    FP16 v = FP16{i: truncate(y >> 4), f: {y[3:0], 0}};

    Vec3 pixel_center =
      pixel00_loc +
      (vec3(u,u,u) * (pixel_delta_u * 16)) +
      (vec3(v,v,v) * (pixel_delta_v * 16));

    fifo.enq(tuple2(x,y));
    sky.request.put(Ray{
      origin: vec3(0,0,0),
      direction: pixel_center - camera_center
    });

    if (x+1 == 320) begin
      y <= y+1 == 240 ? 0 : y+1;
      x <= 0;
    end else
      x <= x + 1;
  endrule

  rule deq_response;
    match {.i, .j} = fifo.first;
    let c <- sky.response.get;
    fifo.deq;

    write_one_pixel(vga, i, j, c.r, c.g, c.b);
  endrule

  method led = led_state;
  method ftdi_rxd = tx_uart.transmit;
  method ftdi_txd = rx_uart.receive;

  interface vga_fab = vga.fabric;
endmodule


(* synthesize *)
module mkCPU_SIM(Empty);
  let cpu <- mkCPU;

  rule rx;
    cpu.ftdi_txd(1);
  endrule
endmodule
