import Screen :: *;
import Connectable :: *;
import UART :: *;

import Divide :: *;
import SquareRoot :: *;
import ClientServer :: *;
import GetPut :: *;
import Fifo :: *;
import Real :: *;
import Ehr :: *;

import Vector :: *;
import BuildVector :: *;

import FixedPoint :: *;

typedef 16 F;
typedef 8 I;
typedef TAdd#(F,I) WIDTH;

typedef FixedPoint#(I,F) F16;

typedef struct {
  F16 x;
  F16 y;
  F16 z;
} Point3
deriving(Bits, FShow, Eq);

function Point3 point3(F16 x, F16 y, F16 z) =
  Point3{x:x, y:y, z:z};

function Point3 const3(F16 x) = point3(x, x, x);

typedef Point3 Vec3;

function Vec3 vec3(F16 x, F16 y, F16 z) = point3(x, y, z);

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

function F16 dot3(Vec3 v1, Vec3 v2) = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

function F16 det3(Vec3 v1, Vec3 v2, Vec3 v3) = dot3(cross3(v1, v2), v3);

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

function Vec3 times(F16 s, Vec3 v) = vec3(s * v.x, s * v.y, s * v.z);

function Vec3 at(Ray r, F16 t) = r.origin + times(t, r.direction);

(* synthesize *)
module mkLength(Server#(Vec3, F16));
  let squareRooter <- mkFixedPointSquareRooter(4);

  interface Get response;
    method ActionValue#(F16) get;
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

(* synthesize *)
module mkF16Divider(Server#(Tuple2#(F16,F16), F16));
  Server#(
    Tuple2#(Int#(TMul#(2,WIDTH)), Int#(WIDTH)),
    Tuple2#(Int#(WIDTH), Int#(WIDTH))) divider <- mkSignedDivider(4);

  interface Put request;
    method Action put(Tuple2#(F16,F16) p);
      action
        function Int#(WIDTH) toUInt16(F16 x) = unpack(pack(x));
        function Int#(TMul#(2,WIDTH)) toUInt32(F16 x) = signExtend(toUInt16(x));
        divider.request.put(tuple2(toUInt32(p.fst) << valueOf(F), toUInt16(p.snd)));
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(F16) get;
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
  F16 t;

  // U-coordinate into the texture file
  F16 u;

  // U-coordinate into the texture file
  F16 v;

  // Texture file id
  Bit#(16) texture_id;

  // Instance id of the triangle
  Bit#(16) instance_id;
} RayHit deriving(Bits, FShow, Eq);

typedef struct {
  // Center of the triangle
  Point3 center;

  // Vertex of the triangle
  Vector#(3, Point3) vertex;

  // Normal vector to each vertex of the triangle, we return it instead of the normal vector to the
  // triangle for a better rendering quality
  Vector#(3, Vec3) normal;

  // Instance of the triangle
  Bit#(16) instance_id;

  // Texture file id
  Bit#(16) texture_id;

  // U-coordinate into the texture file
  Vector#(3, F16) u;

  // V-coordinate into the texture file
  Vector#(3, F16) v;
} Triangle deriving(Bits, FShow, Eq);

(* synthesize *)
module mkDot3(Server#(Tuple2#(Vec3,Vec3), F16));
  Ehr#(2, F16) acc <- mkEhr(0);

  Reg#(Vector#(3, F16)) x1 <- mkReg(replicate(?));
  Reg#(Vector#(3, F16)) x2 <- mkReg(replicate(?));

  Ehr#(2, Bit#(3)) valid <- mkEhr(0);
  Ehr#(2, Bool) done <- mkEhr(False);

  rule step if (valid[0] != 0);
    acc[0] <= acc[0] + x1[0] + x2[0];
    valid[0] <= valid[0] >> 1;
    done[0] <= valid[0] == 1;
    x1 <= rotate(x1);
    x2 <= rotate(x2);
  endrule

  interface Put request;
    method Action put(p) if (valid[1] == 0 && !done[1]);
      action
        valid[1] <= -1;
        acc[1] <= 0;
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(F16) get if (valid[0] == 0 && done[0]);
      done[0] <= False;
      return acc[0];
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkIntersectTriangle(Server#(Tuple2#(Ray, Triangle), RayHit));
  // Rayon-triangle intersection algorithm:
  // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

  let div <- mkF16Divider;

  Fifo#(8, Tuple7#(Ray, Triangle, Bool, Vec3, Vec3, Vec3, Vec3)) fifo <- mkFifo;

  Fifo#(2, Tuple5#(Triangle, Bool, F16, F16, F16)) buffer <- mkFifo;

  rule step;
    match {.ray, .triangle, .not_zero, .s, .q, .h, .edge2} = fifo.first;
    fifo.deq;

    if (not_zero) begin
      let f <- div.response.get;
      let v = f * dot3(ray.direction, q);
      let u = f * dot3(s, h);
      let t = f * dot3(edge2, q);

      Bool found = u >= 0 && v >= 0 && u+v <= 1 && t > 0;
      buffer.enq(tuple5(triangle, found, u, v, t));
    end else begin
      buffer.enq(tuple5(triangle, False, 0, 0, 0));
    end
  endrule

  interface Put request;
    method Action put(Tuple2#(Ray, Triangle) in);
      let edge1 = in.snd.vertex[1] - in.snd.vertex[0];
      let edge2 = in.snd.vertex[2] - in.snd.vertex[0];
      let s = in.fst.origin - in.snd.vertex[0];
      let h = cross3(in.fst.direction, edge2);
      let q = cross3(s, edge1);
      let a = dot3(edge1, h);
      let not_zero = a != 0;

      if (not_zero) div.request.put(tuple2(1, a));
      fifo.enq(tuple7(in.fst, in.snd, not_zero, s, q, h, edge2));
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(RayHit) get;
      match {.triangle, .found, .u, .v, .t} = buffer.first;
      buffer.deq;

      let hit = RayHit{
        instance_id: triangle.instance_id,
        texture_id: triangle.texture_id,
        found: found,
        normal: ?,
        t: t,
        u: ?,
        v: ?
      };

      let w = 1 - (u + v);

      hit.normal =
        const3(u) * triangle.normal[0] +
        const3(v) * triangle.normal[1] +
        const3(w) * triangle.normal[2];

      hit.u =
        u * triangle.u[0] +
        v * triangle.u[1] +
        w * triangle.u[2];

      hit.v =
        u * triangle.v[0] +
        v * triangle.v[1] +
        w * triangle.v[2];

      return hit;
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkNormalizer(Server#(Vec3, Vec3));
  let divider0 <- mkF16Divider;
  let divider1 <- mkF16Divider;
  let divider2 <- mkF16Divider;
  let length <- mkLength;

  Fifo#(8, Vec3) fifo <- mkFifo;

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
(* synthesize *)
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

      let a0 = truncateLSB((0.5 * (n.y + 1)).f);

      let a = rgb(a0, a0, a0);
      Color ones = rgb(255, 255, 255);
      return (ones - a) * ones + a * rgb(128, 178, 255);
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkComputeColor(Server#(Ray, Color));
  let sky <- mkSkyColor;
  let hit <- mkIntersectTriangle;

  let triangle = Triangle{
    vertex: vec(vec3(0, 0, -1), vec3(0.0, 0.5, -1), vec3(0.5, 0, -1)),
    normal: vec(vec3(0,0,-1), vec3(0,0,-1), vec3(0,0,-1)),
    u: vec(0, 0, 1),
    v: vec(0, 1, 0),
    instance_id: 0,
    texture_id: 0,
    center: 0
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

      if (h.found) begin
        let w = 1 - h.u.f - h.v.f;
        return rgb(truncateLSB(h.u.f),truncateLSB(h.v.f),truncateLSB(w));
      end else return c;
    endmethod
  endinterface
endmodule

F16 focal_length = 1.0;

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

Integer log_ray_per_piexel = 3;
Bit#(32) ray_per_pixel = 1 << log_ray_per_piexel;

module mkRandomBit#(Bit#(16) start)(Bit#(1));
  Reg#(Bit#(16)) state <- mkReg(start);

  rule step;
    Bit#(1) b = state[0] ^ state[2] ^ state[3] ^ state[5];
    state <= {b, truncateLSB(state)};
  endrule

  return state[15];
endmodule

(* synthesize *)
module mkSOC(Soc_Ifc);
  TxUART tx_uart <- mkTxUART(217);
  RxUART rx_uart <- mkRxUART(217);
  Reg#(Bit#(8)) led_state <- mkReg(0);
  let vga <- mkVGA;

  Reg#(Bit#(32)) x <- mkReg(0);
  Reg#(Bit#(32)) y <- mkReg(0);
  Reg#(Bit#(32)) count <- mkReg(0);

  Fifo#(8, Tuple3#(Bit#(32), Bit#(32), Bool)) fifo <- mkFifo;

  let sky <- mkComputeColor;

  Reg#(Bit#(32)) cycle <- mkReg(0);
  Reg#(Bit#(32)) frame <- mkReg(0);

  Bit#(1) random1 <- mkRandomBit(16'h7CE1);
  Bit#(1) random2 <- mkRandomBit(16'h0CE1);
  Bit#(1) random3 <- mkRandomBit(16'hBCE1);
  Bit#(1) random4 <- mkRandomBit(16'hACF1);
  Bit#(4) r1 = {random1,random2,random3,random4};

  Bit#(1) random5 <- mkRandomBit(16'h7CE9);
  Bit#(1) random6 <- mkRandomBit(16'h0C31);
  Bit#(1) random7 <- mkRandomBit(16'hBC61);
  Bit#(1) random8 <- mkRandomBit(16'hAC21);
  Bit#(4) r2 = {random5,random6,random7,random8};

  rule incr_cycle;
    cycle <= cycle + 1;
  endrule

  rule enq_request;
    // x and y are too big to fit into an F16 so we divide them first by 16
    F16 u = F16{i: truncate(x >> 4), f: {x[3:0], r1, 0}};
    F16 v = F16{i: truncate(y >> 4), f: {y[3:0], r2, 0}};

    Vec3 pixel_center =
      pixel00_loc +
      (vec3(u,u,u) * (pixel_delta_u * 16)) +
      (vec3(v,v,v) * (pixel_delta_v * 16));

    fifo.enq(tuple3(x,y,count+1==ray_per_pixel));
    sky.request.put(Ray{
      origin: vec3(0,0,0),
      direction: pixel_center - camera_center
    });

    if (count+1 == ray_per_pixel) begin
      count <= 0;

      if (x+1 == 320) begin
        y <= y+1 == 240 ? 0 : y+1;
        x <= 0;

        if (y+1 == 240) begin
          $display("cycle: %d frame: %d", cycle, frame);
          frame <= frame + 1;
        end
      end else
        x <= x + 1;
    end else begin
      count <= count + 1;
    end
  endrule

  Reg#(Vector#(3,Bit#(16))) current_color <- mkReg(vec(0,0,0));
  rule deq_response;
    match {.i, .j, .last} = fifo.first;
    let c <- sky.response.get;
    fifo.deq;

    Vector#(3,Bit#(16)) color = vec(
      current_color[0] + zeroExtend(c.r),
      current_color[1] + zeroExtend(c.g),
      current_color[2] + zeroExtend(c.b)
    );

    if (last) begin
      write_one_pixel(
        vga, i, j,
        (color[0] >> log_ray_per_piexel)[7:0],
        (color[1] >> log_ray_per_piexel)[7:0],
        (color[2] >> log_ray_per_piexel)[7:0]
      );

      current_color <= vec(0, 0, 0);
    end else begin
      current_color <= color;
    end
  endrule

  method led = led_state;
  method ftdi_rxd = tx_uart.transmit;
  method ftdi_txd = rx_uart.receive;

  interface vga_fab = vga.fabric;
endmodule


(* synthesize *)
module mkSOC_SIM(Empty);
  let cpu <- mkSOC;

  rule rx;
    cpu.ftdi_txd(1);
  endrule
endmodule
