import Connectable :: *;

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
  Vec3 inv_direction;
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

(* synthesize *)
module mkVec3Divider(Server#(Tuple2#(Vec3,Vec3), Vec3));
  let div1 <- mkF16Divider;
  let div2 <- mkF16Divider;
  let div3 <- mkF16Divider;

  interface Put request;
    method Action put(Tuple2#(Vec3,Vec3) p);
      action
        div1.request.put(tuple2(p.fst.x, p.snd.x));
        div2.request.put(tuple2(p.fst.y, p.snd.y));
        div3.request.put(tuple2(p.fst.z, p.snd.z));
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Vec3) get;
      Vec3 ret;
      ret.x <- div1.response.get;
      ret.y <- div2.response.get;
      ret.z <- div3.response.get;
      return ret;
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

typedef struct {
  Vec3 min;
  Vec3 max;
} Box deriving(Bits, FShow, Eq);

(* synthesize *)
module mkCross3(Server#(Tuple2#(Vec3,Vec3),Vec3));
  Ehr#(2, Bit#(6)) xValid <- mkEhr(0);
  Ehr#(2, Vector#(6, F16)) x1 <- mkEhr(replicate(?));
  Ehr#(2, Vector#(6, F16)) x2 <- mkEhr(replicate(?));

  Ehr#(2, Bit#(2)) yValid <- mkEhr(0);
  Ehr#(2, Vector#(2, F16)) y <- mkEhr(replicate(?));

  Ehr#(2, Bit#(3)) zValid <- mkEhr(0);
  Ehr#(2, Vector#(3, F16)) z <- mkEhr(replicate(?));

  rule enq_multiplier if (xValid[0][0] == 1 && yValid[1][0] == 0);
    yValid[1] <= {1'b1, truncateLSB(yValid[1])};
    y[1] <= vec(y[1][1], x1[0][0] * x2[0][0]);

    xValid[0] <= xValid[0] >> 1;
    x1[0] <= rotate(x1[0]);
    x2[0] <= rotate(x2[0]);
  endrule

  rule enq_adder if (yValid[0] == 2'b11 && zValid[1][0] == 0);
    zValid[1] <= {1'b1, truncateLSB(zValid[1])};
    z[1] <= vec(z[1][1], z[1][2], y[0][0] - y[0][1]);
    yValid[0] <= 0;
  endrule

  interface Put request;
    method Action put(Tuple2#(Vec3,Vec3) p) if (xValid[1] == 0);
      action
        match {.a, .b} = p;
        x1[1] <= vec(a.y, a.z, a.z, a.x, a.x, a.y);
        x2[1] <= vec(b.z, b.y, b.x, b.z, b.y, b.x);
        xValid[1] <= -1;
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Vec3) get if (zValid[0] == 3'b111);
      zValid[0] <= 0;
      return vec3(z[0][0], z[0][1], z[0][2]);
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkDot3(Server#(Tuple2#(Vec3,Vec3), F16));
  Ehr#(2, F16) acc <- mkEhr(0);

  Reg#(Vector#(3, F16)) x1 <- mkReg(replicate(?));
  Reg#(Vector#(3, F16)) x2 <- mkReg(replicate(?));

  Ehr#(2, Bit#(3)) valid <- mkEhr(0);
  Ehr#(2, Bool) done <- mkEhr(False);

  Fifo#(4, F16) outputs <- mkFifo;

  rule step if (valid[0] != 0);
    let res = acc[0] + x1[0] * x2[0];
    valid[0] <= valid[0] >> 1;
    done[0] <= valid[0] == 1;
    x1 <= rotate(x1);
    x2 <= rotate(x2);
    acc[0] <= res;

    if (valid[0] == 1)
      outputs.enq(res);
  endrule

  interface Put request;
    method Action put(p) if (valid[1] == 0);
      action
        x1 <= vec(p.fst.x, p.fst.y, p.fst.z);
        x2 <= vec(p.snd.x, p.snd.y, p.snd.z);
        valid[1] <= -1;
        acc[1] <= 0;
      endaction
    endmethod
  endinterface

  interface response = toGet(outputs);
endmodule

(* synthesize *)
module mkIntersectTriangle(Server#(Tuple2#(Ray, Triangle), RayHit));
  // Rayon-triangle intersection algorithm:
  // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

  let div <- mkF16Divider;

  Fifo#(8, Tuple2#(Ray, Triangle)) fifo <- mkFifo;
  Fifo#(4, Bool) notZeroQ <- mkFifo;

  Fifo#(4, Tuple5#(Triangle, Bool, F16, F16, F16)) buffer <- mkFifo;

  let dot_prod1 <- mkDot3;
  let dot_prod2 <- mkDot3;
  let dot_prod3 <- mkDot3;

  let dot_prod4 <- mkDot3;
  let dot_prod5 <- mkDot3;
  let dot_prod6 <- mkDot3;
  let dot_prod7 <- mkDot3;
  let dot_prod8 <- mkDot3;
  let dot_prod9 <- mkDot3;

  rule step;
    let not_zero <- toGet(notZeroQ).get;
    match {.ray, .triangle} = fifo.first;
    fifo.deq;

    let v_div_f <- dot_prod1.response.get;
    let u_div_f <- dot_prod2.response.get;
    let t_div_f <- dot_prod3.response.get;
    let f <- div.response.get;

    let v = f * v_div_f;
    let u = f * u_div_f;
    let t = f * t_div_f;
    let w = 1 - (u + v);

    dot_prod4.request.put(tuple2(
        vec3(u,v,w),
        vec3(triangle.u[0], triangle.u[1], triangle.u[2])
    ));

    dot_prod5.request.put(tuple2(
        vec3(u,v,w),
        vec3(triangle.v[0], triangle.v[1], triangle.v[2])
    ));

    let normal_t = transpose3(
      triangle.normal[0],
      triangle.normal[1],
      triangle.normal[2]
    );

    dot_prod6.request.put(tuple2(vec3(u,v,w), normal_t[0]));
    dot_prod7.request.put(tuple2(vec3(u,v,w), normal_t[1]));
    dot_prod8.request.put(tuple2(vec3(u,v,w), normal_t[2]));

    Bool found = not_zero && u >= 0 && v >= 0 && u+v <= 1 && t > 0;
    buffer.enq(tuple5(triangle, found, u, v, t));
  endrule

  rule compute_a;
      let a <- dot_prod9.response.get;
      let not_zero = a != 0;

      notZeroQ.enq(not_zero);

      div.request.put(tuple2(1, a));
  endrule

  Fifo#(4, Tuple4#(Vec3, Vec3, Vec3, Vec3)) stage12 <- mkFifo;
  let cross_prod1 <- mkCross3;
  let cross_prod2 <- mkCross3;

  rule stage2;
    match {.dir, .edge1, .edge2, .s} <- toGet(stage12).get;
    let h <- cross_prod1.response.get;
    let q <- cross_prod2.response.get;

    dot_prod1.request.put(tuple2(dir, q));
    dot_prod2.request.put(tuple2(s, h));
    dot_prod3.request.put(tuple2(edge2, q));
    dot_prod9.request.put(tuple2(edge1, h));
  endrule

  interface Put request;
    method Action put(Tuple2#(Ray, Triangle) in);
      let edge1 = in.snd.vertex[1] - in.snd.vertex[0];
      let edge2 = in.snd.vertex[2] - in.snd.vertex[0];
      let s = in.fst.origin - in.snd.vertex[0];

      cross_prod1.request.put(tuple2(in.fst.direction, edge2));
      cross_prod2.request.put(tuple2(s, edge1));

      stage12.enq(tuple4(in.fst.direction, edge1, edge2, s));

      fifo.enq(tuple2(in.fst, in.snd));
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

      hit.normal.x <- dot_prod6.response.get;
      hit.normal.y <- dot_prod7.response.get;
      hit.normal.z <- dot_prod8.response.get;

      hit.u <- dot_prod4.response.get;

      hit.v <- dot_prod5.response.get;

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
