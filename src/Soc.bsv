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
import FloatingPoint :: *;

typedef FloatingPoint#(5,10) F16;

RoundMode round = Rnd_Zero;

function F16 int16ToF16(Int#(16) x);
  match {.f, .*} = vFixedToFloat(x, 6'b0, round);
  return f;
endfunction

function Int#(16) f16ToInt16(F16 f);
  match {.x, .*} = vFloatToFixed(6'b0, f, round);
  return x;
endfunction

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
  let sqrt_raw <- mkSquareRooter(8);
  let squareRooter <- mkFloatingPointSquareRooter(sqrt_raw);

  let dot <- mkDot3;

  rule stage2;
    let square <- dot.response.get;
    squareRooter.request.put(tuple2(square, round));
  endrule

  interface Get response;
    method ActionValue#(F16) get;
      match {.x, .*} <- squareRooter.response.get;
      return x;
    endmethod
  endinterface

  interface Put request;
    method Action put(Vec3 v);
      action
        dot.request.put(tuple2(v,v));
      endaction
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkF16Adder(Server#(Tuple2#(F16,F16),F16));
  let adder <- mkFloatingPointAdder;

  interface Put request;
    method Action put(Tuple2#(F16,F16) p);
      action
        adder.request.put(tuple3(p.fst,p.snd,round));
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(F16) get;
      match {.x, .*} <- adder.response.get;
      return unpack(pack(x));
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkVec3Adder(Server#(Tuple2#(Vec3,Vec3),Vec3));
  let add1 <- mkF16Adder;
  let add2 <- mkF16Adder;
  let add3 <- mkF16Adder;

  interface Get response;
    method ActionValue#(Vec3) get;
      let x <- add1.response.get;
      let y <- add2.response.get;
      let z <- add3.response.get;

      return vec3(x,y,z);
    endmethod
  endinterface

  interface Put request;
    method put(p);
      action
        add1.request.put(tuple2(p.fst.x,p.snd.x));
        add2.request.put(tuple2(p.fst.y,p.snd.y));
        add3.request.put(tuple2(p.fst.z,p.snd.z));
      endaction
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkVec3Multiplier(Server#(Tuple2#(Vec3,Vec3),Vec3));
  let mul1 <- mkF16Multiplier;
  let mul2 <- mkF16Multiplier;
  let mul3 <- mkF16Multiplier;

  interface Get response;
    method ActionValue#(Vec3) get;
      let x <- mul1.response.get;
      let y <- mul2.response.get;
      let z <- mul3.response.get;

      return vec3(x,y,z);
    endmethod
  endinterface

  interface Put request;
    method put(p);
      action
        mul1.request.put(tuple2(p.fst.x,p.snd.x));
        mul2.request.put(tuple2(p.fst.y,p.snd.y));
        mul3.request.put(tuple2(p.fst.z,p.snd.z));
      endaction
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkDot3(Server#(Tuple2#(Vec3,Vec3),F16));
  Reg#(Vector#(3, F16)) x1 <- mkReg(replicate(?));
  Reg#(Vector#(3, F16)) x2 <- mkReg(replicate(?));

  Reg#(Bit#(2)) state <- mkReg(0);
  Reg#(F16) acc <- mkReg(?);

  Reg#(Bit#(3)) valid <- mkReg(0);
  let fma <- mkFMA;

  rule get_result if (state == 2);
    let a <- fma.response.get;
    state <= 1;
    acc <= a;
  endrule

  rule step if (valid[0] == 1 && state == 1);
    fma.request.put(tuple3(x1[0], x2[0], acc));
    valid <= valid >> 1;

    x1 <= rotate(x1);
    x2 <= rotate(x2);

    state <= 2;
  endrule

  interface Put request;
    method Action put(Tuple2#(Vec3,Vec3) args) if (valid == 0 && state == 0);
      action
        match {.a, .b} = args;
        x1 <= vec(a.x, a.y, a.z);
        x2 <= vec(b.x, b.y, b.z);
        valid <= -1;
        state <= 1;
        acc <= 0;
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(F16) get if (valid == 0 && state == 1);
      state <= 0;

      return acc;
    endmethod
  endinterface
endmodule

//(* synthesize *)
//module mkDot3(Server#(Tuple2#(Vec3,Vec3),F16));
//  Fifo#(32,Tuple2#(F16,F16)) fifo <- mkFifo;
//  let mul1 <- mkF16Multiplier;
//  let mul2 <- mkF16Multiplier;
//  let add <- mkF16Adder;
//  let fma <- mkFMA;
//
//  rule s1;
//    let x <- mul1.response.get;
//    let y <- mul2.response.get;
//    add.request.put(tuple2(x,y));
//  endrule
//
//  rule s2;
//    fifo.deq;
//    let xy <- add.response.get;
//    match {.z1,.z2} = fifo.first;
//    fma.request.put(tuple3(z1,z2,xy));
//  endrule
//
//  interface Put request;
//    method Action put(Tuple2#(Vec3,Vec3) p);
//      action
//        mul1.request.put(tuple2(p.fst.x, p.snd.x));
//        mul2.request.put(tuple2(p.fst.y, p.snd.y));
//        fifo.enq(tuple2(p.fst.z, p.snd.z));
//      endaction
//    endmethod
//  endinterface
//
//  interface response = fma.response;
//endmodule

(* synthesize *)
module mkCross3(Server#(Tuple2#(Vec3,Vec3),Vec3));
  let multiplier <- mkF16Multiplier;
  let adder <- mkF16Adder;

  Reg#(Bit#(6)) xValid <- mkReg(0);
  Reg#(Vector#(6, F16)) x1 <- mkReg(replicate(?));
  Reg#(Vector#(6, F16)) x2 <- mkReg(replicate(?));

  Reg#(Bit#(2)) yValid <- mkReg(0);
  Reg#(Vector#(2, F16)) y <- mkReg(replicate(?));

  Reg#(Bit#(3)) zValid <- mkReg(0);
  Reg#(Vector#(3, F16)) z <- mkReg(replicate(?));

  rule enq_multiplier if (xValid[0] == 1);
    multiplier.request.put(tuple2(x1[0], x2[0]));
    xValid <= xValid >> 1;
    x1 <= rotate(x1);
    x2 <= rotate(x2);
  endrule

  rule deq_multiplier if (yValid[0] == 0);
    yValid <= {1'b1, truncateLSB(yValid)};
    let x12 <- multiplier.response.get;
    y <= vec(y[1], x12);
  endrule

  rule enq_adder if (yValid == 2'b11);
    adder.request.put(tuple2(y[0], negate(y[1])));
    yValid <= 0;
  endrule

  rule deq_adder if (zValid[0] == 0);
    zValid <= {1'b1, truncateLSB(zValid)};
    let y12 <- adder.response.get;

    z <= vec(z[1], z[2], y12);
  endrule

  interface Put request;
    method Action put(Tuple2#(Vec3,Vec3) p) if (xValid == 0);
      action
        match {.a, .b} = p;
        x1 <= vec(a.y, a.z, a.z, a.x, a.x, a.y);
        x2 <= vec(b.z, b.y, b.x, b.z, b.y, b.x);
        xValid <= -1;
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Vec3) get if (zValid == 3'b111);
      zValid <= 0;
      return vec3(z[0], z[1], z[2]);
    endmethod
  endinterface
endmodule

//(* synthesize *)
//module mkCross3(Server#(Tuple2#(Vec3,Vec3),Vec3));
//  let mul1 <- mkF16Multiplier;
//  let mul2 <- mkF16Multiplier;
//  let mul3 <- mkF16Multiplier;
//  let mul4 <- mkF16Multiplier;
//  let mul5 <- mkF16Multiplier;
//  let mul6 <- mkF16Multiplier;
//  let add1 <- mkF16Adder;
//  let add2 <- mkF16Adder;
//  let add3 <- mkF16Adder;
//
//  rule step12;
//    let x <- mul1.response.get;
//    let y <- mul2.response.get;
//    add1.request.put(tuple2(x,negate(y)));
//  endrule
//
//  rule step34;
//    let x <- mul3.response.get;
//    let y <- mul4.response.get;
//    add2.request.put(tuple2(x,negate(y)));
//  endrule
//
//  rule step56;
//    let x <- mul5.response.get;
//    let y <- mul6.response.get;
//    add3.request.put(tuple2(x,negate(y)));
//  endrule
//
//  interface Put request;
//    method Action put(Tuple2#(Vec3,Vec3) p);
//      action
//        mul1.request.put(tuple2(p.fst.y, p.snd.z));
//        mul2.request.put(tuple2(p.fst.z, p.snd.y));
//
//        mul3.request.put(tuple2(p.fst.z, p.snd.x));
//        mul4.request.put(tuple2(p.fst.x, p.snd.z));
//
//        mul5.request.put(tuple2(p.fst.x, p.snd.y));
//        mul6.request.put(tuple2(p.fst.y, p.snd.x));
//      endaction
//    endmethod
//  endinterface
//
//  interface Get response;
//    method ActionValue#(Vec3) get;
//      let x <- add1.response.get;
//      let y <- add2.response.get;
//      let z <- add3.response.get;
//
//      return vec3(x,y,z);
//    endmethod
//  endinterface
//endmodule

// Compute a*b+c for each request tuple3(a,b,c)
(* synthesize *)
module mkFMA(Server#(Tuple3#(F16,F16,F16),F16));
  let fma <- mkFloatingPointFusedMultiplyAccumulate;

  interface Put request;
    // compute a * b + c
    method Action put(Tuple3#(F16,F16,F16) p);
      action
        match {.a,.b,.c} = p;
        fma.request.put(tuple4(Valid(c),a,b,round));
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(F16) get;
      match {.x, .*} <- fma.response.get;
      return unpack(pack(x));
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkF16Multiplier(Server#(Tuple2#(F16,F16), F16));
  let multiplier <- mkFloatingPointMultiplier;

  interface Put request;
    method Action put(Tuple2#(F16,F16) p);
      action
        multiplier.request.put(tuple3(p.fst,p.snd,round));
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(F16) get;
      match {.x, .*} <- multiplier.response.get;
      return unpack(pack(x));
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkF16Divider(Server#(Tuple2#(F16,F16), F16));
  let div <- mkDivider(4);
  let divider <- mkFloatingPointDivider(div);

  interface Put request;
    method Action put(Tuple2#(F16,F16) p);
      action
        divider.request.put(tuple3(p.fst,p.snd,round));
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
module mkIntersectTriangleRaw
  (Server#(Tuple2#(Ray, Triangle), Maybe#(Tuple3#(F16,F16,F16))));
  let cross1 <- mkCross3;
  let cross2 <- mkCross3;

  let dot1 <- mkDot3;
  let dot2 <- mkDot3;
  let dot3 <- mkDot3;
  let dot4 <- mkDot3;

  let div <- mkF16Divider;

  let add1 <- mkVec3Adder;
  let add2 <- mkVec3Adder;
  let add3 <- mkVec3Adder;

  let mul1 <- mkF16Multiplier;
  let mul2 <- mkF16Multiplier;
  let mul3 <- mkF16Multiplier;

  Fifo#(18,Vec3) edge1_queue1 <- mkFifo;

  Fifo#(18,Vec3) edge2_queue1 <- mkFifo;

  Fifo#(18,Vec3) s_queue1 <- mkFifo;

  Fifo#(18,Vec3) q_queue1 <- mkFifo;

  Fifo#(18,Vec3) h_queue1 <- mkFifo;

  Fifo#(18,F16) a_queue1 <- mkFifo;

  Fifo#(18,Vec3) d_queue1 <- mkFifo;
  Fifo#(18,Vec3) d_queue2 <- mkFifo;

  rule stage2;
    let edge1 <- add1.response.get;
    edge1_queue1.enq(edge1);

    let s <- add3.response.get;
    s_queue1.enq(s);

    let edge2 <- add2.response.get;
    edge2_queue1.enq(edge2);

    d_queue1.deq;
    let d = d_queue1.first;

    cross1.request.put(tuple2(s, edge1));
    cross2.request.put(tuple2(d, edge2));
  endrule

  rule stage3;
    let q <- cross1.response.get;
    let h <- cross2.response.get;

    q_queue1.enq(q);

    h_queue1.enq(h);

    edge1_queue1.deq;
    let edge1 = edge1_queue1.first;

    dot1.request.put(tuple2(edge1, h));
  endrule

  rule stage4;
    let a <- dot1.response.get;
    div.request.put(tuple2(1,a));
    a_queue1.enq(a);

    edge2_queue1.deq;
    let edge2 = edge2_queue1.first;

    q_queue1.deq;
    let q = q_queue1.first;

    d_queue2.deq;
    let d = d_queue2.first;

    s_queue1.deq;
    let s = s_queue1.first;

    h_queue1.deq;
    let h = h_queue1.first;

    dot2.request.put(tuple2(d,q));
    dot3.request.put(tuple2(s,h));
    dot4.request.put(tuple2(edge2,q));
  endrule

  rule stage5;
    let f <- div.response.get;

    let u_div_f <- dot2.response.get;
    let v_div_f <- dot3.response.get;
    let t_div_f <- dot4.response.get;

    mul1.request.put(tuple2(f, u_div_f));
    mul2.request.put(tuple2(f, v_div_f));
    mul3.request.put(tuple2(f, t_div_f));
  endrule

  interface Put request;
    method Action put(Tuple2#(Ray, Triangle) p);
      action
        match {.r, .t} = p;
        add1.request.put(tuple2(t.vertex[1], negate(t.vertex[0])));
        add2.request.put(tuple2(t.vertex[2], negate(t.vertex[0])));
        add3.request.put(tuple2(r.origin, negate(t.vertex[0])));

        d_queue1.enq(r.direction);
        d_queue2.enq(r.direction);
      endaction
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Maybe#(Tuple3#(F16,F16,F16))) get;
      F16 eps = 0.001;

      a_queue1.deq;
      let a = a_queue1.first;

      let is_zero = compareFP(a, eps) == LT && compareFP(-eps, a) == LT;

      let u <- mul1.response.get;
      let v <- mul2.response.get;
      let t <- mul3.response.get;

      return is_zero ? Invalid : Valid(tuple3(u,v,t));
    endmethod
  endinterface
endmodule

(* synthesize *)
module mkIntersectTriangle(Server#(Tuple2#(Ray, Triangle), RayHit));
  let raw <- mkIntersectTriangleRaw;

  let add1 <- mkF16Adder;
  let add2 <- mkF16Adder;
  //let add3 <- mkVec3Adder;
  //let add4 <- mkVec3Adder;

  //let dot1 <- mkDot3;
  //let dot2 <- mkDot3;

  Fifo#(8, Maybe#(Tuple3#(F16,F16,F16))) stage_2_4 <- mkFifo;
  Fifo#(8, Maybe#(F16)) stage_4_end <- mkFifo;

  Fifo#(16, Tuple2#(Ray, Triangle)) requests1 <- mkFifo;
  Fifo#(16, Tuple2#(Ray, Triangle)) requests2 <- mkFifo;

  //let mul1 <- mkVec3Multiplier;
  //let mul2 <- mkVec3Multiplier;
  //let mul3 <- mkVec3Multiplier;

  rule stage2;
    let result_opt <- raw.response.get;
    stage_2_4.enq(result_opt);

    if (result_opt matches tagged Valid {.u,.v,.*}) begin
      add1.request.put(tuple2(u,v));
    end
  endrule

  rule stage3;
    let uv <- add1.response.get;
    add2.request.put(tuple2(1, negate(uv)));
  endrule

  rule stage4;
    requests1.deq;
    stage_2_4.deq;
    match {.ray, .triangle} = requests1.first;
    let result_opt = stage_2_4.first;
    requests2.enq(tuple2(ray, triangle));

    if (result_opt matches tagged Valid {.u, .v, .t}) begin
      let w <- add2.response.get;

      let found =
        compareFP(u,0) != LT &&
        compareFP(v,0) != LT &&
        compareFP(w,0) != LT &&
        compareFP(0,t) == LT;

      if (found) begin
        stage_4_end.enq(Valid(t));

        //mul1.request.put(tuple2(const3(u), triangle.normal[0]));
        //mul2.request.put(tuple2(const3(v), triangle.normal[0]));
        //mul3.request.put(tuple2(const3(w), triangle.normal[0]));

        //dot1.request.put(tuple2(
        //  vec3(triangle.u[0], triangle.u[1], triangle.u[2]),
        //  vec3(u,v,w)
        //));

        //dot2.request.put(tuple2(
        //  vec3(triangle.v[0], triangle.v[1], triangle.v[2]),
        //  vec3(u,v,w)
        //));
      end else begin
        stage_4_end.enq(Invalid);
      end

    end else begin
      stage_4_end.enq(Invalid);
    end
  endrule

  //rule stage5;
  //  let x <- mul1.response.get;
  //  let y <- mul2.response.get;
  //  add3.request.put(tuple2(x,y));
  //endrule

  //rule stage6;
  //  let x <- add3.response.get;
  //  let y <- mul3.response.get;
  //  add4.request.put(tuple2(x,y));
  //endrule

  interface Put request;
    method Action put(Tuple2#(Ray, Triangle) in);
      raw.request.put(in);
      requests1.enq(in);
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(RayHit) get;
      match {.ray, .triangle} = requests2.first;
      requests2.deq;

      let hit = RayHit{
        instance_id: triangle.instance_id,
        texture_id: triangle.texture_id,
        found: False,
        normal: ?,
        t: ?,
        u: ?,
        v: ?
      };

      stage_4_end.deq;
      let result_opt = stage_4_end.first;

      if (result_opt matches tagged Valid .t) begin
        //hit.normal <- add4.response.get;
        //hit.u <- dot1.response.get;
        //hit.v <- dot2.response.get;
        hit.found = True;

        hit.t = t;
      end

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

  Fifo#(16, Vec3) fifo <- mkFifo;

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

  let add <- mkF16Adder;
  let mul <- mkF16Multiplier;

  rule stage2;
    let n <- normalizer.response.get;
    add.request.put(tuple2(1, n.y));
  endrule

  rule stage3;
    let x <- add.response.get;
    mul.request.put(tuple2(128, x));
  endrule

  interface Put request;
    method Action put(Ray r);
      normalizer.request.put(r.direction);
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Color) get;

      let a0 <- mul.response.get;
      let a1 = truncate(pack(f16ToInt16(a0)));

      let a = rgb(a1, a1, a1);
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
        return rgb(255,255,255);
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

Integer log_ray_per_piexel = 0;
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
    F16 u = int16ToF16(unpack(truncate(x)));
    F16 v = int16ToF16(unpack(truncate(y)));

    Vec3 pixel_center =
      pixel00_loc +
      (vec3(u,u,u) * pixel_delta_u) +
      (vec3(v,v,v) * pixel_delta_v);

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
