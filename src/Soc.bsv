import Screen :: *;
import Connectable :: *;
import UART :: *;

import BRAMCore :: *;

import ClientServer :: *;
import GetPut :: *;
import Fifo :: *;
import Real :: *;
import Ehr :: *;

import Vector :: *;
import BuildVector :: *;

import FixedPoint :: *;
import Geometry :: *;

import Bvh :: *;
import StmtFSM :: *;

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
  let tree <- mkTree;

  Reg#(Bit#(4)) state <- mkReg(0);

  BRAM_PORT#(Bit#(16), Bit#(32)) triangles <-
    mkBRAMCore1Load(65536, False, "Mem.hex", False);

  Reg#(Vector#(3, Vec3)) triangle <- mkReg(replicate(?));

  Reg#(Bit#(16)) triangle_index <- mkReg(0);
  let divider <- mkVec3Divider;

  let loadStmt = seq
    while (triangle_index != 3488) seq
      triangles.put(False, triangle_index*9, ?);
      triangle[0].x <= unpack(triangles.read);
      triangles.put(False, triangle_index*9+1, ?);
      triangle[0].y <= unpack(triangles.read);
      triangles.put(False, triangle_index*9+2, ?);
      triangle[0].z <= unpack(triangles.read);
      triangles.put(False, triangle_index*9+3, ?);
      triangle[1].x <= unpack(triangles.read);
      triangles.put(False, triangle_index*9+4, ?);
      triangle[1].y <= unpack(triangles.read);
      triangles.put(False, triangle_index*9+5, ?);
      triangle[1].z <= unpack(triangles.read);
      triangles.put(False, triangle_index*9+6, ?);
      triangle[2].x <= unpack(triangles.read);
      triangles.put(False, triangle_index*9+7, ?);
      triangle[2].y <= unpack(triangles.read);
      triangles.put(False, triangle_index*9+8, ?);
      triangle[2].z <= unpack(triangles.read);

      divider.request.put(
        tuple2(triangle[0] + triangle[1] + triangle[2], 3)
      );

      action
        let off = vec3(0,0,-7);

        $display("\ntriangle: %d", triangle_index);
        $display("a: ", fshow(triangle[0] + off));
        $display("b: ", fshow(triangle[1] + off));
        $display("c: ", fshow(triangle[2] + off));

        let resp <- divider.response.get;
        let t = Triangle{
          vertex: vec(triangle[0]+off, triangle[1]+off, triangle[2]+off),
          center: resp + off,
          instance_id: 0,
          texture_id: 0,
          u: vec(0,0,1),
          v: vec(0,1,0),
          normal: ?
        };

        tree.addTriangle(t);
        triangle_index <= triangle_index + 1;
      endaction
    endseq

    tree.startBuild;
    tree.endBuild;

    action
      state <= 5;
    endaction

    while (True) noAction;
  endseq;

  mkAutoFSM(loadStmt);

  //let sky <- mkSkyColor;
  let hit <- mkIntersectTriangle;


  //let triangle1 = Triangle{
  //  vertex: vec(vec3(0, 0, -1), vec3(0.0, 0.5, -1), vec3(0.5, 0, -1)),
  //  normal: vec(vec3(0,0,-1), vec3(0,0,-1), vec3(0,0,-1)),
  //  u: vec(0, 0, 1),
  //  v: vec(0, 1, 0),
  //  instance_id: 0,
  //  texture_id: 0,
  //  center: 0
  //};

  //triangle1.center = (triangle1.vertex[0] + triangle1.vertex[1] + triangle1.vertex[2]) / 3;

  //let triangle2 = triangle1;
  //triangle2.vertex = vec(vec3(0, 0, -1), vec3(0.0, -0.5, -1), vec3(-0.5, 0, -1));
  //triangle2.center = (triangle2.vertex[0] + triangle2.vertex[1] + triangle2.vertex[2]) / 3;

  //let triangle3 = triangle1;
  //triangle3.vertex = vec(vec3(0, 0, -1), vec3(-0.9, 0.6, -1), vec3(-0.7, 0.7, -1));
  //triangle3.center = (triangle3.vertex[0] + triangle3.vertex[1] + triangle3.vertex[2]) / 3;

  //Fifo#(2, Triangle) triangleQ <- mkFifo;

  //rule enq_triangle;
  //  triangleQ.enq(triangle1);
  //endrule

  //rule set_tri0;
  //  if (state == 0) begin
  //    tree.addTriangle(triangle1);
  //    state <= 1;
  //  end

  //  if (state == 1) begin
  //    tree.addTriangle(triangle2);
  //    state <= 2;
  //  end

  //  if (state == 2) begin
  //    tree.addTriangle(triangle3);
  //    state <= 3;
  //  end

  //  if (state == 3) begin
  //    tree.startBuild;
  //    state <= 4;
  //  end

  //  if (state == 4) begin
  //    tree.endBuild;
  //    state <= 5;
  //  end
  //endrule

  interface Put request;
    method Action put(Ray r) if (state == 5);
      //sky.request.put(r);
      //hit.request.put(tuple2(r, triangleQ.first));
      tree.search.request.put(r);
      //triangleQ.deq;
    endmethod
  endinterface

  interface Get response;
    method ActionValue#(Color) get;
      //let c <- sky.response.get;
      //let h <- hit.response.get;
      let h <- tree.search.response.get;
      return h;

      //if (h.found) begin
      //  let w = 1 - h.u.f - h.v.f;
      //  return rgb(truncateLSB(h.u.f),truncateLSB(h.v.f),truncateLSB(w));
      //end else return 40;
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

  Fifo#(16, Tuple3#(Bit#(32), Bit#(32), Bool)) fifo <- mkFifo;

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

  let divider <- mkVec3Divider;
  Fifo#(4, Vec3) directions <- mkFifo;

  rule send_rayon;
    let inv_dir <- divider.response.get;
    let dir <- toGet(directions).get;

    sky.request.put(Ray{origin: 0, direction: dir, inv_direction: inv_dir});
  endrule

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

    divider.request.put(tuple2(1, pixel_center - camera_center));
    directions.enq(pixel_center - camera_center);

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
