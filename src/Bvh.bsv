import Geometry :: *;

import FixedPoint :: *;

import Ehr :: *;
import Fifo :: *;
import GetPut :: *;
import ClientServer :: *;

import BRAMCore :: *;

import StmtFSM :: *;
import Screen :: *;

import RegFile :: *;
import DReg :: *;

import Vector :: *;

import CompletionBuffer :: *;

typedef Bit#(17) NodeIdx;
typedef Bit#(16) TriangleIdx;

typedef Bit#(2) Axis;

typedef 8 Bins;
typedef Bit#(TLog#(Bins)) BinIdx;

function Point3 min3(Point3 p1, Point3 p2) =
  point3(min(p1.x, p2.x), min(p1.y, p2.y), min(p1.z, p2.z));

function Point3 max3(Point3 p1, Point3 p2) =
  point3(max(p1.x, p2.x), max(p1.y, p2.y), max(p1.z, p2.z));

function Point3 getTriangleMin(Triangle t);
  return min3(t.vertex[0], min3(t.vertex[1], t.vertex[2]));
endfunction

function Point3 getTriangleMax(Triangle t);
  return max3(t.vertex[0], max3(t.vertex[1], t.vertex[2]));
endfunction

function F16 atAxis(Point3 p, Axis a);
  return case (a)
    0: p.x;
    1: p.y;
    2: p.z;
  endcase;
endfunction

typedef struct {
  Point3 aa;
  Point3 bb;

  Bool isLeaf;
  NodeIdx leftChild;

  NodeIdx parent;

  TriangleIdx firstTri;
  TriangleIdx length;
} Node deriving(Bits, FShow, Eq);

function Maybe#(NodeIdx) getLeftChildIdx(Node node) =
  node.isLeaf ? Invalid : Valid(node.leftChild);

function Maybe#(NodeIdx) getRightChildIdx(Node node) =
  node.isLeaf ? Invalid : Valid(node.leftChild + 1);


interface Tree;
  method Action addTriangle(Triangle triangle);

  method Action startBuild;
  method Action endBuild;

  interface Server#(Ray, Color) search;
endinterface

typedef union tagged {
  void Idle;
} State deriving(Bits, Eq);

interface Stack#(numeric type n, type t);
  method t read;
  method Bool empty;
  method Bool full;
  method Action pop;
  method Action push(t value);
endinterface

module mkStack(Stack#(n, t)) provisos(Bits#(t, tW));
  BRAM_DUAL_PORT#(Bit#(n), t) ram <- mkBRAMCore2(2 ** valueOf(n), False);
  Ehr#(2, Bit#(n)) top <- mkEhr(0);
  Reg#(Maybe#(t)) lastPush <- mkDReg(Invalid);

  (* no_implicit_conditions, fire_when_enabled *)
  rule read_rule;
    ram.a.put(False, top[1] - 1, ?);
  endrule

  method read = case (lastPush) matches
    tagged Valid .v : v;
    Invalid : ram.a.read;
  endcase;

  method empty = top[0] == 0;

  method full = top[0] == -1;

  method Action push(t value);
    ram.b.put(True, top[0], value);
    lastPush <= Valid(value);
    top[0] <= top[0] + 1;
  endmethod

  method Action pop;
    top[0] <= top[0] - 1;
  endmethod
endmodule

module mkBRAMSafe#(BRAM_PORT#(k, v) port)(BRAM_PORT#(k, v))
  provisos(Bits#(k, kW), Bits#(v, vW));

  Reg#(v) latch <- mkReg(?);
  Reg#(Bool) didPut <- mkDReg(False);

  (* no_implicit_conditions, fire_when_enabled *)
  rule save_value if (didPut);
    latch <= port.read;
  endrule

  method read = didPut ? port.read : latch;

  method Action put(Bool write, k key, v value);
    port.put(write, key, value);
    didPut <= True;
  endmethod
endmodule

function Maybe#(F16) rayonNodeIntersection(Node node, Ray ray);
  let t1 = (node.aa - ray.origin) * ray.inv_direction;
  let t2 = (node.bb - ray.origin) * ray.inv_direction;

  let vmin = min3(t1, t2);
  let vmax = max3(t1, t2);

  let tmin = max(vmin.x, max(vmin.y, vmin.z));
  let tmax = min(vmax.x, min(vmax.y, vmax.z));

  if (tmax >= tmin && tmax > 0) return Valid(tmin);
  else return Invalid;
endfunction

typedef 2 NumSearchUnit;

module mkBvhSearch#(
    Server#(Tuple2#(Ray, Triangle), RayHit) rayTriInter,
    BRAM_PORT#(TriangleIdx, Triangle) trianglesRaw,
    BRAM_PORT#(NodeIdx, Node) nodesRaw
  ) (Server#(Ray, Color));

  let triangles <- mkBRAMSafe(trianglesRaw);
  let nodes <- mkBRAMSafe(nodesRaw);

  NodeIdx root = 0;
  Reg#(RayHit) currentHit <- mkReg(?);
  Reg#(NodeIdx) node <- mkReg(?);
  Reg#(TriangleIdx) triangle <- mkReg(?);
  Reg#(Bool) found <- mkReg(?);

  Stack#(6, NodeIdx) stack <- mkStack;

  Fifo#(2, Ray) inputs <- mkFifo;
  Fifo#(2, Color) outputs <- mkFifo;
  Reg#(Ray) ray <- mkReg(?);

  Reg#(Color) alpha <- mkReg(?);
  Reg#(Color) beta <- mkReg(?);
  Reg#(Bool) done <- mkReg(?);

  Ehr#(2, Bit#(32)) remaining <- mkEhr(0);

  rule update_hit;
    remaining[0] <= remaining[0] - 1;

    let hit <- rayTriInter.response.get;

    if (hit.found && currentHit.found && hit.t < currentHit.t) begin
      currentHit <= hit;
    end

    if (hit.found && !currentHit.found) begin
      currentHit <= hit;
    end
  endrule

  let stmt = seq
    while (True) seq
      action
        inputs.deq;
        ray <= inputs.first;
        alpha <= rgb(255, 255, 255);
        beta <= rgb(30, 30, 30);
        done <= False;
      endaction

      while (!done) seq
        action
          currentHit.found <= False;
          stack.push(root);
        endaction

        while (!stack.empty) seq
          action
            node <= stack.read;
            nodes.put(False, stack.read, ?);
            stack.pop;
          endaction

          action
            let tmin = rayonNodeIntersection(nodes.read, ray);
            found <= isJust(tmin) && (!currentHit.found || unJust(tmin) < currentHit.t);
            triangles.put(False, nodes.read.firstTri, ?);
            triangle <= nodes.read.firstTri;
          endaction

          if (found) seq
            if (nodes.read.isLeaf) seq
              while (triangle < nodes.read.firstTri + nodes.read.length) seq
                action
                  triangle <= triangle + 1;
                  triangles.put(False, triangle + 1, ?);
                  rayTriInter.request.put(tuple2(ray, triangles.read));
                  remaining[1] <= remaining[1] + 1;
                endaction
              endseq
            endseq else seq
              stack.push(nodes.read.leftChild);
              stack.push(nodes.read.leftChild+1);
            endseq
          endseq
        endseq

        while (remaining[0] != 0) noAction;
        action
          if (!currentHit.found) done <= True;
          else begin
            ray.origin <= at(ray, currentHit.t+0.01);
            alpha <= alpha * rgb(30, 30, 0);
            beta <= beta + alpha * rgb(150, 150, 0);
          end
        endaction
      endseq

      outputs.enq(beta);
    endseq
  endseq;

  mkAutoFSM(stmt);

  interface request = toPut(inputs);
  interface response = toGet(outputs);
endmodule

(* synthesize *)
module mkTree(Tree);
  Integer size = 3500;

  Reg#(Bit#(32)) cycle <- mkReg(0);
  rule incr_cycle; cycle <= cycle + 1; endrule

  BRAM_PORT#(TriangleIdx, Triangle) triangles <- mkBRAMCore1(size, False);
  Reg#(TriangleIdx) nextTriangle <- mkReg(0);

  BRAM_PORT#(NodeIdx, Node) nodes <- mkBRAMCore1(2 * size - 1, False);
  Reg#(NodeIdx) nextNode <- mkReg(1);

  NodeIdx root = 0;

  let intersectServer <- mkIntersectTriangle;
  let searchFSM <- mkBvhSearch(intersectServer, triangles, nodes);

  Reg#(Bit#(2)) buildState <- mkReg(0);
  Reg#(NodeIdx) buildNode <- mkReg(root);
  Reg#(Axis) buildAxis <- mkReg(?);
  Reg#(TriangleIdx) buildTri <- mkReg(?);
  Reg#(TriangleIdx) buildTri1 <- mkReg(?);
  Reg#(TriangleIdx) buildTri2 <- mkReg(?);
  Reg#(Point3) buildMin <- mkReg(?);
  Reg#(Point3) buildMax <- mkReg(?);
  Reg#(F16) buildSplit <- mkReg(?);
  Reg#(Triangle) buildTmp <- mkReg(?);

  let buildStmt = seq
    while (True) seq
      while (buildState != 1) delay(1);

      while (buildNode < nextNode) seq

        nodes.put(False, buildNode, ?);
        action
          buildTri <= nodes.read.firstTri;
          buildMin <= const3(maxBound);
          buildMax <= const3(minBound);
        endaction

        // Compute the bounds of the node
        while (buildTri < nodes.read.firstTri + nodes.read.length) seq
          triangles.put(False, buildTri, ?);
          action
            Node node = nodes.read;
            Triangle t = triangles.read;
            node.aa = min3(node.aa, getTriangleMin(t));
            node.bb = max3(node.bb, getTriangleMax(t));
            buildMin <= min3(buildMin, t.center);
            buildMax <= max3(buildMax, t.center);
            nodes.put(True, buildNode, node);
            buildTri <= buildTri + 1;
          endaction
        endseq

        // Choose the biggest axis and split it in two (very naive approach but it works)
        action
          Vec3 size = buildMax - buildMin;
          let axis =
            size.x > size.y && size.x > size.z ? 0 :
            size.y > size.z ? 1 : 2;
          buildAxis <= axis;

          buildSplit <= case (axis)
            0: buildMin.x + size.x / 2;
            1: buildMin.y + size.y / 2;
            2: buildMin.z + size.z / 2;
          endcase;

          buildTri <= nodes.read.firstTri + nodes.read.length;
          buildTri1 <= nodes.read.firstTri;
          buildTri2 <= nodes.read.firstTri + nodes.read.length - 1;
        endaction

        while (buildTri1 <= buildTri2 && nodes.read.length > 1) seq
          triangles.put(False, buildTri1, ?);
          if (atAxis(triangles.read.center, buildAxis) <= buildSplit) seq
            buildTri1 <= buildTri1 + 1;
          endseq else seq
            action
              buildTmp <= triangles.read;
              triangles.put(False, buildTri2, ?);
            endaction
            triangles.put(True, buildTri1, triangles.read);
            action
              triangles.put(True, buildTri2, buildTmp);
              buildTri2 <= buildTri2 - 1;
            endaction
          endseq
        endseq

        action
          Node node = nodes.read;

          node.isLeaf =
            buildTri1 == node.firstTri ||
            buildTri1 == node.firstTri + node.length;

          if (!node.isLeaf) begin
            node.leftChild = nextNode;
          end

          nodes.put(True, buildNode, node);
        endaction

        if (!nodes.read.isLeaf) seq
          action
            let lhs = Node{
              aa: const3(maxBound),
              bb: const3(minBound),
              parent: buildNode,
              isLeaf: True,
              leftChild: ?,
              firstTri: nodes.read.firstTri,
              length: buildTri1 - nodes.read.firstTri
            };
            nextNode <= nextNode + 1;
            nodes.put(True, nextNode, lhs);
          endaction

          action
            let rhs = Node{
              aa: const3(maxBound),
              bb: const3(minBound),
              parent: buildNode,
              isLeaf: True,
              leftChild: ?,
              firstTri: buildTri1,
              length: buildTri - buildTri1
            };
            nextNode <= nextNode + 1;
            nodes.put(True, nextNode, rhs);
          endaction
        endseq

        buildNode <= buildNode + 1;
      endseq

      // go to "wait for ack" state
      buildState <= 2;

    endseq

  endseq;

  mkAutoFSM(buildStmt);

  interface search = searchFSM;

  method Action addTriangle(Triangle triangle) if (buildState == 0);
    triangles.put(True, nextTriangle, triangle);
    nextTriangle <= nextTriangle + 1;
  endmethod

  method Action startBuild() if (buildState == 0);
    $display("start build BVH at cycle: ", cycle);

    let root_node = Node{
      aa: const3(maxBound),
      bb: const3(minBound),
      parent: root,
      isLeaf: True,
      leftChild: ?,
      firstTri: 0,
      length: nextTriangle
    };

    nodes.put(True, root, root_node);
    buildNode <= root;

    buildState <= 1;
  endmethod

  method Action endBuild if (buildState == 2);
    $display("finish build BVH at cycle: ", cycle);
    noAction;
  endmethod
endmodule
