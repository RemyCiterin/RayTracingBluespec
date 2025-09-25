import Geometry :: *;

import FixedPoint :: *;

import Ehr :: *;
import Fifo :: *;
import GetPut :: *;
import ClientServer :: *;

import BRAMCore :: *;

import StmtFSM :: *;

import RegFile :: *;
import DReg :: *;

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

  interface Server#(Ray, RayHit) search;
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

typedef struct {
  NodeIdx current;

} SearchState deriving(Bits, Eq);


(* synthesize *)
module mkTree(Tree);
  Integer size = 3489;

  Reg#(Bit#(32)) cycle <- mkReg(0);
  rule incr_cycle; cycle <= cycle + 1; endrule

  BRAM_PORT#(TriangleIdx, Triangle) triangles <- mkBRAMCore1(size, False);
  Reg#(TriangleIdx) nextTriangle <- mkReg(0);

  BRAM_PORT#(NodeIdx, Node) nodes <- mkBRAMCore1(2 * size - 1, False);
  Reg#(NodeIdx) nextNode <- mkReg(1);

  NodeIdx root = 0;

  CompletionBuffer#(4, RayHit) buffer <- mkCompletionBuffer;
  RegFile#(CBToken#(4), SearchState) states <- mkRegFileFull;

  Reg#(RayHit) searchHit <- mkReg(?);
  Reg#(NodeIdx) searchNode <- mkReg(?);
  Reg#(TriangleIdx) searchTri <- mkReg(?);
  Reg#(Bool) searchFound <- mkReg(?);

  let searchInter <- mkIntersectTriangle;
  Stack#(6, NodeIdx) searchStack <- mkStack;

  Fifo#(2, Ray) searchIn <- mkFifo;
  Fifo#(2, RayHit) searchOut <- mkFifo;
  Reg#(Ray) ray <- mkReg(?);

  let searchStmt = seq
    while (True) seq
      action
        searchIn.deq;
        searchStack.push(root);
        ray <= searchIn.first;
        searchHit.found <= False;
      endaction

      while (!searchStack.empty) seq
        action
          searchNode <= searchStack.read;
          nodes.put(False, searchStack.read, ?);
          searchStack.pop;
        endaction

        action
          let t1 = (nodes.read.aa - ray.origin) * ray.inv_direction;
          let t2 = (nodes.read.bb - ray.origin) * ray.inv_direction;

          let vmin = min3(t1, t2);
          let vmax = max3(t1, t2);

          let tmin = max(vmin.x, max(vmin.y, vmin.z));
          let tmax = min(vmax.x, min(vmax.y, vmax.z));

          searchFound <= tmax >= tmin && tmax > 0 && (!searchHit.found || tmin < searchHit.t);
          triangles.put(False, nodes.read.firstTri, ?);
          searchTri <= nodes.read.firstTri;
        endaction

        if (searchFound) seq
          if (nodes.read.isLeaf) seq
            while (searchTri < nodes.read.firstTri + nodes.read.length) seq
              searchInter.request.put(tuple2(ray, triangles.read));

              action
                let hit <- searchInter.response.get;

                if (hit.found && searchHit.found && hit.t < searchHit.t) begin
                  searchHit <= hit;
                end

                if (hit.found && !searchHit.found) begin
                  searchHit <= hit;
                end

                searchTri <= searchTri + 1;
                triangles.put(False, searchTri + 1, ?);
              endaction
            endseq
          endseq else seq
            searchStack.push(nodes.read.leftChild);
            searchStack.push(nodes.read.leftChild+1);
          endseq
        endseq
      endseq

      action
        searchOut.enq(searchHit);
      endaction
    endseq
  endseq;


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

        nodes.put(False, buildNode, ?);
        action
          $display("leaf: ", nodes.read.isLeaf);
          $display(fshow(nodes.read.aa));
          $display(fshow(nodes.read.bb));
          $display(fshow(buildMin));
          $display(fshow(buildMax));
          $display(buildAxis);
        endaction

        buildNode <= buildNode + 1;
      endseq

      // go to "wait for ack" state
      buildState <= 2;

    endseq

  endseq;

  mkAutoFSM(buildStmt);
  mkAutoFSM(searchStmt);

  interface Server search;
    interface response = toGet(searchOut);
    interface request = toPut(searchIn);
  endinterface

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
