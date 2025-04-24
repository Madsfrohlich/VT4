import ezdxf
from ezdxf.math import bulge_to_arc
from lxml import etree
import math
from collections import defaultdict

###############################################################################
# CONFIG
###############################################################################
origin_x, origin_y, origin_z = 1520, 10, 938
ARC_STEPS, CIRCLE_STEPS, BULGE_STEPS = 20, 20, 10

TRAVEL_POWER  = "0"     # laser OFF
CUT_POWER     = "300"   # laser ON
PULSE_COUNT   = 3       # ON/OFF pairs at each contour start

###############################################################################
# 0) explode INSERTs
###############################################################################
def explode_block_inserts(msp):
    while True:
        inserts = list(msp.query("INSERT"))
        if not inserts:
            break
        for ins in inserts:
            ins.explode()
            msp.delete_entity(ins)

###############################################################################
# 1) geometry helpers (ALL present!)
###############################################################################
def process_line(e):
    x1, y1 = e.dxf.start.x + origin_x, e.dxf.start.y + origin_y
    x2, y2 = e.dxf.end.x   + origin_x, e.dxf.end.y   + origin_y
    return [(x1, y1, origin_z, x2, y2, origin_z)]

def process_arc(e):
    cx, cy = e.dxf.center.x + origin_x, e.dxf.center.y + origin_y
    r      = e.dxf.radius
    a1, a2 = map(math.radians, (e.dxf.start_angle, e.dxf.end_angle))
    if a2 < a1:
        a2 += 2*math.pi
    pts = [(cx + r*math.cos(a1+i*(a2-a1)/ARC_STEPS),
            cy + r*math.sin(a1+i*(a2-a1)/ARC_STEPS),
            origin_z)
           for i in range(ARC_STEPS+1)]
    return [(pts[i][0], pts[i][1], pts[i][2],
             pts[i+1][0], pts[i+1][1], pts[i+1][2])
            for i in range(len(pts)-1)]

def process_circle(e):
    """Return (cx, cy, radius) in robot frame."""
    return (e.dxf.center.x + origin_x,
            e.dxf.center.y + origin_y,
            e.dxf.radius)

def process_lwpolyline(lw):
    segs, pts = [], lw.get_points("xyb")
    n, closed = len(pts), lw.closed
    for i, (x1, y1, b) in enumerate(pts):
        if i == n-1 and not closed:
            break
        x2, y2, _ = pts[(i+1) % n]
        if abs(b) < 1e-12:
            segs.append((x1+origin_x, y1+origin_y, origin_z,
                         x2+origin_x, y2+origin_y, origin_z))
        else:
            arc = bulge_to_arc((x1, y1), (x2, y2), b)
            c, r = arc.center, arc.radius
            s_ang, e_ang = arc.start_angle, arc.end_angle
            if e_ang < s_ang:
                e_ang += 2*math.pi
            for s in range(BULGE_STEPS):
                a1 = s_ang + s*(e_ang-s_ang)/BULGE_STEPS
                a2 = s_ang + (s+1)*(e_ang-s_ang)/BULGE_STEPS
                xa, ya = c[0]+r*math.cos(a1), c[1]+r*math.sin(a1)
                xb, yb = c[0]+r*math.cos(a2), c[1]+r*math.sin(a2)
                segs.append((xa+origin_x, ya+origin_y, origin_z,
                             xb+origin_x, yb+origin_y, origin_z))
    return segs

def process_polyline(pl):
    segs, verts = [], list(pl.vertices())
    n, closed   = len(verts), pl.is_closed
    for i in range(n-1):
        x1,y1,z1 = verts[i].dxf.x, verts[i].dxf.y, verts[i].dxf.z
        x2,y2,z2 = verts[i+1].dxf.x, verts[i+1].dxf.y, verts[i+1].dxf.z
        segs.append((x1+origin_x, y1+origin_y, z1+origin_z,
                     x2+origin_x, y2+origin_y, z2+origin_z))
    if closed and n>1:
        x1,y1,z1 = verts[-1].dxf.x, verts[-1].dxf.y, verts[-1].dxf.z
        x2,y2,z2 = verts[0].dxf.x,  verts[0].dxf.y,  verts[0].dxf.z
        segs.append((x1+origin_x, y1+origin_y, z1+origin_z,
                     x2+origin_x, y2+origin_y, z2+origin_z))
    return segs

###############################################################################
# 2) merge segments → polylines (unchanged from your version)
###############################################################################
def merge_segments_to_polylines(segments, eps=1):
    pts, adj, edges = [], defaultdict(list), set()

    def node(px,py,pz):
        for i,(x,y,z) in enumerate(pts):
            if abs(px-x)<eps and abs(py-y)<eps and abs(pz-z)<eps: return i
        pts.append((px,py,pz)); return len(pts)-1

    for x1,y1,z1,x2,y2,z2 in segments:
        a,b=node(x1,y1,z1),node(x2,y2,z2)
        adj[a].append(b); adj[b].append(a); edges.add(frozenset({a,b}))

    visited, used, polys = set(), set(), []
    def comp(n):
        st=[n];c=[];visited.add(n)
        while st:
            k=st.pop();c.append(k)
            for nb in adj[k]:
                if nb not in visited: visited.add(nb); st.append(nb)
        return c
    def edges_of(ns):
        e=set()
        for n in ns:
            for nb in adj[n]:
                if nb in ns: e.add(frozenset({n,nb}))
        return e
    def chains(ns):
        ce=edges_of(ns)
        while ce-used:
            e0=(ce-used).pop(); a,b=list(e0); used.add(e0); chain=[a,b]
            # fwd
            cur=b
            while True:
                nxt=None
                for nb in adj[cur]:
                    ek=frozenset({cur,nb})
                    if ek in ce and ek not in used:
                        used.add(ek); chain.append(nb); cur=nb; nxt=True; break
                if not nxt: break
            # back
            cur=a
            while True:
                prev=None
                for nb in adj[cur]:
                    ek=frozenset({cur,nb})
                    if ek in ce and ek not in used:
                        used.add(ek); chain.insert(0,nb); cur=nb; prev=True; break
                if not prev: break
            polys.append([pts[i] for i in chain])
    for i in range(len(pts)):
        if i not in visited: chains(comp(i))
    return polys

###############################################################################
# 3) compute area & build shape list
###############################################################################
def poly_area(p):
    a=0
    for i in range(len(p)):
        x1,y1,_=p[i]; x2,y2,_=p[(i+1)%len(p)]
        a+=x1*y2-x2*y1
    return abs(a)/2

def build_shapes(polys, circles):
    shapes=[]
    for p in polys:
        if len(p)>1: shapes.append((poly_area(p),'POLYLINE',p))
    for cx,cy,r in circles:
        shapes.append((math.pi*r*r,'CIRCLE',(cx,cy,r)))
    return shapes

###############################################################################
# BUILD XML – Delay on EACH cutting segment start (50 ms)
###############################################################################
SEG_DELAY_MS = "50"   # pause before every move that actually cuts material

def build_xml(shapes):
    root      = etree.Element("LaserPrograms")
    intertask = etree.SubElement(root, "Intertask", id="515", taskNo="0")

    task_cnt  = 0
    sub_no    = 0
    exec_id   = 0
    prev_x = prev_y = None

    # ------------- helper ---------------------------------------------------
    def add_robot(x, y, z, power, exec_val, vel="0.010", delay="0"):
        nonlocal task_cnt, sub_no
        task = etree.SubElement(
            intertask, "Task",
            Delay=delay,
            ExecutionID=str(exec_val),
            FunctionBlockID="514",
            subNo=str(sub_no)
        )
        etree.SubElement(
            task, "Robot",
            MovType="1",
            PosX=f"{x}", PosY=f"{y}", PosZ=f"{z}",
            PosA="180.00", PosB="0.00", PosC="-180.00",
            Status="22", Turn="27",
            Velocity=vel, Acceleration="0", Jerk="0", Zone="0",
            ToolNum="4", BaseNum="0", MotionNum="0",
            Power=power
        )
        task_cnt += 1
        sub_no   += 1
    # ------------------------------------------------------------------------

    # ─── Air ON ──────────────────────────────────────────────────────────────
    for fn in ("518", "528"):
        tag = "ProtectionAir" if fn == "518" else "CoolingAir"
        t = etree.SubElement(
            intertask, "Task", Delay="0",
            ExecutionID=str(exec_id),
            FunctionBlockID=fn,
            subNo=str(sub_no)
        )
        t.append(etree.Element(tag, State="1"))
        task_cnt += 1; sub_no += 1; exec_id += 1
    # ─────────────────────────────────────────────────────────────────────────

    # ─── Contours inside-first ───────────────────────────────────────────────
    for area, typ, data in shapes:

        # build list of vertices ------------------------------------------------
        if typ == "POLYLINE":
            pts = data
        else:  # CIRCLE
            cx, cy, r = data
            pts = [(cx + r*math.cos(a),
                    cy + r*math.sin(a),
                    origin_z)
                   for a in [i*2*math.pi/CIRCLE_STEPS
                             for i in range(CIRCLE_STEPS+1)]]

        x0, y0, z0 = pts[0]

        # optional travel move --------------------------------------------------
        if (x0, y0) != (prev_x, prev_y):
            add_robot(x0, y0, z0, TRAVEL_POWER, exec_id)  # Delay 0
            exec_id += 1

        # three ON / OFF pulses (each ON & OFF unique ExecID) -------------------
        for _ in range(3):
            add_robot(x0, y0, z0, CUT_POWER,   exec_id, vel="0.000")  # ON
            exec_id += 1
            add_robot(x0, y0, z0, TRAVEL_POWER, exec_id, vel="0.000") # OFF
            exec_id += 1

        # cutting moves – ONE exec_id for whole contour, but Delay on EVERY move
        cut_exec = exec_id; exec_id += 1
        for x, y, z in pts[1:]:
            add_robot(x, y, z, CUT_POWER, cut_exec, delay=SEG_DELAY_MS)

        prev_x, prev_y = pts[-1][0], pts[-1][1]
    # ─────────────────────────────────────────────────────────────────────────

    # ─── Air OFF ─────────────────────────────────────────────────────────────
    for fn in ("518", "528"):
        tag = "ProtectionAir" if fn == "518" else "CoolingAir"
        t = etree.SubElement(
            intertask, "Task", Delay="0",
            ExecutionID=str(exec_id),
            FunctionBlockID=fn,
            subNo=str(sub_no)
        )
        t.append(etree.Element(tag, State="0"))
        task_cnt += 1; sub_no += 1; exec_id += 1
    # ─────────────────────────────────────────────────────────────────────────

    intertask.attrib["taskNo"] = str(task_cnt)
    return root


###############################################################################
# MAIN WRAPPER
###############################################################################
def dxf_to_xml(dxf_file, xml_file):
    doc = ezdxf.readfile(dxf_file); msp = doc.modelspace()
    explode_block_inserts(msp)
    segs,circles = [], []
    for e in msp.query("*"):
        t=e.dxftype()
        if t=="LINE": segs+=process_line(e)
        elif t=="ARC": segs+=process_arc(e)
        elif t=="CIRCLE": circles.append(process_circle(e))
        elif t=="LWPOLYLINE": segs+=process_lwpolyline(e)
        elif t=="POLYLINE": segs+=process_polyline(e)

    polys=merge_segments_to_polylines(segs,eps=1)
    shapes=build_shapes(polys,circles)
    shapes.sort(key=lambda s:s[0])
    etree.ElementTree(build_xml(shapes)).write(
        xml_file,pretty_print=True,xml_declaration=True,encoding="UTF-8")

###############################################################################
if __name__=="__main__":
    dxf_to_xml("Cut_template.dxf","robot_instructions.xml")
