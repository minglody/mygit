#!/usr/bin/env python3
import math, csv, pathlib, argparse, textwrap

# ===== 기본 파라미터 =====
WALL_HEIGHT = 1.0          # 벽 높이
WALL_THICK  = 0.1          # 통로 벽 두께(간선용)
LANE_WIDTH  = 0.5          # 통로 폭 (두 벽 사이)
NODE_RADIUS = 1.0          # 노드(원기둥) 반지름 (겹침 줄이려 1.0 권장)
GAP_AT_NODE = 0.05         # 간선 벽이 노드에 살짝 닿도록 여유
# --- 노드 껍데기(둘레 박스 세분화) 설정 ---
SHELL_THICK = 0.10         # 노드 옆면 '껍데기' 두께 (바깥쪽으로 세움)
SHELL_SEGS  = 48           # 둘레를 몇 분할로 박스 배치할지 (크면 둥글게 보임)
OPEN_MARGIN = 0.10         # 개구부 여유 (통로폭 + 여유)
SHELL_ALPHA = 0.05         # 노드 시각화 투명도 (0=완전 투명, 1=불투명)

PKG   = pathlib.Path(__file__).resolve().parents[1]
NODES = PKG / "nodes.csv"
EDGES = PKG / "edges.csv"
WORLD = PKG / "worlds" / "graph_routes.world"

# ===== CSV =====
def ensure_templates():
    if not NODES.exists():
        with open(NODES, "w", newline="") as f:
            w=csv.writer(f); w.writerow(["name","x","y"])
            w.writerow(["A", 0.0, -8.0])
            w.writerow(["B", 5.0, -4.0])
            w.writerow(["C",-5.0,  0.0])
            w.writerow(["D", 5.0,  1.0])
            w.writerow(["G", 0.0,  8.0])
    if not EDGES.exists():
        with open(EDGES, "w", newline="") as f:
            w=csv.writer(f); w.writerow(["from","to","seconds"])
            for a,b,t in [("A","C",7),("C","G",7),("A","G",10),
                          ("A","B",8),("B","D",4),("D","G",6),("B","G",9)]:
                w.writerow([a,b,t])

def read_nodes():
    d={}
    with open(NODES) as f:
        for r in csv.DictReader(f):
            d[r["name"].strip()] = (float(r["x"]), float(r["y"]))
    return d

def read_edges():
    e=[]
    with open(EDGES) as f:
        for r in csv.DictReader(f):
            e.append([r["from"].strip(), r["to"].strip(), float(r["seconds"])])
    return e

# ===== SDF helpers =====
def wall_box(name, cx, cy, yaw, length, thick, height, mat="Gazebo/DarkGrey", alpha=1.0):
    # length: x-축 방향, thick: y-축 방향
    mat_block = f"""
          <material>
            <ambient>0 0 1 {alpha}</ambient>
            <diffuse>0 0 1 {alpha}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
    """ if alpha < 1.0 else f"""
          <material>
            <script><uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{mat}</name></script>
          </material>
    """
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{cx:.6f} {cy:.6f} {height/2:.3f} 0 0 {yaw:.12f}</pose>
      <link name='link'>
        <visual name='vis'>
          <geometry><box><size>{max(length,0.001)} {max(thick,0.001)} {height}</size></box></geometry>
          {mat_block}
        </visual>
        <collision name='col'>
          <geometry><box><size>{max(length,0.001)} {max(thick,0.001)} {height}</size></box></geometry>
        </collision>
      </link>
    </model>
    """

def unit_normal(dx,dy):
    L=math.hypot(dx,dy); return (-dy/L, dx/L)

# ----- 간선(통로) : 노드 겉면↔겉면까지, 시간 무관 -----
def edge_sdf(idx, p, q, node_radius, lane_width):
    x1,y1=p; x2,y2=q
    dx,dy = x2-x1, y2-y1
    yaw   = math.atan2(dy,dx)
    nx,ny = unit_normal(dx,dy); off=lane_width/2.0

    center_d = math.hypot(dx,dy)
    length   = max(0.0, center_d - 2.0*node_radius - GAP_AT_NODE)

    midx,midy=(x1+x2)/2.0, (y1+y2)/2.0
    cxL,cyL  = midx + nx*off, midy + ny*off
    cxR,cyR  = midx - nx*off, midy - ny*off

    s  = wall_box(f"edge{idx}_L", cxL,cyL,yaw,length, WALL_THICK, WALL_HEIGHT, "Gazebo/DarkGrey", 1.0)
    s += wall_box(f"edge{idx}_R", cxR,cyR,yaw,length, WALL_THICK, WALL_HEIGHT, "Gazebo/DarkGrey", 1.0)
    return s

# ----- 노드 껍데기 : 둘레를 박스로 세분화, 간선 방향 각도 주변만 '개구부' -----
def node_shell_sdf(name, x, y, node_radius, shell_thick, height,
                   incident_angles, segs=SHELL_SEGS, open_width=LANE_WIDTH, open_margin=OPEN_MARGIN, alpha=SHELL_ALPHA):
    # 개구부 반각 (rad) : 통로폭 + 여유가 반지름에서 차지하는 각
    half_width = (open_width/2.0) + open_margin
    # 보수적으로 반각 = asin(half_width / node_radius). 반지름이 매우 작으면 최소 각도 보장.
    half_ang = math.asin(max(0.0, min(1.0, half_width / max(node_radius, 1e-6)))) if node_radius>0 else 0.0
    # 각도 표준화 함수
    def wrap(a):
        while a<=-math.pi: a+=2*math.pi
        while a> math.pi: a-=2*math.pi
        return a
    inc = [wrap(a) for a in incident_angles]

    # 세분화된 박스(호 길이) 생성
    out=[]
    dtheta = 2*math.pi/segs
    for i in range(segs):
        th = -math.pi + (i+0.5)*dtheta  # 세그먼트 중앙각
        # 개구부 범위에 들어가면 스킵
        skip=False
        for ang in inc:
            if abs(wrap(th - ang)) <= max(half_ang, dtheta*0.6):  # 최소 폭 확보
                skip=True; break
        if skip: continue
        # 박스 길이 = 호 길이, 두께 = shell_thick (바깥쪽 법선 방향으로 얇게)
        arc_len = node_radius * dtheta
        # 박스 중심 위치: 반지름 r에서 각 th 방향
        cx = x + node_radius * math.cos(th)
        cy = y + node_radius * math.sin(th)
        # 박스는 접선 방향(=th+pi/2)으로 길이를 둔다
        yaw = th + math.pi/2.0
        out.append(
            wall_box(f"node_{name}_seg{i}", cx, cy, yaw, arc_len, shell_thick, height, mat="Gazebo/Blue", alpha=alpha)
        )
    return "".join(out)

def build_world(nodes, edges, node_radius, lane_width, shell_on=True,
                shell_thick=SHELL_THICK, shell_segs=SHELL_SEGS, open_margin=OPEN_MARGIN, shell_alpha=SHELL_ALPHA):
    header = """
<sdf version="1.6">
  <world name="graph_routes">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
"""
    body=[]

    # 간선 각도 사전 : 노드 -> [각도들]
    angs = {n:[] for n in nodes}
    for a,b,_ in edges:
        (x1,y1)=nodes[a]; (x2,y2)=nodes[b]
        angs[a].append(math.atan2(y2-y1, x2-x1))
        angs[b].append(math.atan2(y1-y2, x1-x2))  # 반대 방향

    # 노드 시각화(기존 실린더는 제거) -> '껍데기'로 대체
    for n,(x,y) in nodes.items():
        if shell_on:
            body.append(
                node_shell_sdf(n, x, y, node_radius, shell_thick, WALL_HEIGHT,
                               angs[n], segs=shell_segs, open_width=lane_width,
                               open_margin=open_margin, alpha=shell_alpha)
            )

    # 간선(통로) 생성
    for i,(a,b,t) in enumerate(edges):
        body.append(edge_sdf(i, nodes[a], nodes[b], node_radius, lane_width))

    footer = """
  </world>
</sdf>
"""
    return header + "\n".join(body) + footer

# ===== CLI =====
def cmd_gen(args):
    ensure_templates()
    nodes = read_nodes()
    edges = read_edges()
    w = build_world(nodes, edges,
                    node_radius=args.radius,
                    lane_width=args.lane,
                    shell_on=(args.shell==1),
                    shell_thick=args.shell_thick,
                    shell_segs=args.shell_segs,
                    open_margin=args.open_margin,
                    shell_alpha=args.alpha)
    WORLD.write_text(textwrap.dedent(w))
    print(f"[OK] world written: {WORLD}")

if __name__=="__main__":
    ap=argparse.ArgumentParser(description="Graph world with node 'shell' and openings at corridor directions")
    sub=ap.add_subparsers(dest="cmd", required=True)

    p=sub.add_parser("gen", help="월드 생성")
    p.add_argument("--lane", type=float, default=LANE_WIDTH, help="통로 폭 (두 벽 사이)")
    p.add_argument("--radius", type=float, default=NODE_RADIUS, help="노드 반지름")
    p.add_argument("--shell", type=int, default=1, help="노드 껍데기 on/off (1/0)")
    p.add_argument("--shell-segs", type=int, default=SHELL_SEGS, help="노드 둘레 분할 개수")
    p.add_argument("--shell-thick", type=float, default=SHELL_THICK, help="노드 껍데기 두께")
    p.add_argument("--open-margin", type=float, default=OPEN_MARGIN, help="개구부 여유 (m)")
    p.add_argument("--alpha", type=float, default=SHELL_ALPHA, help="노드 껍데기 투명도(0~1)")
    p.set_defaults(func=cmd_gen)

    args=ap.parse_args(); args.func(args)
