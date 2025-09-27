#!/usr/bin/env python3
import os, random, math, tempfile, subprocess, argparse, pathlib

HERE = pathlib.Path(__file__).resolve().parent
TPL = (HERE / "models" / "box_template.sdf").read_text()

def sample_xy(min_xy, max_xy, keepout_r):
    while True:
        x = random.uniform(min_xy, max_xy)
        y = random.uniform(min_xy, max_xy)
        if math.hypot(x, y) >= keepout_r:
            return x, y

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--num", type=int, default=12, help="장애물 개수")
    ap.add_argument("--min_xy", type=float, default=-8.5, help="맵 안쪽 최소 x/y")
    ap.add_argument("--max_xy", type=float, default= 8.5, help="맵 안쪽 최대 x/y")
    ap.add_argument("--keepout", type=float, default=2.0, help="로봇 주변 비우는 반경(m)")
    ap.add_argument("--min_size", type=float, default=0.3, help="장애물 최소 가로세로(m)")
    ap.add_argument("--max_size", type=float, default=2.0, help="장애물 최대 가로세로(m)")
    args = ap.parse_args()

    for i in range(args.num):
        sx = random.uniform(args.min_size, args.max_size)
        sy = random.uniform(args.min_size, args.max_size)
        sz = random.uniform(0.5, 1.5)  # 높이

        sdf_str = TPL.replace("__SX__", f"{sx:.3f}") \
                     .replace("__SY__", f"{sy:.3f}") \
                     .replace("__SZ__", f"{sz:.3f}")

        with tempfile.NamedTemporaryFile(delete=False, suffix=f"_box_{i}.sdf") as tmp:
            tmp.write(sdf_str.encode("utf-8"))
            tmp_path = tmp.name

        x, y = sample_xy(args.min_xy, args.max_xy, args.keepout)
        yaw = random.uniform(-math.pi, math.pi)
        z = sz / 2.0  # 바닥 위에 놓이게 절반 높이만큼 띄움

        subprocess.run([
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", f"box_{i}",
            "-file", tmp_path,
            "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", f"{z:.3f}",
            "-Y", f"{yaw:.3f}",
        ], check=True)

if __name__ == "__main__":
    main()
