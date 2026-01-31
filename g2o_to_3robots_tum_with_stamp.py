#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import glob
import math
from collections import defaultdict, Counter

MASK56 = (1 << 56) - 1

def normalize_quat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0:
        return qx, qy, qz, qw
    return qx/n, qy/n, qz/n, qw/n

def decode_symbol_index(g2oid: int):
    sym = chr((g2oid >> 56) & 0xFF)
    idx = g2oid & MASK56
    return sym, idx

def parse_g2o_vertices(g2o_path):
    """
    Read VERTEX_SE3:QUAT lines:
      VERTEX_SE3:QUAT <id> <x> <y> <z> <qx> <qy> <qz> <qw>
    Return list of dict:
      {id, sym, idx, x,y,z,qx,qy,qz,qw}
    """
    out = []
    with open(g2o_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("VERTEX_SE3:QUAT"):
                continue
            parts = line.strip().split()
            if len(parts) < 9:
                continue
            g2oid = int(parts[1])
            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
            qx, qy, qz, qw = float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8])
            qx, qy, qz, qw = normalize_quat(qx, qy, qz, qw)
            sym, idx = decode_symbol_index(g2oid)
            out.append({
                "id": g2oid, "sym": sym, "idx": idx,
                "x": x, "y": y, "z": z,
                "qx": qx, "qy": qy, "qz": qz, "qw": qw
            })
    return out

def load_keyframe_stamp_map(keyframes_dir):
    """
    keyframes_dir/
      <g2oid>/data   (first line: "stamp <ns>")
    Return dict: {g2oid: stamp_ns}
    """
    stamp_map = {}
    # 只遍历一级子目录
    for d in glob.glob(os.path.join(keyframes_dir, "*")):
        if not os.path.isdir(d):
            continue
        base = os.path.basename(d)
        if not base.isdigit():
            continue
        g2oid = int(base)
        data_path = os.path.join(d, "data")
        if not os.path.isfile(data_path):
            continue

        stamp_ns = None
        with open(data_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                if line.startswith("stamp "):
                    try:
                        stamp_ns = int(line.strip().split()[1])
                    except Exception:
                        stamp_ns = None
                    break
        if stamp_ns is not None:
            stamp_map[g2oid] = stamp_ns
    return stamp_map

def write_tum(rows, out_path):
    """
    rows: list of tuples (t_sec, x,y,z,qx,qy,qz,qw)
    """
    rows.sort(key=lambda r: r[0])  # sort by time
    with open(out_path, "w", encoding="utf-8") as f:
        for t, x, y, z, qx, qy, qz, qw in rows:
            f.write(f"{t:.6f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--g2o", required=True, help="Path to full_graph.g2o")
    ap.add_argument("--keyframes_dir", required=True, help="Path to Keyframes directory (contains <g2oid>/data)")
    ap.add_argument("--outdir", required=True, help="Output directory")
    ap.add_argument("--symbols", default="a,b,c", help="Robot symbols, e.g. a,b,c (order maps to robot1..3)")
    ap.add_argument("--on_missing_stamp", choices=["skip", "index", "seq"], default="skip",
                    help="What to do if a g2o vertex has no stamp: skip / use index / use seq")

    args = ap.parse_args()

    syms = [s.strip() for s in args.symbols.split(",") if s.strip()]
    if len(syms) != 3:
        raise SystemExit("Please provide exactly 3 symbols, e.g. --symbols a,b,c")

    if not os.path.isfile(args.g2o):
        raise SystemExit(f"g2o not found: {args.g2o}")
    if not os.path.isdir(args.keyframes_dir):
        raise SystemExit(f"keyframes_dir not found: {args.keyframes_dir}")

    os.makedirs(args.outdir, exist_ok=True)

    vertices = parse_g2o_vertices(args.g2o)
    if not vertices:
        raise SystemExit("No VERTEX_SE3:QUAT found in g2o.")

    stamp_map = load_keyframe_stamp_map(args.keyframes_dir)

    # 统计一下 symbol 分布，方便你检查是不是只有 a/b/c
    cnt = Counter(v["sym"] for v in vertices)
    print("G2O vertex symbol counts:", dict(cnt))
    print("Keyframe stamp folders:", len(stamp_map))

    groups = {syms[0]: [], syms[1]: [], syms[2]: []}
    missing = {syms[0]: 0, syms[1]: 0, syms[2]: 0}
    used = {syms[0]: 0, syms[1]: 0, syms[2]: 0}

    # 先按 (sym, idx) 排序，便于 seq/index fallback 时顺序稳定
    vertices_sorted = sorted(vertices, key=lambda v: (v["sym"], v["idx"]))

    seq_counter = {syms[0]: 0, syms[1]: 0, syms[2]: 0}

    for v in vertices_sorted:
        sym = v["sym"]
        if sym not in groups:
            continue

        g2oid = v["id"]
        stamp_ns = stamp_map.get(g2oid)

        if stamp_ns is None:
            missing[sym] += 1
            if args.on_missing_stamp == "skip":
                continue
            elif args.on_missing_stamp == "index":
                t = float(v["idx"])  # 伪时间戳：index
            else:  # seq
                t = float(seq_counter[sym])
                seq_counter[sym] += 1
        else:
            # 真实时间戳：ns -> sec
            t = stamp_ns / 1e9

        groups[sym].append((t, v["x"], v["y"], v["z"], v["qx"], v["qy"], v["qz"], v["qw"]))
        used[sym] += 1

    # 输出：robot1/2/3 对应 syms 的顺序
    for i, sym in enumerate(syms, start=1):
        out_path = os.path.join(args.outdir, f"robot{i}.txt")
        write_tum(groups[sym], out_path)
        print(f"[OK] robot{i} <- symbol '{sym}': wrote {used[sym]} poses. missing_stamp={missing[sym]} -> {out_path}")

if __name__ == "__main__":
    main()
