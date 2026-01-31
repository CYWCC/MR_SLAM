#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
from collections import defaultdict, Counter
import math

MASK56 = (1 << 56) - 1

def normalize_quat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0:
        return qx, qy, qz, qw
    return qx/n, qy/n, qz/n, qw/n

def decode_symbol_index(vid: int):
    sym = chr((vid >> 56) & 0xFF)
    idx = vid & MASK56
    return sym, idx

def parse_vertices(g2o_path):
    # returns list: (vid, sym, idx, x,y,z,qx,qy,qz,qw)
    out = []
    with open(g2o_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("VERTEX_SE3:QUAT"):
                continue
            parts = line.strip().split()
            if len(parts) < 9:
                continue
            vid = int(parts[1])
            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
            qx, qy, qz, qw = float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8])
            qx, qy, qz, qw = normalize_quat(qx, qy, qz, qw)
            sym, idx = decode_symbol_index(vid)
            out.append((vid, sym, idx, x, y, z, qx, qy, qz, qw))
    return out

def quat_to_rot(qx, qy, qz, qw):
    """
    Quaternion -> Rotation matrix (3x3)
    Using (x, y, z, w) convention.
    """
    # precompute
    xx = qx*qx; yy = qy*qy; zz = qz*qz
    xy = qx*qy; xz = qx*qz; yz = qy*qz
    wx = qw*qx; wy = qw*qy; wz = qw*qz

    r00 = 1.0 - 2.0*(yy + zz)
    r01 = 2.0*(xy - wz)
    r02 = 2.0*(xz + wy)

    r10 = 2.0*(xy + wz)
    r11 = 1.0 - 2.0*(xx + zz)
    r12 = 2.0*(yz - wx)

    r20 = 2.0*(xz - wy)
    r21 = 2.0*(yz + wx)
    r22 = 1.0 - 2.0*(xx + yy)

    return (r00, r01, r02,
            r10, r11, r12,
            r20, r21, r22)

def sort_rows(rows, timestamp_mode="index"):
    # rows: (vid, sym, idx, x,y,z,qx,qy,qz,qw)
    if timestamp_mode == "index":
        return sorted(rows, key=lambda r: r[2])
    else:
        return sorted(rows, key=lambda r: r[0])

def write_tum(rows, out_path, timestamp_mode="index"):
    """
    TUM: t x y z qx qy qz qw
    timestamp_mode: index | seq | id
    """
    rows = sort_rows(rows, timestamp_mode=timestamp_mode)

    with open(out_path, "w", encoding="utf-8") as f:
        for i, (vid, sym, idx, x, y, z, qx, qy, qz, qw) in enumerate(rows):
            if timestamp_mode == "seq":
                t = float(i)
            elif timestamp_mode == "id":
                t = float(vid)
            else:  # index
                t = float(idx)
            f.write(f"{t:.6f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")

def write_kitti(rows, out_path, order_mode="index"):
    """
    KITTI: each line is 12 floats of a 3x4 pose matrix [R|t]
    r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz

    order_mode: index | id
      - index: sort by decoded index (recommended)
      - id: sort by full vertex id
    """
    rows = sort_rows(rows, timestamp_mode=order_mode)

    with open(out_path, "w", encoding="utf-8") as f:
        for (vid, sym, idx, x, y, z, qx, qy, qz, qw) in rows:
            r = quat_to_rot(qx, qy, qz, qw)
            # write in kitti row-major 3x4
            f.write(
                f"{r[0]:.9f} {r[1]:.9f} {r[2]:.9f} {x:.9f} "
                f"{r[3]:.9f} {r[4]:.9f} {r[5]:.9f} {y:.9f} "
                f"{r[6]:.9f} {r[7]:.9f} {r[8]:.9f} {z:.9f}\n"
            )

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--g2o", required=True, help="fullGraph.g2o path")
    ap.add_argument("--outdir", default="out", help="output directory")
    ap.add_argument("--symbols", default="", help="comma symbols to export, e.g. a,b,c (empty=auto top3)")

    # output format
    ap.add_argument("--format", choices=["tum", "kitti", "both"], default="tum",
                    help="output format: tum / kitti / both")

    # TUM timestamp behavior
    ap.add_argument("--timestamp", choices=["index", "seq", "id"], default="index",
                    help="TUM timestamp source (only for --format tum/both)")

    # KITTI ordering behavior (no timestamp, but ordering matters)
    ap.add_argument("--kitti_order", choices=["index", "id"], default="index",
                    help="Ordering for KITTI output")

    args = ap.parse_args()

    rows = parse_vertices(args.g2o)
    if not rows:
        raise SystemExit("No VERTEX_SE3:QUAT found.")

    cnt = Counter(r[1] for r in rows)
    print("Symbol counts:", dict(cnt))

    if args.symbols.strip():
        syms = [s.strip() for s in args.symbols.split(",") if s.strip()]
    else:
        syms = [k for k, _ in cnt.most_common(3)]

    groups = defaultdict(list)
    for r in rows:
        if r[1] in syms:
            groups[r[1]].append(r)

    os.makedirs(args.outdir, exist_ok=True)

    for i, sym in enumerate(syms, start=1):
        if args.format in ("tum", "both"):
            out_path = os.path.join(args.outdir, f"robot{i}.txt")
            write_tum(groups[sym], out_path, timestamp_mode=args.timestamp)
            print(f"[OK] TUM  robot{i} <- symbol '{sym}': {len(groups[sym])} poses -> {out_path}")

        if args.format in ("kitti", "both"):
            out_path = os.path.join(args.outdir, f"robot{i}.txt")
            write_kitti(groups[sym], out_path, order_mode=args.kitti_order)
            print(f"[OK] KITTI robot{i} <- symbol '{sym}': {len(groups[sym])} poses -> {out_path}")

if __name__ == "__main__":
    main()
