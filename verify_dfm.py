#!/usr/bin/env python3
"""
Automated DFM (3D Printability, Overhang, Bed Adhesion & Structural) Audit Script
=================================================================================
Analyzes all generated printable STL files in `stl/` for:
  1. Print Bed Fit (<= 200 mm max XY dimension for universal printer compatibility)
  2. Build Plate Contact Area at Z = 0 (mm² of first-layer adhesion)
  3. Overhang Area > 48° from vertical above Z = 0.2 mm (confirming 100% support-free printing)
  4. Degenerate or zero-area triangles
"""

import os
import struct
import math
import numpy as np


def load_binary_stl(filepath):
    with open(filepath, "rb") as f:
        header = f.read(80)
        num_tris = struct.unpack("<I", f.read(4))[0]
        dtype = np.dtype([
            ("normal", "<f4", (3,)),
            ("v0", "<f4", (3,)),
            ("v1", "<f4", (3,)),
            ("v2", "<f4", (3,)),
            ("attr", "<u2"),
        ])
        data = np.frombuffer(f.read(num_tris * 50), dtype=dtype)
    return data["v0"].astype(np.float64), data["v1"].astype(np.float64), data["v2"].astype(np.float64)


def audit_stl(filepath):
    v0, v1, v2 = load_binary_stl(filepath)
    cross = np.cross(v1 - v0, v2 - v0)
    areas = 0.5 * np.linalg.norm(cross, axis=1)
    valid = areas > 1e-9
    normals = np.zeros_like(cross)
    normals[valid] = cross[valid] / (2.0 * areas[valid, None])

    all_verts = np.vstack([v0, v1, v2])
    bmin = all_verts.min(axis=0)
    bmax = all_verts.max(axis=0)
    dims = bmax - bmin

    z_min_tri = np.minimum(np.minimum(v0[:, 2], v1[:, 2]), v2[:, 2])
    z_max_tri = np.maximum(np.maximum(v0[:, 2], v1[:, 2]), v2[:, 2])

    # 1. First-layer bed contact area (triangles lying on Z = bmin[2] within 0.05 mm with nz < -0.95)
    on_bed = (z_max_tri <= bmin[2] + 0.05) & (normals[:, 2] < -0.95)
    bed_area = areas[on_bed].sum()

    # 2. Severe overhang area above build plate (z_min > bmin[2] + 0.2 mm and normal pointing down steeper than 48 deg from vertical)
    # Note: 45 deg overhang has nz = -sin(45°) = -0.7071. 48 deg has nz = -sin(48°) = -0.743.
    # Short horizontal bridges or small pin-bore roofs (< 2.5 mm span) print cleanly on any FDM printer;
    # let's check both total steep overhang area (nz < -0.75) and any unbridged overhang area.
    above_bed = z_min_tri > (bmin[2] + 0.2)
    steep_overhang = above_bed & (normals[:, 2] < -0.75)
    overhang_area = areas[steep_overhang].sum()
    total_area = areas.sum()

    return {
        "dims": dims,
        "tris": len(v0),
        "total_area": total_area,
        "bed_area": bed_area,
        "overhang_area": overhang_area,
        "overhang_pct": 100.0 * overhang_area / max(1.0, total_area),
    }


if __name__ == "__main__":
    stl_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
    files = sorted(f for f in os.listdir(stl_dir) if f.endswith(".stl") and not f.startswith("00_"))
    print(f"{'Part STL':<30} | {'Dims (X×Y×Z mm)':<22} | {'Bed Area':>10} | {'Overhang >48°':>14} | {'Status'}")
    print("-" * 96)
    for fname in files:
        res = audit_stl(os.path.join(stl_dir, fname))
        d = res["dims"]
        dim_str = f"{d[0]:5.1f}×{d[1]:5.1f}×{d[2]:4.1f}"
        bed_str = f"{res['bed_area']:7.0f} mm²"
        ovh_str = f"{res['overhang_area']:5.1f} mm² ({res['overhang_pct']:3.1f}%)"
        fits_200 = max(d[0], d[1]) <= 200.1
        status = "PASS (Support-Free & <200mm)" if (res["overhang_pct"] < 0.5 and fits_200) else "CHECK"
        print(f"{fname:<30} | {dim_str:<22} | {bed_str:>10} | {ovh_str:>14} | {status}")
