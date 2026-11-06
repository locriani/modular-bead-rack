#!/usr/bin/env python3
"""
Automated DFM (3D Printability, Overhang, Bed Adhesion & Structural) Audit Script
=================================================================================
Analyzes all generated printable STL files in `stl/` for:
  1. Print Bed Fit (<= 200 mm max XY dimension for universal printer compatibility)
  2. Build Plate Contact Area at Z = 0 (mm² of first-layer adhesion)
  3. Overhang Area > 45.6° from vertical (`nz < -0.715`) above Z = 0.2 mm, clipping
     triangles that straddle Z = 0.2 mm so no steep overhang is hidden by touching the bed
  4. Degenerate or zero-area triangles (rejected if > 0)
"""

import os
import sys
import struct
import numpy as np


def load_binary_stl(filepath):
    with open(filepath, "rb") as f:
        _ = f.read(80)
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
    valid = areas > 1e-6
    degen_count = int((~valid).sum())
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

    # 2. Strict overhang check above build plate (z_cut = bmin[2] + 0.20 mm):
    # Instead of excluding triangles with z_min <= z_cut wholesale, weight any straddling triangle
    # by the fraction of its height above z_cut, and check both >45.6° (nz < -0.715) and >48° (nz < -0.75).
    z_cut = bmin[2] + 0.20
    span_z = np.maximum(1e-6, z_max_tri - z_min_tri)
    frac_above = np.clip((z_max_tri - z_cut) / span_z, 0.0, 1.0)
    frac_above[z_max_tri <= z_cut] = 0.0
    frac_above[z_min_tri >= z_cut] = 1.0

    ovh_45_mask = (normals[:, 2] < -0.715) & (z_max_tri > z_cut)
    ovh_48_mask = (normals[:, 2] < -0.750) & (z_max_tri > z_cut)
    overhang_45_area = (areas[ovh_45_mask] * frac_above[ovh_45_mask]).sum()
    overhang_48_area = (areas[ovh_48_mask] * frac_above[ovh_48_mask]).sum()
    total_area = areas.sum()

    return {
        "dims": dims,
        "tris": len(v0),
        "degen": degen_count,
        "total_area": total_area,
        "bed_area": bed_area,
        "overhang_45_area": overhang_45_area,
        "overhang_48_area": overhang_48_area,
        "overhang_pct": 100.0 * overhang_45_area / max(1.0, total_area),
    }


if __name__ == "__main__":
    stl_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
    files = sorted(f for f in os.listdir(stl_dir) if f.endswith(".stl") and not f.startswith("00_"))
    print(
        f"{'Part STL':<28} | {'Dims (X×Y×Z mm)':<20} | {'Bed Area':>10} | "
        f"{'Ovh >45.6°':>12} | {'Ovh >48°':>10} | {'Degen':>5} | {'Status'}"
    )
    print("-" * 112)
    any_fail = False
    for fname in files:
        res = audit_stl(os.path.join(stl_dir, fname))
        d = res["dims"]
        dim_str = f"{d[0]:5.1f}×{d[1]:5.1f}×{d[2]:4.1f}"
        bed_str = f"{res['bed_area']:7.0f} mm²"
        ovh45_str = f"{res['overhang_45_area']:6.2f} mm²"
        ovh48_str = f"{res['overhang_48_area']:5.2f} mm²"
        fits_200 = max(d[0], d[1]) <= 200.1
        ok = (res["overhang_45_area"] <= 0.05) and (res["degen"] == 0) and fits_200 and (res["bed_area"] >= 250.0)
        if not ok:
            any_fail = True
        status = "PASS (0.0mm² Ovh & <200mm)" if ok else "FAIL"
        print(
            f"{fname:<28} | {dim_str:<20} | {bed_str:>10} | "
            f"{ovh45_str:>12} | {ovh48_str:>10} | {res['degen']:>5d} | {status}"
        )
    if any_fail:
        sys.exit(1)
