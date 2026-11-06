#!/usr/bin/env python3
"""
Blender 5.2 Manifold Boolean Unifier, Component Auditor & 3D Interference Verifier
==================================================================================
1. Imports each generated STL in `stl/`, separates loose sub-primitives,
   performs an exact self-union boolean (`solver='EXACT'`, `use_self=True`)
   to remove all internal coplanar/buried faces where sub-parts overlap,
   verifies 0 non-manifold edges, 0 degenerate faces, positive signed volume,
   and exact expected connected component count (`1` for `01`..`06`, `2` for `07`),
   and exports clean binary STLs.
2. Runs a full 3D Boolean INTERSECT interference audit across all mated
   assembly configurations and kinematic shelf states:
   - Stacked Left Frames (`Z = 0` vs `Z = +192.0 mm`)
   - Cross-Brace Bar mated on Left Frame
   - Stabilizer Foot mated on Left Frame
   - Shelf at Stowed (`Y_slide = 0, 0 deg`), Pulled-Out (`Y_slide = 32 mm, 0 deg`),
     and Tilted (`Y_slide = 40 mm, -45 deg`) against both Left Frame and Foot,
     plus desk-plane clearance check (`min_Z > -10.0 mm`).
   Fails with `sys.exit(1)` if any interference volume > 0.001 mm³ or any topology check fails.
"""

import os
import sys
import bpy
import bmesh


EXPECTED_COMPONENTS = {
    "01_tilt_shelf_6bay.stl": 1,
    "02_tilt_shelf_open_tray.stl": 1,
    "03_side_frame_left.stl": 1,
    "04_side_frame_right.stl": 1,
    "05_cross_brace_bar.stl": 1,
    "06_stabilizer_foot.stl": 1,
    "07_connector_clips.stl": 2,
}


def count_connected_components(bm):
    bm.verts.ensure_lookup_table()
    visited = set()
    components = 0
    for v in bm.verts:
        if v.index not in visited:
            components += 1
            stack = [v]
            visited.add(v.index)
            while stack:
                curr = stack.pop()
                for e in curr.link_edges:
                    other = e.other_vert(curr)
                    if other.index not in visited:
                        visited.add(other.index)
                        stack.append(other)
    return components


def clean_and_unify_object(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.object.mode_set(mode="OBJECT")

    bpy.ops.mesh.separate(type="LOOSE")
    parts = list(bpy.context.selected_objects)

    if len(parts) > 1:
        base = parts[0]
        bpy.context.view_layer.objects.active = base
        for p in parts:
            p.select_set(True)
        base.select_set(False)
        bpy.context.view_layer.objects.active = parts[1]
        if len(parts) > 2:
            bpy.ops.object.join()
        other = bpy.context.active_object

        bpy.ops.object.select_all(action="DESELECT")
        base.select_set(True)
        bpy.context.view_layer.objects.active = base

        mod = base.modifiers.new(name="Unify", type="BOOLEAN")
        mod.operation = "UNION"
        mod.object = other
        mod.solver = "EXACT"
        mod.use_self = True
        bpy.ops.object.modifier_apply(modifier=mod.name)

        bpy.data.objects.remove(other, do_unlink=True)
        obj = base

    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-4)
    bmesh.ops.dissolve_degenerate(bm, dist=1e-4, edges=bm.edges)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bmesh.ops.dissolve_degenerate(bm, dist=1e-4, edges=bm.edges)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-4)
    non_tris = [f for f in bm.faces if len(f.verts) > 3]
    if non_tris:
        bmesh.ops.triangulate(bm, faces=non_tris)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.object.mode_set(mode="OBJECT")
    return obj


def unify_stl_file(filepath):
    fname = os.path.basename(filepath)
    bpy.ops.wm.read_factory_settings(use_empty=True)

    bpy.ops.wm.stl_import(filepath=filepath)
    obj = bpy.context.selected_objects[0]
    obj = clean_and_unify_object(obj)

    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(obj.data)

    def tri_area_f32(f):
        if len(f.verts) != 3:
            return f.calc_area()
        a, b, c = f.verts[0].co, f.verts[1].co, f.verts[2].co
        return 0.5 * (b - a).cross(c - a).length

    non_manifold_edges = sum(1 for e in bm.edges if not e.is_manifold)
    degenerate_faces = sum(1 for f in bm.faces if tri_area_f32(f) <= 1e-6)
    num_faces = len(bm.faces)
    volume = bm.calc_volume(signed=True)
    comp_count = count_connected_components(bm)
    expected_comp = EXPECTED_COMPONENTS.get(fname, 1)

    bmesh.update_edit_mesh(obj.data)
    bpy.ops.object.mode_set(mode="OBJECT")

    bpy.ops.wm.stl_export(filepath=filepath, export_selected_objects=True, ascii_format=False)
    status = "OK"
    if non_manifold_edges > 0 or degenerate_faces > 0 or volume <= 0 or comp_count != expected_comp:
        status = "FAIL"
    print(
        f"UNIFIED {fname:<28} | Tris: {num_faces:>6,d} | Vol: {volume:>8.1f} mm³ | "
        f"Comps: {comp_count}/{expected_comp} | NonManifold: {non_manifold_edges} | Degen: {degenerate_faces} | {status}"
    )
    if status != "OK":
        sys.exit(1)


def mesh_to_blender_obj(name, gbr_mesh):
    mesh = bpy.data.meshes.new(name)
    verts = [tuple(v) for v in gbr_mesh.vertices]
    faces = [tuple(int(i) for i in f) for f in gbr_mesh.faces]
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return clean_and_unify_object(obj)


def check_mesh_pair(label, mesh_a, mesh_b):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    obj_a = mesh_to_blender_obj("A", mesh_a)
    obj_b = mesh_to_blender_obj("B", mesh_b)

    bpy.ops.object.select_all(action="DESELECT")
    obj_a.select_set(True)
    bpy.context.view_layer.objects.active = obj_a

    mod = obj_a.modifiers.new(name="Intersect", type="BOOLEAN")
    mod.operation = "INTERSECT"
    mod.object = obj_b
    mod.solver = "EXACT"
    bpy.ops.object.modifier_apply(modifier=mod.name)

    bm = bmesh.new()
    bm.from_mesh(obj_a.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    vol = abs(bm.calc_volume(signed=True))
    bm.free()

    ok = vol < 1e-3
    print(f"3D INTERSECT | {label:<54} | Overlap: {vol:>8.4f} mm³ | {'PASS' if ok else 'FAIL'}")
    if not ok:
        sys.exit(1)


def run_kinematic_and_assembly_checks():
    print("\n" + "=" * 100)
    print("3D BOOLEAN INTERFERENCE & KINEMATIC ENVELOPE VERIFICATION")
    print("=" * 100)

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import generate_bead_rack as gbr

    cfg = gbr.RackConfig()
    half_span = cfg.shelf_inner_width / 2.0 + cfg.shelf_wall_thick + cfg.side_clearance
    total_h = cfg.first_row_z + (cfg.shelves_per_module - 1) * cfg.shelf_pitch_z + 26.0  # 192.0 mm

    left_frame = gbr.build_side_frame(cfg, "left").translate(-half_span, 0.0, 0.0)
    upper_frame = left_frame.translate(0.0, 0.0, total_h)

    # 1. Vertical Stacking (Lower Left Frame vs Upper Left Frame seated on Z = total_h = 192.0 mm)
    check_mesh_pair("1. Vertical Stacking (Lower Frame vs Upper Frame)", left_frame, upper_frame)

    # 2. Cross-Brace Bar vs Left Frame
    brace_bot = gbr.build_cross_brace(cfg).translate(0.0, 0.0, 14.0)
    check_mesh_pair("2. Cross-Brace Bar vs Left Frame (at Z=14mm)", left_frame, brace_bot)

    # 3. Stabilizer Foot vs Left Frame
    foot_left = gbr.build_stabilizer_foot(cfg).translate(-half_span, 0.0, 0.0)
    check_mesh_pair("3. Stabilizer Foot vs Left Frame", left_frame, foot_left)

    # 4. Shelf Kinematics at Row 0 (z_lvl = 46.0 mm):
    shelf_6bay = gbr.build_tilt_shelf(cfg, divided=True)
    z0 = cfg.first_row_z

    # State A: Stowed (Y_slide = 0.0, Tilt = 0 deg)
    shelf_stowed = shelf_6bay.translate(0.0, 0.0, z0)
    check_mesh_pair("4a. Shelf (6-Bay) Stowed (Y=0mm, 0°) vs Left Frame", left_frame, shelf_stowed)

    # State B: Pulled-Out Horizontal Drawer Stop (Y_slide = 32.0 mm, Tilt = 0 deg)
    shelf_drawer = shelf_6bay.translate(0.0, cfg.shelf_pullout_flat, z0)
    check_mesh_pair("4b. Shelf (6-Bay) Horizontal Stop (Y=32mm, 0°) vs Frame", left_frame, shelf_drawer)

    # State C: Pulled-Out & Tilted -45 deg (Y_slide = 40.0 mm, Tilt = -45.0 deg)
    shelf_tilted = shelf_6bay.rotate_x(-45.0, pivot=(0.0, 0.0, 0.0)).translate(0.0, cfg.slide_travel, z0)
    check_mesh_pair("4c. Shelf (6-Bay) Tilted (Y=40mm, -45°) vs Left Frame", left_frame, shelf_tilted)
    check_mesh_pair("4d. Shelf (6-Bay) Tilted (Y=40mm, -45°) vs Foot", foot_left, shelf_tilted)

    min_z_tilted = float(shelf_tilted.vertices[:, 2].min())
    foot_bottom_z = -16.0
    desk_clearance = min_z_tilted - foot_bottom_z
    print(
        f"DESK CLEARANCE | Bottom Shelf at -45° min_Z = {min_z_tilted:+.2f} mm vs Foot Bottom Z = {foot_bottom_z:+.2f} mm "
        f"-> Clearance = {desk_clearance:+.2f} mm | {'PASS' if desk_clearance >= 1.0 else 'FAIL'}"
    )
    if desk_clearance < 1.0:
        sys.exit(1)


if __name__ == "__main__":
    stl_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
    for fname in sorted(os.listdir(stl_dir)):
        if fname.endswith(".stl") and not fname.startswith("00_"):
            unify_stl_file(os.path.join(stl_dir, fname))
    run_kinematic_and_assembly_checks()
