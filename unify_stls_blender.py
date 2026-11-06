#!/usr/bin/env python3
"""
Blender 5.2 Manifold Boolean Unifier & Mesh Cleaner
===================================================
Imports each generated STL in `stl/`, separates loose sub-primitives,
performs an exact/manifold self-union boolean (`solver='MANIFOLD'` or `'EXACT'`)
to remove all internal coplanar/buried faces where sub-parts overlap,
verifies 0 non-manifold edges, and exports clean binary STLs.
"""

import os
import bpy
import bmesh


def unify_stl_file(filepath):
    # Clear scene
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # Import STL
    bpy.ops.wm.stl_import(filepath=filepath)
    obj = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = obj

    # Enter edit mode, merge vertices by distance per loose island first, or use Boolean Self-Union!
    # Wait: in Blender 4.0+/5.2, a Boolean modifier with solver='EXACT' and use_self=True on a mesh
    # (or separating loose parts and unioning them) dissolves all internal intersecting faces!
    # Even simpler in bmesh / Blender:
    # Add a dummy non-overlapping tiny cube far away, or separate into 2 groups and boolean union with use_self=True!
    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.object.mode_set(mode="OBJECT")

    # Separate loose parts so we can boolean-union them with solver='MANIFOLD' or 'EXACT' + use_self
    bpy.ops.mesh.separate(type="LOOSE")
    parts = list(bpy.context.selected_objects)

    if len(parts) > 1:
        base = parts[0]
        bpy.context.view_layer.objects.active = base
        # Join all remaining parts into one target object and boolean-union with use_self=True
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

        # Delete `other`
        bpy.data.objects.remove(other, do_unlink=True)
        obj = base

    # Clean up redundant coplanar vertices/normals and check manifold status
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-4)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    non_manifold_edges = sum(1 for e in bm.edges if not e.is_manifold)
    num_faces = len(bm.faces)
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.object.mode_set(mode="OBJECT")

    # Export binary STL
    bpy.ops.wm.stl_export(filepath=filepath, export_selected_objects=True, ascii_format=False)
    print(f"UNIFIED {os.path.basename(filepath):<30} | Tris: {num_faces:>6,d} | Non-Manifold Edges: {non_manifold_edges}")


if __name__ == "__main__":
    stl_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
    for fname in sorted(os.listdir(stl_dir)):
        if fname.endswith(".stl") and not fname.startswith("00_"):
            unify_stl_file(os.path.join(stl_dir, fname))
