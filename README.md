# Modular 3D-Printed Bead Storage Rack (v3 — DFM & Strength-Hardened Edition)

A 100% support-free, watertight manifold 3D-printed rack system designed for flat bead organizer cases (Tic-Tac style boxes, Elizabeth Ward trays, and shallow craft cases).

## Key Features
- **Generous 56 mm Vertical Row Spacing (`34 mm` Clear Opening):** Gives tall or standing bead boxes plenty of headroom and prevents tilted shelves from obstructing adjacent rows.
- **Universal `< 200 mm` Bed Compatibility:** Each 3-tier vertical frame module is `193.5 mm` long (including top stacking tenons), fitting straight on every `200×200`, `210×210`, and `220×220 mm` printer bed without diagonal placement.
- **Pull-Out Drawer + 45° Tilt-Down Kinematics:**
  - **Stowed (`0 mm`, `0°`):** Sits flush inside the rack with a rear detent catch.
  - **Pulled-Out Flat Shelf (`+40 mm`, `0°`):** Slides straight out horizontally like a drawer using a stepped dual-pin runner channel.
  - **Pulled-Out + 45° Tilted (`+40 mm`, `45°`):** At full extension (`40 mm`), the shorter front guide pin (`Ø6.0 × 2.5 mm`) clears the horizontal runner rail while the longer rear trunnion (`Ø7.0 × 5.2 mm`) seats into the front C-clip pivot socket—allowing the shelf to tilt down `45°` onto a solid structural rest ramp.
  - **Compliant Cantilever Snap-Latch:** Features a `14 mm` flex beam with a `2.2 mm` deflection relief slot and `30°` entry/exit ramps so shelves click in and out smoothly by hand (~12 N) without cracking layers.
- **DFM & Structural Hardening (v3 Audit Verified):**
  - **0 Non-Manifold Edges:** All 7 printable STL files are unified via Blender 5.2 Exact Boolean Union so slicers generate continuous, unbroken structural perimeters.
  - **≤ 0.1% Steep Overhangs:** Horizontal pins use flat-bottomed `45°` teardrop profiles with `45°` root shear gussets anchored to the print bed; cross-braces and U-clips print flat on `Z = 0`.

---

## Included STL Files (`stl/`)

All printable STL files are pre-oriented to lie **flat on the build plate (`Z = 0`) with zero supports**:

| File | Dimensions (X × Y × Z) | Qty per 3-Tier Column | Description |
| :--- | :--- | :---: | :--- |
| `01_tilt_shelf_6bay.stl` | `146.4 × 82.6 × 22.0 mm` | 3 (or mix) | Click-in pull-out/tilt shelf with 6 divided bays, `Ø7.0`/`Ø6.0 mm` 45°-gusseted pins, front finger scoops, and pull handle |
| `02_tilt_shelf_open_tray.stl` | `146.4 × 82.6 × 22.0 mm` | Optional | Undivided open-deck shelf (`128 × 68 mm` interior) with 4 stiffening floor ribs for large bead cases |
| `03_side_frame_left.stl` | `193.5 × 96.0 × 10.8 mm` | 1 | Left vertical frame module (`56 mm` row pitch, stepped `40 mm` drawer tracks, compliant snap-beam catch) |
| `04_side_frame_right.stl` | `193.5 × 96.0 × 10.8 mm` | 1 | Right vertical frame module (mirror of left frame, pre-oriented flat on outer face) |
| `05_cross_brace_bar.stl` | `161.2 × 11.3 × 26.5 mm` | 2 | Top & bottom rear dovetail cross-brace bars with `45°` gabled wall-mount screw slots |
| `06_stabilizer_foot.stl` | `14.6 × 136.0 × 17.0 mm` | 2 | Detachable `6.0 mm` thick front anti-tip desk foot (`46 mm` outrigger; mirror one in slicer for right side) |
| `07_connector_clips.stl` | `24.7 × 34.0 × 18.0 mm` | 1 set per joint | XY-contour printed U-channel clips for locking two rack columns side-by-side (continuous perimeter loops) |
| `00_complete_assembly_preview.stl` | `161.2 × 147.5 × 215.0 mm` | Reference only | Full 3-tier assembly reference showing Stowed (`0 mm`), Pulled-Out (`40 mm`, `0°`), and Tilted (`40 mm`, `45°`) states |

---

## Recommended 3D Print Settings

- **Material:** **PETG** (recommended for best snap-fit compliance and toughness) or **PLA+ / Tough PLA / ABS**.
- **Layer Height:** `0.20 mm` (`0.4 mm` nozzle).
- **Perimeters / Walls:** **4 walls** (`1.6–1.8 mm`) so the `Ø7.0 mm` trunnion pins, `2.4 mm` compliant flex beam, and U-clips print as solid concentric perimeters.
- **Top / Bottom Layers:** 5 layers.
- **Infill:** `20%–25%` Gyroid or Cubic.
- **Supports:** **None required** (`0` supports on all 7 parts).

---

## Interactive 3D Preview
Open `preview_viewer.html` in Google Chrome or any modern browser to interactively orbit the 3D assembly, test the 4-state shelf kinematics, and review the DFM & structural engineering table.
