"""Parametric CAD generator for the Modular Concert Penlight Display Rack.

Generates print-ready STL, STEP, and 3MF files for a modular, katana-style
horizontal display rack designed for standard cylindrical concert penlights
(~30 mm tube diameter, ~250–270 mm length).

Supports stacking 1 to 6 tiers while keeping total height (including top cap
and top penlight) under the 330 mm (33 cm) maximum height limit.

Run from the repository root:
    uv run python projects/concert-penlight-rack/src/generate.py
"""

from __future__ import annotations

import base64
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from build123d import (
    Align,
    Axis,
    Box,
    BuildLine,
    BuildPart,
    BuildSketch,
    Circle,
    Cylinder,
    Line,
    Location,
    Locations,
    Mesher,
    Mode,
    Part,
    Plane,
    Polygon,
    Rectangle,
    chamfer,
    export_step,
    export_stl,
    extrude,
    fillet,
    make_face,
)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import penlight

# ---------------------------------------------------------------------------
# Parametric Dimensions (all units in millimeters)
# ---------------------------------------------------------------------------

# Target penlight (measured concert penlight with sculpted handle)
PENLIGHT_DIAMETER = penlight.TUBE_DIAMETER  # 30.0 mm tube
PENLIGHT_LENGTH = penlight.TOTAL_LENGTH  # 266.5 mm total

# Rack constraints & modularity
MAX_TOTAL_HEIGHT = 330.0
DEFAULT_TIERS = 6
PRINTER_BUILD_PLATE = 256.0  # Bambu Lab A1 / P1 / X1

# Tower & spine dimensions
TOWER_THICKNESS = 18.0  # Extrusion thickness of each upright tower (print Z)
SPINE_WIDTH = 24.0  # Front-to-back spine width (X = 0..24)
TIER_PITCH = 48.0  # Vertical height per stacked tier

# Cradle geometry (relative to tier bottom Y = 0)
# 36.0 mm ID clears the 30.0 mm tube, 32.4 mm sculpted handle swell, and 35.0 mm chrome collar
CRADLE_CLEARANCE = 3.0  # Radial clearance around 30 mm tube -> 36.0 mm ID
CRADLE_INNER_R = PENLIGHT_DIAMETER / 2.0 + CRADLE_CLEARANCE  # 18.0 mm
CRADLE_WALL = 4.5
CRADLE_OUTER_R = CRADLE_INNER_R + CRADLE_WALL  # 22.5 mm
CRADLE_CENTER_X = 45.5  # Forward of spine (X = 24)
CRADLE_CENTER_Y = 29.0  # Lowest outer cradle point at Y = 6.5 mm
CRADLE_FRONT_ANGLE_DEG = -34.0  # Front lip angle (gives ~7.9 mm lip height)
CRADLE_BACK_ANGLE_DEG = 165.0  # Back cradle wall blends into spine

# Base foot dimensions
BASE_HEIGHT = 20.0
BASE_REAR_EXT = 12.0  # Extends behind spine (X = -12..0)
BASE_FRONT_EXT = 64.0  # Extends in front of spine (X = 24..88 -> 100 mm total)
BASE_TOE_HEIGHT = 10.0  # Tapered height at front/rear tips

# Top cap dimensions
TOP_CAP_HEIGHT = 14.0

# Interlocking dovetail joint (extruded along Z, prints flat with 0 overhangs)
DOVETAIL_NECK_W = 10.0
DOVETAIL_TOP_W = 14.0
DOVETAIL_HEIGHT = 8.0
FIT_CLEARANCE = 0.20  # Snug slip fit per side
DOVETAIL_TOP_CLEARANCE = 0.30

# Crossbar & mortise-and-tenon (connects Left & Right towers)
INNER_SPAN = 160.0  # Inner distance between Left and Right towers
TENON_SIZE = 10.0  # 10 x 10 mm square tenon
TENON_LENGTH = 17.5  # Recessed 0.5 mm inside 18 mm tower
CROSSBAR_BEAM_W = 14.0  # Beam width (Y) creating 2 mm shoulders per side
CROSSBAR_BEAM_H = 10.0  # Beam height (Z) flush with tenon bottom -> 0 overhangs
MORTISE_SIZE = TENON_SIZE + 2.0 * FIT_CLEARANCE  # 10.4 mm
TIER_MORTISE_Y = 26.0
BASE_MORTISE_Y = 10.0

# Edge treatments
EDGE_CHAMFER = 0.5

# Output directories
PROJECT_DIR = Path(__file__).resolve().parent.parent
EXPORTS_DIR = PROJECT_DIR / "exports"
RENDERS_DIR = PROJECT_DIR / "renders"


# ---------------------------------------------------------------------------
# Geometry Builders (all parts oriented in print orientation, bottom at Z = 0)
# ---------------------------------------------------------------------------


def _dovetail_male_pts(base_y: float) -> list[tuple[float, float]]:
    cx = SPINE_WIDTH / 2.0
    nw = DOVETAIL_NECK_W / 2.0
    tw = DOVETAIL_TOP_W / 2.0
    return [
        (cx - nw, base_y - 0.5),  # Slight overlap into body for clean union
        (cx - nw, base_y),
        (cx - tw, base_y + DOVETAIL_HEIGHT),
        (cx + tw, base_y + DOVETAIL_HEIGHT),
        (cx + nw, base_y),
        (cx + nw, base_y - 0.5),
    ]


def _dovetail_female_pts(base_y: float = 0.0) -> list[tuple[float, float]]:
    cx = SPINE_WIDTH / 2.0
    slope = ((DOVETAIL_TOP_W - DOVETAIL_NECK_W) / 2.0) / DOVETAIL_HEIGHT
    h = DOVETAIL_HEIGHT + DOVETAIL_TOP_CLEARANCE
    nw = DOVETAIL_NECK_W / 2.0 + FIT_CLEARANCE
    tw = nw + slope * h
    return [
        (cx - nw, base_y - 1.0),
        (cx - nw, base_y),
        (cx - tw, base_y + h),
        (cx + tw, base_y + h),
        (cx + nw, base_y),
        (cx + nw, base_y - 1.0),
    ]


def build_base_foot() -> Part:
    """Build the base foot in print orientation (flat on side, Z = 0..18)."""
    x_min = -BASE_REAR_EXT
    x_max = SPINE_WIDTH + BASE_FRONT_EXT

    with BuildPart() as bp:
        with BuildSketch(Plane.XY):
            # Main foot profile with tapered front and rear toes
            Polygon(
                (x_min, 0.0),
                (x_max, 0.0),
                (x_max, BASE_TOE_HEIGHT),
                (SPINE_WIDTH + 6.0, BASE_HEIGHT),
                (0.0, BASE_HEIGHT),
                (x_min, BASE_TOE_HEIGHT),
                align=None,
            )
            # Male dovetail on top of spine seat
            Polygon(*_dovetail_male_pts(BASE_HEIGHT), align=None)
            # Crossbar mortise cutout
            with Locations((SPINE_WIDTH / 2.0, BASE_MORTISE_Y)):
                Rectangle(MORTISE_SIZE, MORTISE_SIZE, mode=Mode.SUBTRACT)

        extrude(amount=TOWER_THICKNESS)

        # Gentle chamfer on outer top/bottom perimeter faces for clean edges
        top_bottom_edges = bp.edges().filter_by(
            lambda e: abs(e.center().Z) < 0.1
            or abs(e.center().Z - TOWER_THICKNESS) < 0.1
        )
        # Skip dovetail & mortise edges from chamfering? Actually 0.4 mm chamfer
        # on Z=0 and Z=18 edges acts as elephant's-foot compensation AND lead-in!
        chamfer(top_bottom_edges, length=0.4)

    return bp.part


def build_tier_module() -> Part:
    """Build one stackable tier module in print orientation (flat on side, Z = 0..18)."""
    front_rad = math.radians(CRADLE_FRONT_ANGLE_DEG)
    back_rad = math.radians(CRADLE_BACK_ANGLE_DEG)

    # Outer tangent points for the cantilevered arm connecting spine to cradle
    arm_bottom_y = CRADLE_CENTER_Y - CRADLE_OUTER_R  # 7.0 mm

    with BuildPart() as bp:
        with BuildSketch(Plane.XY):
            # 1. Vertical spine
            Polygon(
                (0.0, 0.0),
                (SPINE_WIDTH, 0.0),
                (SPINE_WIDTH, TIER_PITCH),
                (0.0, TIER_PITCH),
                align=None,
            )

            # 2. Male dovetail on top
            Polygon(*_dovetail_male_pts(TIER_PITCH), align=None)

            # 3. Support web & outer cradle ring
            Polygon(
                (SPINE_WIDTH - 2.0, arm_bottom_y + 2.0),
                (CRADLE_CENTER_X, arm_bottom_y),
                (CRADLE_CENTER_X, CRADLE_CENTER_Y),
                (SPINE_WIDTH - 2.0, CRADLE_CENTER_Y + 10.0),
                align=None,
            )
            with Locations((CRADLE_CENTER_X, CRADLE_CENTER_Y)):
                Circle(CRADLE_OUTER_R)

            # 4. Subtract open-top / front-angled wedge to form the U-cradle
            cut_r = CRADLE_OUTER_R + 20.0
            p_center = (CRADLE_CENTER_X, CRADLE_CENTER_Y)
            p_front = (
                CRADLE_CENTER_X + cut_r * math.cos(front_rad),
                CRADLE_CENTER_Y + cut_r * math.sin(front_rad),
            )
            p_top_right = (CRADLE_CENTER_X + cut_r, CRADLE_CENTER_Y + cut_r)
            p_top_left = (SPINE_WIDTH, CRADLE_CENTER_Y + cut_r)
            p_back = (
                CRADLE_CENTER_X + cut_r * math.cos(back_rad),
                CRADLE_CENTER_Y + cut_r * math.sin(back_rad),
            )
            # Clip back point so we don't cut into the vertical spine (X <= SPINE_WIDTH)
            p_back_clamped = (
                max(SPINE_WIDTH, p_back[0]),
                CRADLE_CENTER_Y
                + (CRADLE_CENTER_X - max(SPINE_WIDTH, p_back[0]))
                * math.tan(math.pi - back_rad),
            )
            Polygon(
                p_center,
                p_front,
                p_top_right,
                p_top_left,
                p_back_clamped,
                mode=Mode.SUBTRACT,
                align=None,
            )

            # 5. Subtract inner penlight cradle bore
            with Locations((CRADLE_CENTER_X, CRADLE_CENTER_Y)):
                Circle(CRADLE_INNER_R, mode=Mode.SUBTRACT)

            # 6. Subtract female dovetail pocket at bottom
            Polygon(*_dovetail_female_pts(0.0), mode=Mode.SUBTRACT, align=None)

            # 7. Subtract crossbar mortise in spine
            with Locations((SPINE_WIDTH / 2.0, TIER_MORTISE_Y)):
                Rectangle(MORTISE_SIZE, MORTISE_SIZE, mode=Mode.SUBTRACT)

        extrude(amount=TOWER_THICKNESS)

        # Round the front lip tip (vertical Z-parallel edges near front of cradle)
        front_tip_x = CRADLE_CENTER_X + (CRADLE_INNER_R + CRADLE_WALL / 2.0) * math.cos(
            front_rad
        )
        front_tip_y = CRADLE_CENTER_Y + (CRADLE_INNER_R + CRADLE_WALL / 2.0) * math.sin(
            front_rad
        )
        tip_edges = bp.edges().filter_by(Axis.Z).filter_by(
            lambda e: math.hypot(
                e.center().X - front_tip_x, e.center().Y - front_tip_y
            )
            < CRADLE_WALL + 1.5
        )
        if tip_edges:
            fillet(tip_edges, radius=1.8)

        # Smooth internal load-bearing fillet where cradle arm meets spine underside
        under_arm_edges = bp.edges().filter_by(Axis.Z).filter_by(
            lambda e: abs(e.center().X - SPINE_WIDTH) < 1.0
            and abs(e.center().Y - (arm_bottom_y + 2.0)) < 2.0
        )
        if under_arm_edges:
            fillet(under_arm_edges, radius=4.0)

        # Elephant's foot / lead-in chamfer on Z=0 and Z=18 faces
        z_face_edges = bp.edges().filter_by(
            lambda e: abs(e.center().Z) < 0.1
            or abs(e.center().Z - TOWER_THICKNESS) < 0.1
        )
        chamfer(z_face_edges, length=0.4)

    return bp.part


def build_top_cap() -> Part:
    """Build the finishing top cap in print orientation (flat on side, Z = 0..18)."""
    bevel = 3.0
    with BuildPart() as bp:
        with BuildSketch(Plane.XY):
            Polygon(
                (0.0, 0.0),
                (SPINE_WIDTH, 0.0),
                (SPINE_WIDTH, TOP_CAP_HEIGHT - bevel),
                (SPINE_WIDTH - bevel, TOP_CAP_HEIGHT),
                (bevel, TOP_CAP_HEIGHT),
                (0.0, TOP_CAP_HEIGHT - bevel),
                align=None,
            )
            Polygon(*_dovetail_female_pts(0.0), mode=Mode.SUBTRACT, align=None)

        extrude(amount=TOWER_THICKNESS)

        z_face_edges = bp.edges().filter_by(
            lambda e: abs(e.center().Z) < 0.1
            or abs(e.center().Z - TOWER_THICKNESS) < 0.1
        )
        chamfer(z_face_edges, length=0.4)

    return bp.part


def build_crossbar() -> Part:
    """Build the horizontal crossbar in print orientation (flat on Z = 0, 0 overhangs)."""
    total_len = INNER_SPAN + 2.0 * TENON_LENGTH

    with BuildPart() as bp:
        with BuildSketch(Plane.XY):
            # Center shouldered beam
            Rectangle(INNER_SPAN, CROSSBAR_BEAM_W)
            # Full-length tenon core
            Rectangle(total_len, TENON_SIZE)

        extrude(amount=CROSSBAR_BEAM_H)

        # Lead-in chamfer on the two tenon end faces for effortless insertion
        end_edges = bp.edges().filter_by(
            lambda e: abs(abs(e.center().X) - total_len / 2.0) < 0.1
        )
        chamfer(end_edges, length=0.8)

        # Subtle top/bottom edge chamfer along the center beam
        z_edges = bp.edges().filter_by(
            lambda e: (
                abs(e.center().Z) < 0.1 or abs(e.center().Z - CROSSBAR_BEAM_H) < 0.1
            )
            and abs(e.center().X) < INNER_SPAN / 2.0 - 0.1
        )
        chamfer(z_edges, length=0.4)

    return bp.part


# ---------------------------------------------------------------------------
# Verification & Rendering Helpers
# ---------------------------------------------------------------------------


def verify_stl(path: Path) -> dict[str, object]:
    """Load exported STL with trimesh and verify printability & manifoldness."""
    mesh = trimesh.load_mesh(path, force="mesh")
    bounds = mesh.bounds  # [[min_x, min_y, min_z], [max_x, max_y, max_z]]
    extents = mesh.extents
    fits_plate = all(dim <= PRINTER_BUILD_PLATE for dim in extents)
    rests_on_z0 = abs(bounds[0][2]) < 1e-3

    info = {
        "name": path.stem,
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "volume_cm3": float(mesh.volume / 1000.0),
        "extents_mm": tuple(round(float(x), 2) for x in extents),
        "min_z_mm": round(float(bounds[0][2]), 4),
        "fits_plate": fits_plate,
        "rests_on_z0": rests_on_z0,
    }

    assert info["watertight"], f"{path.name} is NOT watertight!"
    assert info["volume_cm3"] > 0, f"{path.name} has non-positive volume!"
    assert fits_plate, f"{path.name} exceeds {PRINTER_BUILD_PLATE} mm build plate: {extents}"
    assert rests_on_z0, f"{path.name} does not rest cleanly on Z = 0 (min_z={bounds[0][2]})"
    return info


def _add_mesh_to_ax(
    ax,
    mesh: trimesh.Trimesh,
    color: str,
    alpha: float = 1.0,
    edgecolor: str = "#222222",
    linewidth: float = 0.08,
) -> None:
    verts = mesh.vertices[mesh.faces]
    # Simple directional shading for crisp readability
    normals = mesh.face_normals
    light = np.array([0.4, -0.5, 0.76])
    light /= np.linalg.norm(light)
    intensity = np.clip(0.45 + 0.55 * np.abs(normals @ light), 0.35, 1.0)

    base_rgb = np.array(matplotlib.colors.to_rgb(color))
    face_colors = np.column_stack([base_rgb * intensity[:, None], np.full(len(normals), alpha)])
    poly = Poly3DCollection(verts, facecolors=face_colors, edgecolors=edgecolor, linewidths=linewidth)
    ax.add_collection3d(poly)


def _set_equal_3d_axes(ax, bounds_min: np.ndarray, bounds_max: np.ndarray, pad: float = 10.0) -> None:
    center = (bounds_min + bounds_max) / 2.0
    max_range = float(np.max(bounds_max - bounds_min)) / 2.0 + pad
    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)


def _stand_upright_transform(tower_x_offset: float) -> np.ndarray:
    """Transform a tower part from print orientation (XY profile, Z thickness)
    to assembled orientation (X = left-right, Y = front-back depth, Z = height).
    """
    # Print (px, py, pz) -> Assembled (X = pz + offset, Y = px, Z = py)
    T = np.eye(4)
    T[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
    )
    T[0, 3] = tower_x_offset
    return T


def render_parts_overview(stl_paths: dict[str, Path], out_path: Path) -> None:
    """Render all 4 individual parts in their print orientation."""
    fig = plt.figure(figsize=(14, 10), dpi=150)
    fig.suptitle(
        "Concert Penlight Rack — Individual Parts (Print Orientation, Z = 0)",
        fontsize=14,
        fontweight="bold",
    )

    colors = {
        "base_foot": "#4A90D9",
        "tier_module": "#50B878",
        "top_cap": "#E6A23C",
        "crossbar": "#9B72CF",
    }

    for idx, (name, path) in enumerate(stl_paths.items(), start=1):
        ax = fig.add_subplot(2, 2, idx, projection="3d")
        mesh = trimesh.load_mesh(path, force="mesh")
        _add_mesh_to_ax(ax, mesh, color=colors[name])
        _set_equal_3d_axes(ax, mesh.bounds[0], mesh.bounds[1], pad=6.0)
        ext = mesh.extents
        ax.set_title(
            f"{name}.stl\n{ext[0]:.1f} × {ext[1]:.1f} × {ext[2]:.1f} mm",
            fontsize=11,
        )
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.view_init(elev=28, azim=-50)

    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def build_assembled_scene_meshes(
    stl_paths: dict[str, Path], n_tiers: int = DEFAULT_TIERS
) -> list[tuple[trimesh.Trimesh, str, float, str]]:
    """Build all positioned meshes for an N-tier assembly with sculpted concert penlights.

    Returns list of (mesh, hex_color, alpha, material_kind) tuples.
    """
    base_m = trimesh.load_mesh(stl_paths["base_foot"], force="mesh")
    tier_m = trimesh.load_mesh(stl_paths["tier_module"], force="mesh")
    cap_m = trimesh.load_mesh(stl_paths["top_cap"], force="mesh")
    bar_m = trimesh.load_mesh(stl_paths["crossbar"], force="mesh")

    assembled: list[tuple[trimesh.Trimesh, str, float, str]] = []

    left_x = -INNER_SPAN / 2.0 - TOWER_THICKNESS
    right_x = INNER_SPAN / 2.0

    for x_off in (left_x, right_x):
        T_base = _stand_upright_transform(x_off)
        m = base_m.copy()
        m.apply_transform(T_base)
        assembled.append((m, "#3B4A5A", 1.0, "rack"))

        for t in range(n_tiers):
            T_tier = _stand_upright_transform(x_off)
            T_tier[2, 3] = BASE_HEIGHT + t * TIER_PITCH
            mt = tier_m.copy()
            mt.apply_transform(T_tier)
            assembled.append((mt, "#4F6275" if t % 2 == 0 else "#5E7388", 1.0, "rack"))

        T_cap = _stand_upright_transform(x_off)
        T_cap[2, 3] = BASE_HEIGHT + n_tiers * TIER_PITCH
        mc = cap_m.copy()
        mc.apply_transform(T_cap)
        assembled.append((mc, "#3B4A5A", 1.0, "rack"))

    # Crossbars: one in base, one in top tier
    bar_z_positions = [
        BASE_MORTISE_Y,
        BASE_HEIGHT + (n_tiers - 1) * TIER_PITCH + TIER_MORTISE_Y,
    ]
    for bz in bar_z_positions:
        mb = bar_m.copy()
        mb.apply_translation([0.0, SPINE_WIDTH / 2.0, bz - CROSSBAR_BEAM_H / 2.0])
        assembled.append((mb, "#2D3744", 1.0, "rack"))

    # Detailed concert penlights (sculpted ergonomic handle + chrome collar + tube)
    tube_colors = ["#FF4D8D", "#38BDF8", "#FACC15", "#4ADE80", "#C084FC", "#FB923C"]
    handle_colors = ["#F4A6C1", "#BAE6FD", "#FEF08A", "#BBF7D0", "#E9D5FF", "#FED7AA"]
    for t in range(n_tiers):
        cz = BASE_HEIGHT + t * TIER_PITCH + (CRADLE_CENTER_Y - CRADLE_INNER_R) + PENLIGHT_DIAMETER / 2.0
        cy = CRADLE_CENTER_X

        pm_list = penlight.build_penlight_meshes(
            tube_color=tube_colors[t % len(tube_colors)],
            handle_color=handle_colors[t % len(handle_colors)],
        )
        kinds = ["handle", "collar", "tube"]
        for (pm, pcol, palpha), kind in zip(pm_list, kinds):
            pm.apply_translation([0.0, cy, cz])
            assembled.append((pm, pcol, palpha, kind))

    return assembled


def export_interactive_assembly(
    stl_paths: dict[str, Path],
    glb_path: Path,
    html_path: Path,
    n_tiers: int = DEFAULT_TIERS,
) -> None:
    """Export the full colored assembly as a Y-up GLB and self-contained Three.js HTML viewer."""
    assembled = build_assembled_scene_meshes(stl_paths, n_tiers=n_tiers)
    scene = trimesh.Scene()

    # Convert Z-up (CAD) -> Y-up (glTF / Three.js) and scale mm -> meters? Keep mm for clarity
    rot_z_to_y = trimesh.transformations.rotation_matrix(-math.pi / 2.0, [1, 0, 0])

    for idx, (mesh, hex_col, alpha, kind) in enumerate(assembled):
        m = mesh.copy()
        m.apply_transform(rot_z_to_y)
        r, g, b = matplotlib.colors.to_rgb(hex_col)
        rgba = [int(r * 255), int(g * 255), int(b * 255), int(alpha * 255)]

        if kind == "collar":
            mat = trimesh.visual.material.PBRMaterial(
                baseColorFactor=rgba, metallicFactor=0.88, roughnessFactor=0.18
            )
        elif kind == "tube":
            mat = trimesh.visual.material.PBRMaterial(
                baseColorFactor=rgba,
                metallicFactor=0.05,
                roughnessFactor=0.25,
                alphaMode="BLEND",
            )
        elif kind == "handle":
            mat = trimesh.visual.material.PBRMaterial(
                baseColorFactor=rgba, metallicFactor=0.05, roughnessFactor=0.35
            )
        else:
            mat = trimesh.visual.material.PBRMaterial(
                baseColorFactor=rgba, metallicFactor=0.08, roughnessFactor=0.62
            )

        m.visual = trimesh.visual.TextureVisuals(material=mat)
        scene.add_geometry(m, node_name=f"{kind}_{idx}")

    glb_bytes = scene.export(file_type="glb")
    glb_path.write_bytes(glb_bytes)

    glb_b64 = base64.b64encode(glb_bytes).decode("ascii")
    total_h = BASE_HEIGHT + n_tiers * TIER_PITCH + TOP_CAP_HEIGHT

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Concert Penlight Rack — Interactive 3D Assembly</title>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{
      background: #111827;
      color: #f3f4f6;
      font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
      overflow: hidden;
    }}
    #hud {{
      position: fixed;
      top: 14px;
      left: 14px;
      background: rgba(17, 24, 39, 0.84);
      backdrop-filter: blur(8px);
      border: 1px solid rgba(255, 255, 255, 0.12);
      border-radius: 10px;
      padding: 12px 14px;
      max-width: 310px;
      z-index: 10;
      font-size: 13px;
      line-height: 1.45;
    }}
    #hud h1 {{ font-size: 15px; margin-bottom: 4px; }}
    #hud .meta {{ color: #9ca3af; font-size: 12px; margin-bottom: 10px; }}
    .btns {{ display: flex; flex-wrap: wrap; gap: 6px; margin-top: 8px; }}
    button {{
      background: #1f2937;
      color: #e5e7eb;
      border: 1px solid #374151;
      border-radius: 6px;
      padding: 5px 9px;
      font-size: 12px;
      cursor: pointer;
    }}
    button:hover {{ background: #374151; }}
    button.active {{ background: #2563eb; border-color: #3b82f6; color: #fff; }}
    label {{ display: flex; align-items: center; gap: 6px; margin-top: 6px; cursor: pointer; }}
  </style>
  <script type="importmap">
    {{
      "imports": {{
        "three": "https://unpkg.com/three@0.170.0/build/three.module.js",
        "three/addons/": "https://unpkg.com/three@0.170.0/examples/jsm/"
      }}
    }}
  </script>
</head>
<body>
  <div id="hud">
    <h1>Concert Penlight Rack ({n_tiers}-Tier)</h1>
    <div class="meta">
      Height: {total_h:.1f} mm / 330 mm max &middot; Width: {INNER_SPAN + 2 * TOWER_THICKNESS:.0f} mm<br/>
      Left-drag: Orbit &middot; Right-drag: Pan &middot; Scroll: Zoom
    </div>
    <label><input type="checkbox" id="togglePenlights" checked /> Show penlights</label>
    <label><input type="checkbox" id="toggleLimit" checked /> Show 330 mm height limit</label>
    <div class="btns">
      <button data-view="iso">Iso</button>
      <button data-view="front">Front</button>
      <button data-view="side">Side</button>
      <button data-view="top">Top</button>
      <button data-view="back">Back</button>
    </div>
  </div>
  <script type="module">
    import * as THREE from 'three';
    import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';
    import {{ GLTFLoader }} from 'three/addons/loaders/GLTFLoader.js';

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111827);

    const camera = new THREE.PerspectiveCamera(40, innerWidth / innerHeight, 1, 5000);
    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setSize(innerWidth, innerHeight);
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    document.body.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 160, -35);

    // Studio lighting
    scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.1));
    const key = new THREE.DirectionalLight(0xffffff, 1.8);
    key.position.set(250, 420, 320);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xcbd5e1, 0.9);
    fill.position.set(-280, 220, -240);
    scene.add(fill);

    // Floor grid (10 mm minor, 50 mm major)
    const grid = new THREE.GridHelper(500, 50, 0x475569, 0x1e293b);
    scene.add(grid);

    // 330 mm max height ceiling indicator
    const limitGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(280, 1, 130));
    const limitMat = new THREE.LineBasicMaterial({{ color: 0xef4444 }});
    const limitBox = new THREE.LineSegments(limitGeo, limitMat);
    limitBox.position.set(0, {MAX_TOTAL_HEIGHT:.1f}, -35);
    scene.add(limitBox);

    // Load embedded GLB
    const b64 = "{glb_b64}";
    const raw = Uint8Array.from(atob(b64), c => c.charCodeAt(0)).buffer;
    const penlightNodes = [];

    new GLTFLoader().parse(raw, '', (gltf) => {{
      const root = gltf.scene;
      root.traverse((obj) => {{
        if (obj.name && (obj.name.startsWith('tube_') || obj.name.startsWith('handle_') || obj.name.startsWith('collar_'))) {{
          penlightNodes.push(obj);
        }}
      }});
      scene.add(root);
      setView('iso');
    }});

    function setView(name) {{
      const t = new THREE.Vector3(0, 160, -35);
      controls.target.copy(t);
      const pos = {{
        iso:   new THREE.Vector3(290, 270, -340),
        front: new THREE.Vector3(0, 165, -480),
        side:  new THREE.Vector3(460, 165, -35),
        top:   new THREE.Vector3(0, 620, -36),
        back:  new THREE.Vector3(0, 165, 420),
      }}[name];
      camera.position.copy(pos);
      controls.update();
    }}

    document.querySelectorAll('button[data-view]').forEach(btn => {{
      btn.addEventListener('click', () => setView(btn.dataset.view));
    }});
    document.getElementById('togglePenlights').addEventListener('change', (e) => {{
      penlightNodes.forEach(n => {{ n.visible = e.target.checked; }});
    }});
    document.getElementById('toggleLimit').addEventListener('change', (e) => {{
      limitBox.visible = e.target.checked;
    }});

    addEventListener('resize', () => {{
      camera.aspect = innerWidth / innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(innerWidth, innerHeight);
    }});

    (function animate() {{
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    }})();
  </script>
</body>
</html>
"""
    html_path.write_text(html, encoding="utf-8")


def render_assembly_views(stl_paths: dict[str, Path], out_path: Path, n_tiers: int = DEFAULT_TIERS) -> None:
    """Render full 6-tier assembled rack with 6 detailed concert penlights."""
    assembled = [
        (m, col, alpha) for m, col, alpha, _ in build_assembled_scene_meshes(stl_paths, n_tiers=n_tiers)
    ]

    all_min = np.min([m.bounds[0] for m, _, _ in assembled], axis=0)
    all_max = np.max([m.bounds[1] for m, _, _ in assembled], axis=0)

    total_height = BASE_HEIGHT + n_tiers * TIER_PITCH + TOP_CAP_HEIGHT
    top_penlight_z = (
        BASE_HEIGHT
        + (n_tiers - 1) * TIER_PITCH
        + CRADLE_CENTER_Y
        - CRADLE_CLEARANCE
        + PENLIGHT_DIAMETER / 2.0
    )

    fig = plt.figure(figsize=(15, 11), dpi=150)
    fig.suptitle(
        f"Concert Penlight Rack — {n_tiers}-Tier Assembly  "
        f"(Rack Height: {total_height:.1f} mm | Top Penlight: {top_penlight_z:.1f} mm | Limit: {MAX_TOTAL_HEIGHT:.0f} mm)",
        fontsize=13,
        fontweight="bold",
    )

    views = [
        ("Isometric View (Front)", 22, 52),
        ("Front View", 5, 90),
        ("Side Profile (Right)", 3, 0),
        ("Top-Down View", 82, 90),
    ]

    for idx, (title, elev, azim) in enumerate(views, start=1):
        ax = fig.add_subplot(2, 2, idx, projection="3d")
        for mesh, color, alpha in assembled:
            _add_mesh_to_ax(ax, mesh, color=color, alpha=alpha, linewidth=0.04)

        # Draw 330 mm max height reference frame on isometric/front/side views
        if elev < 60:
            x0, x1 = -130, 130
            y0, y1 = -15, 95
            z_lim = MAX_TOTAL_HEIGHT
            ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], [z_lim] * 5, "r--", linewidth=1.2, label="330 mm limit")

        _set_equal_3d_axes(ax, all_min, np.maximum(all_max, [0, 0, MAX_TOTAL_HEIGHT]), pad=12.0)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.view_init(elev=elev, azim=azim)

    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def render_side_profile_detail(stl_paths: dict[str, Path], out_path: Path) -> None:
    """Render a crisp 2D cross-section/side-profile showing base + 2 tiers + penlights + clearances."""
    base_m = trimesh.load_mesh(stl_paths["base_foot"], force="mesh")
    tier_m = trimesh.load_mesh(stl_paths["tier_module"], force="mesh")

    fig, ax = plt.subplots(figsize=(8, 9), dpi=150)
    ax.set_title("Side Profile Detail — Base + 2 Tiers & Penlight Clearances", fontsize=12, fontweight="bold")

    # Slice each mesh at mid-plane Z = TOWER_THICKNESS / 2 to get exact 2D outline
    def plot_slice(mesh: trimesh.Trimesh, y_offset: float, color: str, label: str | None = None):
        sec = mesh.section(plane_origin=[0, 0, TOWER_THICKNESS / 2.0], plane_normal=[0, 0, 1])
        if sec is None:
            return
        for ent in sec.entities:
            pts = sec.vertices[ent.points]
            ax.plot(pts[:, 0], pts[:, 1] + y_offset, color=color, linewidth=1.8, label=label)
            label = None

    plot_slice(base_m, 0.0, "#1E3A5F", "Base Foot")
    plot_slice(tier_m, BASE_HEIGHT, "#2E7D32", "Tier 1")
    plot_slice(tier_m, BASE_HEIGHT + TIER_PITCH, "#1565C0", "Tier 2")

    # Draw Ø30 mm penlight cross-sections resting in Tier 1 & Tier 2 cradles
    for t in range(2):
        cy = BASE_HEIGHT + t * TIER_PITCH + CRADLE_CENTER_Y - CRADLE_CLEARANCE
        circ = plt.Circle(
            (CRADLE_CENTER_X, cy),
            PENLIGHT_DIAMETER / 2.0,
            facecolor="#FF6B9D",
            edgecolor="#B0003A",
            alpha=0.45,
            linewidth=1.5,
            label="Ø30 mm Penlight" if t == 0 else None,
        )
        ax.add_patch(circ)

    # Annotate vertical clearance between Tier 1 penlight top and Tier 2 cradle bottom
    p1_top = BASE_HEIGHT + CRADLE_CENTER_Y - CRADLE_CLEARANCE + PENLIGHT_DIAMETER / 2.0
    t2_bot = BASE_HEIGHT + TIER_PITCH + (CRADLE_CENTER_Y - CRADLE_OUTER_R)
    gap = t2_bot - p1_top
    ax.annotate(
        f"Lift clearance: {gap:.1f} mm",
        xy=(CRADLE_CENTER_X + 20, (p1_top + t2_bot) / 2.0),
        xytext=(CRADLE_CENTER_X + 30, (p1_top + t2_bot) / 2.0),
        fontsize=9,
        color="#B0003A",
        arrowprops={"arrowstyle": "->", "color": "#B0003A"},
    )

    ax.axhline(0, color="#333333", linewidth=1.0, linestyle="-")
    ax.set_aspect("equal")
    ax.set_xlim(-22, 108)
    ax.set_ylim(-6, BASE_HEIGHT + 2 * TIER_PITCH + 16)
    ax.set_xlabel("Depth X (mm)")
    ax.set_ylabel("Height Z (mm)")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.legend(loc="upper right")
    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def render_fit_verification(stl_paths: dict[str, Path], out_path: Path) -> None:
    """Render longitudinal + cross-section fit verification for the sculpted concert penlight."""
    tier_m = trimesh.load_mesh(stl_paths["tier_module"], force="mesh")
    stations = penlight.get_profile_stations()
    half_len = penlight.TOTAL_LENGTH / 2.0

    xs = np.array([s.z - half_len for s in stations])
    rs = np.array([s.radius for s in stations])

    fig = plt.figure(figsize=(14, 9), dpi=150)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.15])
    fig.suptitle(
        "Concert Penlight Fit Verification (Measured Sculpted Handle vs. Ø36.0 mm Cradles)",
        fontsize=13,
        fontweight="bold",
    )

    # Top: Longitudinal profile across Left (handle) & Right (tube) cradles
    ax_long = fig.add_subplot(gs[0, :])
    ax_long.fill_between(xs, -rs, rs, color="#F4A6C1", alpha=0.35)
    ax_long.plot(xs, rs, color="#B0003A", linewidth=1.8, label="Modeled Concert Penlight")
    ax_long.plot(xs, -rs, color="#B0003A", linewidth=1.8)

    # Highlight chrome collar region
    c0 = penlight.HANDLE_LENGTH - half_len
    c1 = penlight.HANDLE_LENGTH + penlight.COLLAR_LENGTH - half_len
    ax_long.axvspan(c0, c1, color="#9CA3AF", alpha=0.35, label="Chrome Collar (Ø34.5–35.0 mm)")

    # Draw Left & Right cradle spans
    left_x0, left_x1 = -INNER_SPAN / 2.0 - TOWER_THICKNESS, -INNER_SPAN / 2.0
    right_x0, right_x1 = INNER_SPAN / 2.0, INNER_SPAN / 2.0 + TOWER_THICKNESS
    for x0, x1, lbl in [
        (left_x0, left_x1, "Handle Cradle (18 mm wide)"),
        (right_x0, right_x1, "Tube Cradle (18 mm wide)"),
    ]:
        ax_long.axvspan(x0, x1, color="#2563EB", alpha=0.20, label=lbl)
        ax_long.plot([x0, x1], [-CRADLE_INNER_R, -CRADLE_INNER_R], color="#1D4ED8", linewidth=3.0)

    # Key dimension callouts
    ax_long.annotate(
        f"Palm swell\nØ{penlight.HANDLE_SWELL_DIAMETER:.1f} mm",
        xy=(24.0 - half_len, penlight.HANDLE_SWELL_DIAMETER / 2.0),
        xytext=(24.0 - half_len - 18, 24.0),
        fontsize=9,
        arrowprops={"arrowstyle": "->"},
    )
    ax_long.annotate(
        f"Waist\nØ{penlight.HANDLE_WAIST_DIAMETER:.1f} mm",
        xy=(58.0 - half_len, penlight.HANDLE_WAIST_DIAMETER / 2.0),
        xytext=(58.0 - half_len - 10, 24.0),
        fontsize=9,
        arrowprops={"arrowstyle": "->"},
    )
    ax_long.annotate(
        f"Collar\nØ{penlight.COLLAR_RING_DIAMETER:.1f} mm",
        xy=((c0 + c1) / 2.0, penlight.COLLAR_RING_DIAMETER / 2.0),
        xytext=((c0 + c1) / 2.0 - 8, 25.0),
        fontsize=9,
        arrowprops={"arrowstyle": "->"},
    )
    ax_long.annotate(
        f"Light tube\nØ{penlight.TUBE_DIAMETER:.1f} mm",
        xy=(45.0, penlight.TUBE_DIAMETER / 2.0),
        xytext=(38.0, 24.0),
        fontsize=9,
        arrowprops={"arrowstyle": "->"},
    )

    ax_long.set_aspect("equal")
    ax_long.set_xlim(-half_len - 12, half_len + 12)
    ax_long.set_ylim(-28, 36)
    ax_long.set_xlabel("Axial Position X (mm, 0 = Rack Center)")
    ax_long.set_ylabel("Radial Profile (mm)")
    ax_long.grid(True, linestyle=":", alpha=0.5)
    ax_long.legend(loc="lower right", ncol=2, fontsize=8.5)

    # Helper to draw cradle 2D cross-section
    sec = tier_m.section(plane_origin=[0, 0, TOWER_THICKNESS / 2.0], plane_normal=[0, 0, 1])

    def plot_cradle_check(ax, title: str, circles: list[tuple[float, str, str, str]]):
        if sec is not None:
            for ent in sec.entities:
                pts = sec.vertices[ent.points]
                ax.plot(pts[:, 0], pts[:, 1], color="#1E3A5F", linewidth=2.0)
        cradle_bot = CRADLE_CENTER_Y - CRADLE_INNER_R
        for diam, fill, edge, lbl in circles:
            r = diam / 2.0
            c = plt.Circle(
                (CRADLE_CENTER_X, cradle_bot + r),
                r,
                facecolor=fill,
                edgecolor=edge,
                alpha=0.35,
                linewidth=1.8,
                label=lbl,
            )
            ax.add_patch(c)
        ax.set_title(title, fontsize=10.5, fontweight="bold")
        ax.set_aspect("equal")
        ax.set_xlim(18, 72)
        ax.set_ylim(2, 54)
        ax.set_xlabel("Depth X (mm)")
        ax.set_ylabel("Tier Height Y (mm)")
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.legend(loc="upper left", fontsize=8.5)

    # Bottom-left: Handle cradle cross-section
    ax_h = fig.add_subplot(gs[1, 0])
    plot_cradle_check(
        ax_h,
        "Handle Cradle Fit (Ø36.0 mm ID)",
        [
            (penlight.HANDLE_SWELL_DIAMETER, "#EC4899", "#9D174D", f"Max Palm Swell (Ø{penlight.HANDLE_SWELL_DIAMETER:.1f} mm)"),
            (penlight.HANDLE_WAIST_DIAMETER, "#F472B6", "#BE185D", f"Handle Waist (Ø{penlight.HANDLE_WAIST_DIAMETER:.1f} mm)"),
        ],
    )

    # Bottom-right: Tube & worst-case collar fit
    ax_t = fig.add_subplot(gs[1, 1])
    plot_cradle_check(
        ax_t,
        "Tube & Shifted Collar Fit (Ø36.0 mm ID)",
        [
            (penlight.COLLAR_RING_DIAMETER, "#9CA3AF", "#374151", f"Chrome Collar (Ø{penlight.COLLAR_RING_DIAMETER:.1f} mm)"),
            (penlight.TUBE_DIAMETER, "#38BDF8", "#0369A1", f"Light Tube (Ø{penlight.TUBE_DIAMETER:.1f} mm)"),
        ],
    )

    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def _place_on_plate(part: Part, target_min_x: float, target_min_y: float, label: str) -> Part:
    """Return a copy of `part` shifted so its bounding box min corner sits at (target_min_x, target_min_y, 0)."""
    bb = part.bounding_box()
    dx = target_min_x - bb.min.X
    dy = target_min_y - bb.min.Y
    dz = -bb.min.Z
    moved = part.moved(Location((dx, dy, dz)))
    moved.label = label
    return moved


def export_print_plates(parts: dict[str, Part], out_dir: Path) -> dict[str, list[tuple[str, Part]]]:
    """Arrange ready-to-slice 256x256 mm Bambu Lab 3MF plates and verify zero overlaps."""
    gap = 8.0
    bf = parts["base_foot"]
    tm = parts["tier_module"]
    tc = parts["top_cap"]
    cb = parts["crossbar"]

    bf_w, bf_h = bf.bounding_box().size.X, bf.bounding_box().size.Y  # 100 x 28
    tm_w, tm_h = tm.bounding_box().size.X, tm.bounding_box().size.Y  # 63.4 x 56
    tc_w, tc_h = tc.bounding_box().size.X, tc.bounding_box().size.Y  # 24 x 14
    cb_w, cb_h = cb.bounding_box().size.X, cb.bounding_box().size.Y  # 195 x 14

    def center_layout(items: list[tuple[str, Part]]) -> list[tuple[str, Part]]:
        """Center a list of placed parts on the 256x256 mm build plate."""
        min_x = min(p.bounding_box().min.X for _, p in items)
        max_x = max(p.bounding_box().max.X for _, p in items)
        min_y = min(p.bounding_box().min.Y for _, p in items)
        max_y = max(p.bounding_box().max.Y for _, p in items)
        dx = (PRINTER_BUILD_PLATE - (max_x - min_x)) / 2.0 - min_x
        dy = (PRINTER_BUILD_PLATE - (max_y - min_y)) / 2.0 - min_y
        centered: list[tuple[str, Part]] = []
        for name, p in items:
            mp = p.moved(Location((dx, dy, 0.0)))
            mp.label = name
            centered.append((name, mp))
        return centered

    # Plate 1: Complete 3-tier rack (2x base_foot, 2x crossbar, 2x top_cap, 6x tier_module)
    p1_items: list[tuple[str, Part]] = []
    y = 0.0
    p1_items.append(("crossbar_1", _place_on_plate(cb, 0.0, y, "crossbar_1")))
    p1_items.append(("top_cap_1", _place_on_plate(tc, cb_w + gap, y, "top_cap_1")))
    y += max(cb_h, tc_h) + gap

    p1_items.append(("crossbar_2", _place_on_plate(cb, 0.0, y, "crossbar_2")))
    p1_items.append(("top_cap_2", _place_on_plate(tc, cb_w + gap, y, "top_cap_2")))
    y += max(cb_h, tc_h) + gap

    p1_items.append(("base_foot_1", _place_on_plate(bf, 0.0, y, "base_foot_1")))
    p1_items.append(("base_foot_2", _place_on_plate(bf, bf_w + gap, y, "base_foot_2")))
    y += bf_h + gap

    for row in range(2):
        for col in range(3):
            idx = row * 3 + col + 1
            p1_items.append(
                (f"tier_module_{idx}", _place_on_plate(tm, col * (tm_w + gap), y, f"tier_module_{idx}"))
            )
        y += tm_h + gap

    # Plate 2: +3 tier expansion to reach 6 tiers (6x tier_module + 1x optional middle crossbar)
    p2_items: list[tuple[str, Part]] = []
    y = 0.0
    p2_items.append(("crossbar_mid", _place_on_plate(cb, 0.0, y, "crossbar_mid")))
    y += cb_h + gap
    for row in range(2):
        for col in range(3):
            idx = row * 3 + col + 7
            p2_items.append(
                (f"tier_module_{idx}", _place_on_plate(tm, col * (tm_w + gap), y, f"tier_module_{idx}"))
            )
        y += tm_h + gap

    # Bonus Plate: Quick 1-tier test-fit print (2x base_foot, 2x tier_module, 1x crossbar)
    pt_items: list[tuple[str, Part]] = []
    y = 0.0
    pt_items.append(("crossbar_test", _place_on_plate(cb, 0.0, y, "crossbar_test")))
    y += cb_h + gap
    pt_items.append(("base_foot_1", _place_on_plate(bf, 0.0, y, "base_foot_1")))
    pt_items.append(("base_foot_2", _place_on_plate(bf, bf_w + gap, y, "base_foot_2")))
    y += bf_h + gap
    pt_items.append(("tier_module_1", _place_on_plate(tm, 0.0, y, "tier_module_1")))
    pt_items.append(("tier_module_2", _place_on_plate(tm, tm_w + gap, y, "tier_module_2")))

    plates = {
        "plate1_base_and_3tiers": center_layout(p1_items),
        "plate2_expansion_3tiers": center_layout(p2_items),
        "plate_test_fit_1tier": center_layout(pt_items),
    }

    print("\n=== Exporting & Verifying 3MF Print Plates (256x256 mm) ===")
    for plate_name, items in plates.items():
        # Verify zero 2D bbox overlaps & generous build plate margins
        boxes = [(name, p.bounding_box()) for name, p in items]
        for i in range(len(boxes)):
            n1, b1 = boxes[i]
            assert b1.min.X >= 5.0 and b1.max.X <= PRINTER_BUILD_PLATE - 5.0, f"{plate_name}/{n1} X out of safe plate bounds!"
            assert b1.min.Y >= 5.0 and b1.max.Y <= PRINTER_BUILD_PLATE - 5.0, f"{plate_name}/{n1} Y out of safe plate bounds!"
            assert abs(b1.min.Z) < 1e-3, f"{plate_name}/{n1} not on Z=0!"
            for j in range(i + 1, len(boxes)):
                n2, b2 = boxes[j]
                overlap_x = b1.min.X < b2.max.X and b2.min.X < b1.max.X
                overlap_y = b1.min.Y < b2.max.Y and b2.min.Y < b1.max.Y
                assert not (overlap_x and overlap_y), f"Overlap on {plate_name} between {n1} and {n2}!"

        out_3mf = out_dir / f"{plate_name}.3mf"
        mesher = Mesher()
        for name, p in items:
            mesher.add_shape(p, part_number=name)
        mesher.write(str(out_3mf))

        min_x = min(b.min.X for _, b in boxes)
        max_x = max(b.max.X for _, b in boxes)
        min_y = min(b.min.Y for _, b in boxes)
        max_y = max(b.max.Y for _, b in boxes)
        print(
            f"  [OK] {out_3mf.name:28s} | {len(items):2d} parts | "
            f"span=({max_x - min_x:.1f} x {max_y - min_y:.1f}) mm | "
            f"margins=({min_x:.1f}, {min_y:.1f}) mm"
        )

    return plates


def render_print_plates(plates: dict[str, list[tuple[str, Part]]], out_path: Path) -> None:
    """Render top-down layout diagrams of all 256x256 mm 3MF print plates."""
    fig, axes = plt.subplots(1, len(plates), figsize=(15, 5.5), dpi=150)
    fig.suptitle(
        "Ready-to-Slice 3MF Build Plates (256 × 256 mm Bambu Lab Bed)",
        fontsize=13,
        fontweight="bold",
    )

    color_map = {
        "base_foot": "#4A90D9",
        "tier_module": "#50B878",
        "top_cap": "#E6A23C",
        "crossbar": "#9B72CF",
    }

    for ax, (plate_name, items) in zip(axes, plates.items()):
        # Draw build plate outline + 10 mm safe border
        ax.add_patch(plt.Rectangle((0, 0), PRINTER_BUILD_PLATE, PRINTER_BUILD_PLATE, facecolor="#F8FAFC", edgecolor="#1E293B", linewidth=2.0))
        ax.add_patch(plt.Rectangle((10, 10), PRINTER_BUILD_PLATE - 20, PRINTER_BUILD_PLATE - 20, facecolor="none", edgecolor="#94A3B8", linestyle="--", linewidth=0.9))

        for name, p in items:
            bb = p.bounding_box()
            kind = next(k for k in color_map if name.startswith(k))
            ax.add_patch(
                plt.Rectangle(
                    (bb.min.X, bb.min.Y),
                    bb.size.X,
                    bb.size.Y,
                    facecolor=color_map[kind],
                    edgecolor="#1E293B",
                    alpha=0.72,
                    linewidth=1.1,
                )
            )
            ax.text(
                (bb.min.X + bb.max.X) / 2.0,
                (bb.min.Y + bb.max.Y) / 2.0,
                name.replace("_", "\n"),
                ha="center",
                va="center",
                fontsize=6.5,
                color="#0F172A",
                fontweight="bold",
            )

        ax.set_title(f"{plate_name}.3mf\n({len(items)} parts)", fontsize=10, fontweight="bold")
        ax.set_aspect("equal")
        ax.set_xlim(-6, PRINTER_BUILD_PLATE + 6)
        ax.set_ylim(-6, PRINTER_BUILD_PLATE + 6)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.grid(True, linestyle=":", alpha=0.4)

    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main Entrypoint
# ---------------------------------------------------------------------------


def main() -> None:
    EXPORTS_DIR.mkdir(parents=True, exist_ok=True)
    RENDERS_DIR.mkdir(parents=True, exist_ok=True)

    parts: dict[str, Part] = {
        "base_foot": build_base_foot(),
        "tier_module": build_tier_module(),
        "top_cap": build_top_cap(),
        "crossbar": build_crossbar(),
    }

    stl_paths: dict[str, Path] = {}
    print("=== Exporting & Verifying Parts ===")
    for name, part in parts.items():
        stl_path = EXPORTS_DIR / f"{name}.stl"
        step_path = EXPORTS_DIR / f"{name}.step"
        export_stl(part, str(stl_path))
        export_step(part, str(step_path))
        stl_paths[name] = stl_path

        info = verify_stl(stl_path)
        print(
            f"  [OK] {name:12s} | bbox={info['extents_mm']} mm | "
            f"vol={info['volume_cm3']:.2f} cm^3 | watertight={info['watertight']} | "
            f"z0={info['min_z_mm']} mm"
        )

    total_h = BASE_HEIGHT + DEFAULT_TIERS * TIER_PITCH + TOP_CAP_HEIGHT
    top_penlight_h = (
        BASE_HEIGHT
        + (DEFAULT_TIERS - 1) * TIER_PITCH
        + (CRADLE_CENTER_Y - CRADLE_INNER_R)
        + penlight.COLLAR_RING_DIAMETER
    )
    print(f"\n=== {DEFAULT_TIERS}-Tier Assembly Summary ===")
    print(f"  Rack outer width : {INNER_SPAN + 2 * TOWER_THICKNESS:.1f} mm (penlight: {PENLIGHT_LENGTH:.1f} mm)")
    print(f"  Base foot depth  : {BASE_REAR_EXT + SPINE_WIDTH + BASE_FRONT_EXT:.1f} mm")
    print(f"  Total rack height: {total_h:.1f} mm (limit: {MAX_TOTAL_HEIGHT:.1f} mm)")
    print(f"  Top collar peak  : {top_penlight_h:.1f} mm (clearance to 330 mm: {MAX_TOTAL_HEIGHT - top_penlight_h:.1f} mm)")
    assert total_h <= MAX_TOTAL_HEIGHT, f"Total height {total_h} exceeds {MAX_TOTAL_HEIGHT} mm!"
    assert top_penlight_h <= MAX_TOTAL_HEIGHT, f"Top penlight {top_penlight_h} exceeds {MAX_TOTAL_HEIGHT} mm!"

    plates = export_print_plates(parts, EXPORTS_DIR)

    print("\n=== Generating Renders & Interactive Viewer ===")
    render_parts_overview(stl_paths, RENDERS_DIR / "parts_overview.png")
    render_assembly_views(stl_paths, RENDERS_DIR / "assembly_views.png", n_tiers=DEFAULT_TIERS)
    render_side_profile_detail(stl_paths, RENDERS_DIR / "detail_side_profile.png")
    render_fit_verification(stl_paths, RENDERS_DIR / "fit_verification.png")
    render_print_plates(plates, RENDERS_DIR / "print_plates.png")
    export_interactive_assembly(
        stl_paths,
        RENDERS_DIR / "assembly_preview.glb",
        RENDERS_DIR / "assembly_viewer.html",
        n_tiers=DEFAULT_TIERS,
    )
    print("  Saved renders & interactive viewer to:", RENDERS_DIR)


if __name__ == "__main__":
    main()
