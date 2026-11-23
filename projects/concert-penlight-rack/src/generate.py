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

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import penlight
import trimesh
from build123d import (
    Axis,
    BuildPart,
    BuildSketch,
    Circle,
    Locations,
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
)

from cadlib import (
    CameraView,
    LimitBox,
    ViewerItem,
    add_mesh_to_ax,
    center_layout,
    export_3mf_plates,
    export_interactive_viewer,
    place_on_plate,
    render_print_plates,
    set_equal_3d_axes,
    verify_stl,
)

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

# Output directories
PROJECT_DIR = Path(__file__).resolve().parent.parent
EXPORTS_DIR = PROJECT_DIR / "exports"
RENDERS_DIR = PROJECT_DIR / "renders"

PART_COLORS = {
    "base_foot": "#4A90D9",
    "tier_module": "#50B878",
    "top_cap": "#E6A23C",
    "crossbar": "#9B72CF",
}


# ---------------------------------------------------------------------------
# Geometry Builders (all parts oriented in print orientation, bottom at Z = 0)
# ---------------------------------------------------------------------------


def _dovetail_male_pts(base_y: float) -> list[tuple[float, float]]:
    cx = SPINE_WIDTH / 2.0
    nw = DOVETAIL_NECK_W / 2.0
    tw = DOVETAIL_TOP_W / 2.0
    return [
        (cx - nw, base_y - 0.5),
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
            Polygon(
                (x_min, 0.0),
                (x_max, 0.0),
                (x_max, BASE_TOE_HEIGHT),
                (SPINE_WIDTH + 6.0, BASE_HEIGHT),
                (0.0, BASE_HEIGHT),
                (x_min, BASE_TOE_HEIGHT),
                align=None,
            )
            Polygon(*_dovetail_male_pts(BASE_HEIGHT), align=None)
            with Locations((SPINE_WIDTH / 2.0, BASE_MORTISE_Y)):
                Rectangle(MORTISE_SIZE, MORTISE_SIZE, mode=Mode.SUBTRACT)

        extrude(amount=TOWER_THICKNESS)
        z_edges = bp.edges().filter_by(
            lambda e: abs(e.center().Z) < 0.1 or abs(e.center().Z - TOWER_THICKNESS) < 0.1
        )
        chamfer(z_edges, length=0.4)

    return bp.part


def build_tier_module() -> Part:
    """Build one stackable tier module in print orientation (flat on side, Z = 0..18)."""
    front_rad = math.radians(CRADLE_FRONT_ANGLE_DEG)
    back_rad = math.radians(CRADLE_BACK_ANGLE_DEG)
    arm_bottom_y = CRADLE_CENTER_Y - CRADLE_OUTER_R

    with BuildPart() as bp:
        with BuildSketch(Plane.XY):
            Polygon(
                (0.0, 0.0),
                (SPINE_WIDTH, 0.0),
                (SPINE_WIDTH, TIER_PITCH),
                (0.0, TIER_PITCH),
                align=None,
            )
            Polygon(*_dovetail_male_pts(TIER_PITCH), align=None)
            Polygon(
                (SPINE_WIDTH - 2.0, arm_bottom_y + 2.0),
                (CRADLE_CENTER_X, arm_bottom_y),
                (CRADLE_CENTER_X, CRADLE_CENTER_Y),
                (SPINE_WIDTH - 2.0, CRADLE_CENTER_Y + 10.0),
                align=None,
            )
            with Locations((CRADLE_CENTER_X, CRADLE_CENTER_Y)):
                Circle(CRADLE_OUTER_R)

            cut_r = CRADLE_OUTER_R + 20.0
            p_center = (CRADLE_CENTER_X, CRADLE_CENTER_Y)
            p_front = (
                CRADLE_CENTER_X + cut_r * math.cos(front_rad),
                CRADLE_CENTER_Y + cut_r * math.sin(front_rad),
            )
            p_top_right = (CRADLE_CENTER_X + cut_r, CRADLE_CENTER_Y + cut_r)
            p_top_left = (SPINE_WIDTH, CRADLE_CENTER_Y + cut_r)
            p_back_x = max(SPINE_WIDTH, CRADLE_CENTER_X + cut_r * math.cos(back_rad))
            p_back_clamped = (
                p_back_x,
                CRADLE_CENTER_Y + (CRADLE_CENTER_X - p_back_x) * math.tan(math.pi - back_rad),
            )
            Polygon(
                p_center, p_front, p_top_right, p_top_left, p_back_clamped,
                mode=Mode.SUBTRACT, align=None,
            )

            with Locations((CRADLE_CENTER_X, CRADLE_CENTER_Y)):
                Circle(CRADLE_INNER_R, mode=Mode.SUBTRACT)
            Polygon(*_dovetail_female_pts(0.0), mode=Mode.SUBTRACT, align=None)
            with Locations((SPINE_WIDTH / 2.0, TIER_MORTISE_Y)):
                Rectangle(MORTISE_SIZE, MORTISE_SIZE, mode=Mode.SUBTRACT)

        extrude(amount=TOWER_THICKNESS)

        front_tip_x = CRADLE_CENTER_X + (CRADLE_INNER_R + CRADLE_WALL / 2.0) * math.cos(front_rad)
        front_tip_y = CRADLE_CENTER_Y + (CRADLE_INNER_R + CRADLE_WALL / 2.0) * math.sin(front_rad)
        tip_edges = bp.edges().filter_by(Axis.Z).filter_by(
            lambda e: math.hypot(e.center().X - front_tip_x, e.center().Y - front_tip_y)
            < CRADLE_WALL + 1.5
        )
        if tip_edges:
            fillet(tip_edges, radius=1.8)

        under_arm_edges = bp.edges().filter_by(Axis.Z).filter_by(
            lambda e: abs(e.center().X - SPINE_WIDTH) < 1.0
            and abs(e.center().Y - (arm_bottom_y + 2.0)) < 2.0
        )
        if under_arm_edges:
            fillet(under_arm_edges, radius=4.0)

        z_edges = bp.edges().filter_by(
            lambda e: abs(e.center().Z) < 0.1 or abs(e.center().Z - TOWER_THICKNESS) < 0.1
        )
        chamfer(z_edges, length=0.4)

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
        z_edges = bp.edges().filter_by(
            lambda e: abs(e.center().Z) < 0.1 or abs(e.center().Z - TOWER_THICKNESS) < 0.1
        )
        chamfer(z_edges, length=0.4)

    return bp.part


def build_crossbar() -> Part:
    """Build the horizontal crossbar in print orientation (flat on Z = 0, 0 overhangs)."""
    total_len = INNER_SPAN + 2.0 * TENON_LENGTH

    with BuildPart() as bp:
        with BuildSketch(Plane.XY):
            Rectangle(INNER_SPAN, CROSSBAR_BEAM_W)
            Rectangle(total_len, TENON_SIZE)

        extrude(amount=CROSSBAR_BEAM_H)

        end_edges = bp.edges().filter_by(
            lambda e: abs(abs(e.center().X) - total_len / 2.0) < 0.1
        )
        chamfer(end_edges, length=0.8)

        z_edges = bp.edges().filter_by(
            lambda e: (abs(e.center().Z) < 0.1 or abs(e.center().Z - CROSSBAR_BEAM_H) < 0.1)
            and abs(e.center().X) < INNER_SPAN / 2.0 - 0.1
        )
        chamfer(z_edges, length=0.4)

    return bp.part


# ---------------------------------------------------------------------------
# Assembly, Print Plates & Renders
# ---------------------------------------------------------------------------


def _stand_upright_transform(tower_x_offset: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    T[0, 3] = tower_x_offset
    return T


def build_assembled_items(
    stl_paths: dict[str, Path], n_tiers: int = DEFAULT_TIERS
) -> list[ViewerItem]:
    """Build all positioned ViewerItems for an N-tier assembly with sculpted concert penlights."""
    base_m = trimesh.load_mesh(stl_paths["base_foot"], force="mesh")
    tier_m = trimesh.load_mesh(stl_paths["tier_module"], force="mesh")
    cap_m = trimesh.load_mesh(stl_paths["top_cap"], force="mesh")
    bar_m = trimesh.load_mesh(stl_paths["crossbar"], force="mesh")

    items: list[ViewerItem] = []
    left_x = -INNER_SPAN / 2.0 - TOWER_THICKNESS
    right_x = INNER_SPAN / 2.0

    for x_off in (left_x, right_x):
        m = base_m.copy()
        m.apply_transform(_stand_upright_transform(x_off))
        items.append(ViewerItem(m, "#3B4A5A", metallic=0.08, roughness=0.62, group="rack"))

        for t in range(n_tiers):
            T_tier = _stand_upright_transform(x_off)
            T_tier[2, 3] = BASE_HEIGHT + t * TIER_PITCH
            mt = tier_m.copy()
            mt.apply_transform(T_tier)
            col = "#4F6275" if t % 2 == 0 else "#5E7388"
            items.append(ViewerItem(mt, col, metallic=0.08, roughness=0.62, group="rack"))

        T_cap = _stand_upright_transform(x_off)
        T_cap[2, 3] = BASE_HEIGHT + n_tiers * TIER_PITCH
        mc = cap_m.copy()
        mc.apply_transform(T_cap)
        items.append(ViewerItem(mc, "#3B4A5A", metallic=0.08, roughness=0.62, group="rack"))

    for bz in (BASE_MORTISE_Y, BASE_HEIGHT + (n_tiers - 1) * TIER_PITCH + TIER_MORTISE_Y):
        mb = bar_m.copy()
        mb.apply_translation([0.0, SPINE_WIDTH / 2.0, bz - CROSSBAR_BEAM_H / 2.0])
        items.append(ViewerItem(mb, "#2D3744", metallic=0.08, roughness=0.62, group="rack"))

    tube_colors = ["#FF4D8D", "#38BDF8", "#FACC15", "#4ADE80", "#C084FC", "#FB923C"]
    handle_colors = ["#F4A6C1", "#BAE6FD", "#FEF08A", "#BBF7D0", "#E9D5FF", "#FED7AA"]
    mat_props = {
        "handle": (0.05, 0.35),
        "collar": (0.88, 0.18),
        "tube": (0.05, 0.25),
    }
    for t in range(n_tiers):
        cz = BASE_HEIGHT + t * TIER_PITCH + (CRADLE_CENTER_Y - CRADLE_INNER_R) + PENLIGHT_DIAMETER / 2.0
        pm_list = penlight.build_penlight_meshes(
            tube_color=tube_colors[t % len(tube_colors)],
            handle_color=handle_colors[t % len(handle_colors)],
        )
        for (pm, pcol, palpha), kind in zip(pm_list, ("handle", "collar", "tube")):
            pm.apply_translation([0.0, CRADLE_CENTER_X, cz])
            metallic, roughness = mat_props[kind]
            items.append(
                ViewerItem(pm, pcol, alpha=palpha, metallic=metallic, roughness=roughness, group="penlight")
            )

    return items


def build_print_plates(parts: dict[str, Part]) -> dict[str, list[tuple[str, Part]]]:
    """Arrange ready-to-slice 256x256 mm build plates."""
    gap = 8.0
    bf, tm, tc, cb = parts["base_foot"], parts["tier_module"], parts["top_cap"], parts["crossbar"]
    bf_w, bf_h = bf.bounding_box().size.X, bf.bounding_box().size.Y
    tm_w, tm_h = tm.bounding_box().size.X, tm.bounding_box().size.Y
    tc_h = tc.bounding_box().size.Y
    cb_w, cb_h = cb.bounding_box().size.X, cb.bounding_box().size.Y

    # Plate 1: Complete 3-tier rack
    p1: list[tuple[str, Part]] = []
    y = 0.0
    for i in (1, 2):
        p1.append((f"crossbar_{i}", place_on_plate(cb, 0.0, y, f"crossbar_{i}")))
        p1.append((f"top_cap_{i}", place_on_plate(tc, cb_w + gap, y, f"top_cap_{i}")))
        y += max(cb_h, tc_h) + gap
    p1.append(("base_foot_1", place_on_plate(bf, 0.0, y, "base_foot_1")))
    p1.append(("base_foot_2", place_on_plate(bf, bf_w + gap, y, "base_foot_2")))
    y += bf_h + gap
    for row in range(2):
        for col in range(3):
            idx = row * 3 + col + 1
            p1.append((f"tier_module_{idx}", place_on_plate(tm, col * (tm_w + gap), y, f"tier_module_{idx}")))
        y += tm_h + gap

    # Plate 2: +3 tier expansion (to 6 tiers) + optional middle crossbar
    p2: list[tuple[str, Part]] = [("crossbar_mid", place_on_plate(cb, 0.0, 0.0, "crossbar_mid"))]
    y = cb_h + gap
    for row in range(2):
        for col in range(3):
            idx = row * 3 + col + 7
            p2.append((f"tier_module_{idx}", place_on_plate(tm, col * (tm_w + gap), y, f"tier_module_{idx}")))
        y += tm_h + gap

    # Test-fit plate (1 tier)
    pt: list[tuple[str, Part]] = [
        ("crossbar_test", place_on_plate(cb, 0.0, 0.0, "crossbar_test")),
        ("base_foot_1", place_on_plate(bf, 0.0, cb_h + gap, "base_foot_1")),
        ("base_foot_2", place_on_plate(bf, bf_w + gap, cb_h + gap, "base_foot_2")),
        ("tier_module_1", place_on_plate(tm, 0.0, cb_h + bf_h + 2 * gap, "tier_module_1")),
        ("tier_module_2", place_on_plate(tm, tm_w + gap, cb_h + bf_h + 2 * gap, "tier_module_2")),
    ]

    return {
        "plate1_base_and_3tiers": center_layout(p1, PRINTER_BUILD_PLATE),
        "plate2_expansion_3tiers": center_layout(p2, PRINTER_BUILD_PLATE),
        "plate_test_fit_1tier": center_layout(pt, PRINTER_BUILD_PLATE),
    }


def render_parts_overview(stl_paths: dict[str, Path], out_path: Path) -> None:
    fig = plt.figure(figsize=(14, 10), dpi=150)
    fig.suptitle(
        "Concert Penlight Rack — Individual Parts (Print Orientation, Z = 0)",
        fontsize=14, fontweight="bold",
    )
    for idx, (name, path) in enumerate(stl_paths.items(), start=1):
        ax = fig.add_subplot(2, 2, idx, projection="3d")
        mesh = trimesh.load_mesh(path, force="mesh")
        add_mesh_to_ax(ax, mesh, color=PART_COLORS[name])
        set_equal_3d_axes(ax, mesh.bounds[0], mesh.bounds[1], pad=6.0)
        ext = mesh.extents
        ax.set_title(f"{name}.stl\n{ext[0]:.1f} × {ext[1]:.1f} × {ext[2]:.1f} mm", fontsize=11)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.view_init(elev=28, azim=-50)
    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def render_assembly_views(items: list[ViewerItem], out_path: Path, n_tiers: int = DEFAULT_TIERS) -> None:
    all_min = np.min([it.mesh.bounds[0] for it in items], axis=0)
    all_max = np.max([it.mesh.bounds[1] for it in items], axis=0)
    total_height = BASE_HEIGHT + n_tiers * TIER_PITCH + TOP_CAP_HEIGHT
    top_penlight_z = (
        BASE_HEIGHT + (n_tiers - 1) * TIER_PITCH + (CRADLE_CENTER_Y - CRADLE_INNER_R) + PENLIGHT_DIAMETER
    )

    fig = plt.figure(figsize=(15, 11), dpi=150)
    fig.suptitle(
        f"Concert Penlight Rack — {n_tiers}-Tier Assembly  "
        f"(Rack Height: {total_height:.1f} mm | Top Penlight: {top_penlight_z:.1f} mm | Limit: {MAX_TOTAL_HEIGHT:.0f} mm)",
        fontsize=13, fontweight="bold",
    )
    views = [
        ("Isometric View (Front)", 22, 52),
        ("Front View", 5, 90),
        ("Side Profile (Right)", 3, 0),
        ("Top-Down View", 82, 90),
    ]
    for idx, (title, elev, azim) in enumerate(views, start=1):
        ax = fig.add_subplot(2, 2, idx, projection="3d")
        for it in items:
            add_mesh_to_ax(ax, it.mesh, color=it.color, alpha=it.alpha, linewidth=0.04)
        if elev < 60:
            x0, x1, y0, y1 = -130, 130, -15, 95
            ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], [MAX_TOTAL_HEIGHT] * 5, "r--", linewidth=1.2)
        set_equal_3d_axes(ax, all_min, np.maximum(all_max, [0, 0, MAX_TOTAL_HEIGHT]), pad=12.0)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.view_init(elev=elev, azim=azim)
    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def render_side_profile_detail(stl_paths: dict[str, Path], out_path: Path) -> None:
    base_m = trimesh.load_mesh(stl_paths["base_foot"], force="mesh")
    tier_m = trimesh.load_mesh(stl_paths["tier_module"], force="mesh")

    fig, ax = plt.subplots(figsize=(8, 9), dpi=150)
    ax.set_title("Side Profile Detail — Base + 2 Tiers & Penlight Clearances", fontsize=12, fontweight="bold")

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

    for t in range(2):
        cy = BASE_HEIGHT + t * TIER_PITCH + (CRADLE_CENTER_Y - CRADLE_INNER_R) + PENLIGHT_DIAMETER / 2.0
        ax.add_patch(
            plt.Circle(
                (CRADLE_CENTER_X, cy), PENLIGHT_DIAMETER / 2.0,
                facecolor="#FF6B9D", edgecolor="#B0003A", alpha=0.45, linewidth=1.5,
                label="Ø30 mm Penlight" if t == 0 else None,
            )
        )

    p1_top = BASE_HEIGHT + (CRADLE_CENTER_Y - CRADLE_INNER_R) + PENLIGHT_DIAMETER
    t2_bot = BASE_HEIGHT + TIER_PITCH + (CRADLE_CENTER_Y - CRADLE_OUTER_R)
    ax.annotate(
        f"Lift clearance: {t2_bot - p1_top:.1f} mm",
        xy=(CRADLE_CENTER_X + 20, (p1_top + t2_bot) / 2.0),
        xytext=(CRADLE_CENTER_X + 30, (p1_top + t2_bot) / 2.0),
        fontsize=9, color="#B0003A", arrowprops={"arrowstyle": "->", "color": "#B0003A"},
    )
    ax.axhline(0, color="#333333", linewidth=1.0)
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
    tier_m = trimesh.load_mesh(stl_paths["tier_module"], force="mesh")
    stations = penlight.get_profile_stations()
    half_len = penlight.TOTAL_LENGTH / 2.0
    xs = np.array([s.z - half_len for s in stations])
    rs = np.array([s.radius for s in stations])

    fig = plt.figure(figsize=(14, 9), dpi=150)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.15])
    fig.suptitle(
        "Concert Penlight Fit Verification (Measured Sculpted Handle vs. Ø36.0 mm Cradles)",
        fontsize=13, fontweight="bold",
    )

    ax_long = fig.add_subplot(gs[0, :])
    ax_long.fill_between(xs, -rs, rs, color="#F4A6C1", alpha=0.35)
    ax_long.plot(xs, rs, color="#B0003A", linewidth=1.8, label="Modeled Concert Penlight")
    ax_long.plot(xs, -rs, color="#B0003A", linewidth=1.8)

    c0 = penlight.HANDLE_LENGTH - half_len
    c1 = penlight.HANDLE_LENGTH + penlight.COLLAR_LENGTH - half_len
    ax_long.axvspan(c0, c1, color="#9CA3AF", alpha=0.35, label="Chrome Collar (Ø34.5–35.0 mm)")

    for x0, x1, lbl in [
        (-INNER_SPAN / 2.0 - TOWER_THICKNESS, -INNER_SPAN / 2.0, "Handle Cradle (18 mm wide)"),
        (INNER_SPAN / 2.0, INNER_SPAN / 2.0 + TOWER_THICKNESS, "Tube Cradle (18 mm wide)"),
    ]:
        ax_long.axvspan(x0, x1, color="#2563EB", alpha=0.20, label=lbl)
        ax_long.plot([x0, x1], [-CRADLE_INNER_R, -CRADLE_INNER_R], color="#1D4ED8", linewidth=3.0)

    for text, xy, xytext in [
        (f"Palm swell\nØ{penlight.HANDLE_SWELL_DIAMETER:.1f} mm", (24.0 - half_len, penlight.HANDLE_SWELL_DIAMETER / 2.0), (24.0 - half_len - 18, 24.0)),
        (f"Waist\nØ{penlight.HANDLE_WAIST_DIAMETER:.1f} mm", (58.0 - half_len, penlight.HANDLE_WAIST_DIAMETER / 2.0), (58.0 - half_len - 10, 24.0)),
        (f"Collar\nØ{penlight.COLLAR_RING_DIAMETER:.1f} mm", ((c0 + c1) / 2.0, penlight.COLLAR_RING_DIAMETER / 2.0), ((c0 + c1) / 2.0 - 8, 25.0)),
        (f"Light tube\nØ{penlight.TUBE_DIAMETER:.1f} mm", (45.0, penlight.TUBE_DIAMETER / 2.0), (38.0, 24.0)),
    ]:
        ax_long.annotate(text, xy=xy, xytext=xytext, fontsize=9, arrowprops={"arrowstyle": "->"})

    ax_long.set_aspect("equal")
    ax_long.set_xlim(-half_len - 12, half_len + 12)
    ax_long.set_ylim(-28, 36)
    ax_long.set_xlabel("Axial Position X (mm, 0 = Rack Center)")
    ax_long.set_ylabel("Radial Profile (mm)")
    ax_long.grid(True, linestyle=":", alpha=0.5)
    ax_long.legend(loc="lower right", ncol=2, fontsize=8.5)

    sec = tier_m.section(plane_origin=[0, 0, TOWER_THICKNESS / 2.0], plane_normal=[0, 0, 1])

    def plot_cradle_check(ax, title: str, circles: list[tuple[float, str, str, str]]):
        if sec is not None:
            for ent in sec.entities:
                pts = sec.vertices[ent.points]
                ax.plot(pts[:, 0], pts[:, 1], color="#1E3A5F", linewidth=2.0)
        cradle_bot = CRADLE_CENTER_Y - CRADLE_INNER_R
        for diam, fill, edge, lbl in circles:
            r = diam / 2.0
            ax.add_patch(
                plt.Circle(
                    (CRADLE_CENTER_X, cradle_bot + r), r,
                    facecolor=fill, edgecolor=edge, alpha=0.35, linewidth=1.8, label=lbl,
                )
            )
        ax.set_title(title, fontsize=10.5, fontweight="bold")
        ax.set_aspect("equal")
        ax.set_xlim(18, 72)
        ax.set_ylim(2, 54)
        ax.set_xlabel("Depth X (mm)")
        ax.set_ylabel("Tier Height Y (mm)")
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.legend(loc="upper left", fontsize=8.5)

    plot_cradle_check(
        fig.add_subplot(gs[1, 0]),
        "Handle Cradle Fit (Ø36.0 mm ID)",
        [
            (penlight.HANDLE_SWELL_DIAMETER, "#EC4899", "#9D174D", f"Max Palm Swell (Ø{penlight.HANDLE_SWELL_DIAMETER:.1f} mm)"),
            (penlight.HANDLE_WAIST_DIAMETER, "#F472B6", "#BE185D", f"Handle Waist (Ø{penlight.HANDLE_WAIST_DIAMETER:.1f} mm)"),
        ],
    )
    plot_cradle_check(
        fig.add_subplot(gs[1, 1]),
        "Tube & Shifted Collar Fit (Ø36.0 mm ID)",
        [
            (penlight.COLLAR_RING_DIAMETER, "#9CA3AF", "#374151", f"Chrome Collar (Ø{penlight.COLLAR_RING_DIAMETER:.1f} mm)"),
            (penlight.TUBE_DIAMETER, "#38BDF8", "#0369A1", f"Light Tube (Ø{penlight.TUBE_DIAMETER:.1f} mm)"),
        ],
    )
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
        export_stl(part, str(stl_path))
        export_step(part, str(EXPORTS_DIR / f"{name}.step"))
        stl_paths[name] = stl_path
        verify_stl(stl_path, build_plate_mm=PRINTER_BUILD_PLATE)

    total_h = BASE_HEIGHT + DEFAULT_TIERS * TIER_PITCH + TOP_CAP_HEIGHT
    top_collar_h = (
        BASE_HEIGHT + (DEFAULT_TIERS - 1) * TIER_PITCH + (CRADLE_CENTER_Y - CRADLE_INNER_R) + penlight.COLLAR_RING_DIAMETER
    )
    print(f"\n=== {DEFAULT_TIERS}-Tier Assembly Summary ===")
    print(f"  Rack outer width : {INNER_SPAN + 2 * TOWER_THICKNESS:.1f} mm (penlight: {PENLIGHT_LENGTH:.1f} mm)")
    print(f"  Base foot depth  : {BASE_REAR_EXT + SPINE_WIDTH + BASE_FRONT_EXT:.1f} mm")
    print(f"  Total rack height: {total_h:.1f} mm (limit: {MAX_TOTAL_HEIGHT:.1f} mm)")
    print(f"  Top collar peak  : {top_collar_h:.1f} mm (clearance to 330 mm: {MAX_TOTAL_HEIGHT - top_collar_h:.1f} mm)")
    assert total_h <= MAX_TOTAL_HEIGHT
    assert top_collar_h <= MAX_TOTAL_HEIGHT

    plates = build_print_plates(parts)
    export_3mf_plates(plates, EXPORTS_DIR, build_plate_mm=PRINTER_BUILD_PLATE)

    print("\n=== Generating Renders & Interactive Viewer ===")
    items = build_assembled_items(stl_paths, n_tiers=DEFAULT_TIERS)
    render_parts_overview(stl_paths, RENDERS_DIR / "parts_overview.png")
    render_assembly_views(items, RENDERS_DIR / "assembly_views.png", n_tiers=DEFAULT_TIERS)
    render_side_profile_detail(stl_paths, RENDERS_DIR / "detail_side_profile.png")
    render_fit_verification(stl_paths, RENDERS_DIR / "fit_verification.png")
    render_print_plates(plates, RENDERS_DIR / "print_plates.png", color_map=PART_COLORS, build_plate_mm=PRINTER_BUILD_PLATE)
    export_interactive_viewer(
        items,
        RENDERS_DIR / "assembly_preview.glb",
        RENDERS_DIR / "assembly_viewer.html",
        title="Concert Penlight Rack — Interactive 3D Assembly",
        heading=f"Concert Penlight Rack ({DEFAULT_TIERS}-Tier)",
        subtitle_html=f"Height: {total_h:.1f} mm / {MAX_TOTAL_HEIGHT:.0f} mm max &middot; Width: {INNER_SPAN + 2 * TOWER_THICKNESS:.0f} mm",
        target=(0.0, 160.0, -35.0),
        views=[
            CameraView("iso", "Iso", (290, 270, -340)),
            CameraView("front", "Front", (0, 165, -480)),
            CameraView("side", "Side", (460, 165, -35)),
            CameraView("top", "Top", (0, 620, -36)),
            CameraView("back", "Back", (0, 165, 420)),
        ],
        default_view="iso",
        grid_size=500.0,
        grid_divisions=50,
        toggle_groups={"penlight": "Show penlights"},
        limit_box=LimitBox("Show 330 mm height limit", (280, 1, 130), (0, MAX_TOTAL_HEIGHT, -35)),
    )
    print("  Saved renders & interactive viewer to:", RENDERS_DIR)


if __name__ == "__main__":
    main()
