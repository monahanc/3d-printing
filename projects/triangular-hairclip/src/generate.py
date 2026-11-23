"""Parametric CAD generator for the Triangular Hairclip (Bobby-Pin & Clip Variants).

Generates print-ready STL files, multi-angle inspection renders, and an
interactive Three.js 3D viewer for a smooth, curved triangular domed hairclip
(19.8 × 25.0 × 5.8 mm).

Variants exported to exports/:
- triangular_hairclip_bobbypin.stl : Pass-through underside groove & flush retaining
  bridge for a standard 50 mm (2-inch) bobby pin (prints flat, zero supports).
- triangular_hairclip_recessed.stl : Open-top underside channel (6.4 × 20.0 × 0.85 mm)
  for gluing an alligator hair clip.
- triangular_hairclip_flat.stl     : Solid flat back for general glue-on mounting.

Run from the repository root:
    uv run python projects/triangular-hairclip/src/generate.py
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
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------------------------------------------------------------------
# Parametric Dimensions (all units in millimeters)
# ---------------------------------------------------------------------------

# Hairclip outer body
CLIP_LENGTH = 25.0
CLIP_WIDTH = 19.8
CLIP_HEIGHT = 5.8
BOUNDARY_R_OFFSET = 0.28
BOUNDARY_BULGE = 0.13

# Alligator-clip open-top recess variant
RECESS_HALF_WIDTH = 3.2  # 6.4 mm total channel width
RECESS_Y_MIN = -7.5  # Leaves solid bottom tip
RECESS_DEPTH = 0.85

# Bobby-pin pass-through tunnel & flush retaining bridge variant
BOBBY_GROOVE_WIDTH = 3.2  # Tapers slightly near narrow tip to preserve wall thickness
BOBBY_BRIDGE_HEIGHT = 1.05  # Flush retaining bridge printed flat on Z = 0
BOBBY_TUNNEL_CEILING = 3.15
BOBBY_ROOF_PITCH = 0.40  # Self-supporting pitched roof (+0.40 mm at center)

# Build plate for 3MF export
PRINTER_BUILD_PLATE = 256.0

# Output directories
PROJECT_DIR = Path(__file__).resolve().parent.parent
EXPORTS_DIR = PROJECT_DIR / "exports"
RENDERS_DIR = PROJECT_DIR / "renders"


# ---------------------------------------------------------------------------
# Geometry Builders
# ---------------------------------------------------------------------------


def generate_curved_triangle_boundary(
    length: float = CLIP_LENGTH,
    width: float = CLIP_WIDTH,
    r_offset: float = BOUNDARY_R_OFFSET,
    bulge: float = BOUNDARY_BULGE,
) -> np.ndarray:
    """Generate a smooth, uniformly sampled curved triangular 2D perimeter."""
    w0 = 0.92
    y_top = 0.75
    y_bot = -1.25
    verts = np.array(
        [
            [0.0, y_bot],  # Bottom tip
            [w0, y_top],  # Top-right
            [-w0, y_top],  # Top-left
        ]
    )

    boundary_pts = []
    for i in range(3):
        p0 = verts[i]
        p1 = verts[(i + 1) % 3]
        p_prev = verts[(i - 1) % 3]

        mid_prev = 0.5 * (p_prev + p0)
        chord_prev = p0 - p_prev
        len_prev = np.linalg.norm(chord_prev)
        norm_prev = np.array([chord_prev[1], -chord_prev[0]]) / len_prev
        ctrl_prev = mid_prev + norm_prev * (bulge * len_prev)
        tan_in = p0 - ctrl_prev
        n_in = np.array([tan_in[1], -tan_in[0]]) / np.linalg.norm(tan_in)

        mid_curr = 0.5 * (p0 + p1)
        chord_curr = p1 - p0
        len_curr = np.linalg.norm(chord_curr)
        norm_curr = np.array([chord_curr[1], -chord_curr[0]]) / len_curr
        ctrl_curr = mid_curr + norm_curr * (bulge * len_curr)
        tan_out = ctrl_curr - p0
        n_out = np.array([tan_out[1], -tan_out[0]]) / np.linalg.norm(tan_out)

        ang_in = np.arctan2(n_in[1], n_in[0])
        ang_out = np.arctan2(n_out[1], n_out[0])
        if ang_out < ang_in:
            ang_out += 2 * np.pi
        angs = np.linspace(ang_in, ang_out, 90, endpoint=False)
        arc = p0 + r_offset * np.column_stack([np.cos(angs), np.sin(angs)])
        boundary_pts.append(arc)

        t = np.linspace(0, 1, 180, endpoint=False)[:, None]
        pts = (1 - t) ** 2 * p0 + 2 * (1 - t) * t * ctrl_curr + t**2 * p1
        dpts = 2 * (1 - t) * (ctrl_curr - p0) + 2 * t * (p1 - ctrl_curr)
        norms = np.column_stack([dpts[:, 1], -dpts[:, 0]]) / np.linalg.norm(
            dpts, axis=1, keepdims=True
        )
        edge_offset = pts + r_offset * norms
        boundary_pts.append(edge_offset)

    b = np.vstack(boundary_pts)
    diffs = np.diff(np.vstack([b, b[0]]), axis=0)
    seg_lens = np.linalg.norm(diffs, axis=1)
    cum_len = np.concatenate([[0], np.cumsum(seg_lens)])
    total_len = cum_len[-1]
    s_uniform = np.linspace(0, total_len, 360, endpoint=False)
    bx = np.interp(s_uniform, cum_len, np.append(b[:, 0], b[0, 0]))
    by = np.interp(s_uniform, cum_len, np.append(b[:, 1], b[0, 1]))
    b = np.column_stack([bx, by])

    b[:, 0] -= 0.5 * (b[:, 0].min() + b[:, 0].max())
    b[:, 1] -= 0.5 * (b[:, 1].min() + b[:, 1].max())

    cur_w = b[:, 0].max() - b[:, 0].min()
    cur_l = b[:, 1].max() - b[:, 1].min()
    b[:, 0] *= width / cur_w
    b[:, 1] *= length / cur_l
    return b


def recess_depth_fn(x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray | float:
    """Open-top alligator clip channel (6.4 mm wide × 20.0 mm long × 0.85 mm deep)."""
    dx = np.abs(x) - (RECESS_HALF_WIDTH - 0.4)
    dy = RECESS_Y_MIN - y - 0.4
    d_slot = (
        np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
        + np.minimum(np.maximum(dx, dy), 0.0)
        - 0.4
    )
    return np.clip((0.5 - d_slot) * (RECESS_DEPTH / 0.5), 0.0, RECESS_DEPTH)


def build_hairclip_mesh(
    boundary: np.ndarray, height: float = CLIP_HEIGHT, with_recess: bool = False
) -> trimesh.Trimesh:
    """Build the domed hairclip mesh resting flat on Z = 0."""
    center = boundary.mean(axis=0)
    rel_b = boundary - center
    n_pts = len(boundary)

    n_rings = 60
    phi = np.linspace(0.0, 0.5 * np.pi, n_rings, endpoint=False)
    u_top = np.cos(phi)
    z_dome = height * np.power(np.sin(phi), 0.75)

    n_bot = 45
    u_bot = np.linspace(1.0, 0.0, n_bot + 1)[1:-1]

    vertices = []
    m_top = len(u_top)
    for u, zd in zip(u_top, z_dome):
        ring_xy = center + u * rel_b
        if with_recess:
            z_floor = recess_depth_fn(ring_xy[:, 0], ring_xy[:, 1])
            z_ring = np.maximum(zd, z_floor)
        else:
            z_ring = np.full(n_pts, zd)
        vertices.append(np.column_stack([ring_xy, z_ring]))

    m_bot = len(u_bot)
    for u in u_bot:
        ring_xy = center + u * rel_b
        if with_recess:
            z_vals = recess_depth_fn(ring_xy[:, 0], ring_xy[:, 1])
        else:
            z_vals = np.zeros(n_pts)
        vertices.append(np.column_stack([ring_xy, z_vals]))

    vertices_arr = np.vstack(vertices)

    idx_top_apex = len(vertices_arr)
    top_apex = np.array([[center[0], center[1], height]])

    idx_bot_apex = len(vertices_arr) + 1
    bot_center_z = float(recess_depth_fn(center[0], center[1])) if with_recess else 0.0
    bot_apex = np.array([[center[0], center[1], bot_center_z]])

    vertices_arr = np.vstack([vertices_arr, top_apex, bot_apex])

    faces = []
    for r in range(m_top - 1):
        base0 = r * n_pts
        base1 = (r + 1) * n_pts
        for i in range(n_pts):
            inext = (i + 1) % n_pts
            faces.append([base0 + i, base0 + inext, base1 + inext])
            faces.append([base0 + i, base1 + inext, base1 + i])

    base_last_top = (m_top - 1) * n_pts
    for i in range(n_pts):
        inext = (i + 1) % n_pts
        faces.append([base_last_top + i, base_last_top + inext, idx_top_apex])

    bot_rings = [0] + [m_top + b_idx for b_idx in range(m_bot)]
    for idx_pair in range(len(bot_rings) - 1):
        r0 = bot_rings[idx_pair]
        r1 = bot_rings[idx_pair + 1]
        base0 = r0 * n_pts
        base1 = r1 * n_pts
        for i in range(n_pts):
            inext = (i + 1) % n_pts
            faces.append([base0 + inext, base0 + i, base1 + inext])
            faces.append([base1 + inext, base0 + i, base1 + i])

    base_last_bot = bot_rings[-1] * n_pts
    for i in range(n_pts):
        inext = (i + 1) % n_pts
        faces.append([base_last_bot + inext, base_last_bot + i, idx_bot_apex])

    mesh = trimesh.Trimesh(
        vertices=vertices_arr, faces=np.array(faces, dtype=np.int64), process=True
    )
    trimesh.repair.fix_normals(mesh)
    return mesh


def make_bobbypin_cutter(width: float = BOBBY_GROOVE_WIDTH) -> trimesh.Trimesh:
    """Construct manifold boolean cutter for the pass-through bobby-pin groove & tunnel."""
    ny = 140
    nx = 21
    ys = np.linspace(-13.2, 13.2, ny)
    xs_unit = np.linspace(-1.0, 1.0, nx)

    def half_width_fn(y: np.ndarray) -> np.ndarray:
        hw = np.full_like(y, 0.5 * width)
        mask = y < -7.5
        t = np.clip((-7.5 - y[mask]) / 5.0, 0.0, 1.0)
        hw[mask] = 0.5 * width * (1.0 - 0.16 * t)
        return hw

    def z_bot_fn(y: np.ndarray) -> np.ndarray:
        d_bridge = np.maximum(np.abs(y - 0.25) - 3.05, 0.0)
        t = np.clip(d_bridge / 0.4, 0.0, 1.0)
        return BOBBY_BRIDGE_HEIGHT * (1.0 - t) + (-0.2) * t

    def z_top_fn(x_u: np.ndarray, y: np.ndarray) -> np.ndarray:
        z_c = np.full_like(y, BOBBY_TUNNEL_CEILING)
        mask_bot_ramp = (y >= -8.5) & (y < -5.5)
        z_c[mask_bot_ramp] = 1.10 + (BOBBY_TUNNEL_CEILING - 1.10) * (
            (y[mask_bot_ramp] - (-8.5)) / 3.0
        )
        z_c[y < -8.5] = 1.10

        mask_top_ramp = (y > 6.0) & (y <= 9.0)
        z_c[mask_top_ramp] = BOBBY_TUNNEL_CEILING - (BOBBY_TUNNEL_CEILING - 1.10) * (
            (y[mask_top_ramp] - 6.0) / 3.0
        )
        z_c[y > 9.0] = 1.10

        roof_pitch = BOBBY_ROOF_PITCH * (1.0 - np.abs(x_u))
        return z_c + roof_pitch

    uu, yy = np.meshgrid(xs_unit, ys, indexing="ij")
    xx = uu * half_width_fn(yy)
    zz_bot = z_bot_fn(yy)
    zz_top = z_top_fn(uu, yy)

    v_bot = np.column_stack([xx.ravel(), yy.ravel(), zz_bot.ravel()])
    v_top = np.column_stack([xx.ravel(), yy.ravel(), zz_top.ravel()])
    verts = np.vstack([v_bot, v_top])
    n_grid = nx * ny

    def idx_b(i: int, j: int) -> int:
        return i * ny + j

    def idx_t(i: int, j: int) -> int:
        return n_grid + i * ny + j

    faces = []
    for i in range(nx - 1):
        for j in range(ny - 1):
            faces.append([idx_b(i, j), idx_b(i, j + 1), idx_b(i + 1, j + 1)])
            faces.append([idx_b(i, j), idx_b(i + 1, j + 1), idx_b(i + 1, j)])
            faces.append([idx_t(i, j), idx_t(i + 1, j + 1), idx_t(i, j + 1)])
            faces.append([idx_t(i, j), idx_t(i + 1, j), idx_t(i + 1, j + 1)])
    for j in range(ny - 1):
        faces.append([idx_b(0, j), idx_t(0, j), idx_t(0, j + 1)])
        faces.append([idx_b(0, j), idx_t(0, j + 1), idx_b(0, j + 1)])
        faces.append([idx_b(nx - 1, j), idx_t(nx - 1, j + 1), idx_t(nx - 1, j)])
        faces.append([idx_b(nx - 1, j), idx_b(nx - 1, j + 1), idx_t(nx - 1, j + 1)])
    for i in range(nx - 1):
        faces.append([idx_b(i, 0), idx_b(i + 1, 0), idx_t(i + 1, 0)])
        faces.append([idx_b(i, 0), idx_t(i + 1, 0), idx_t(i, 0)])
        faces.append([idx_b(i, ny - 1), idx_t(i + 1, ny - 1), idx_b(i + 1, ny - 1)])
        faces.append([idx_b(i, ny - 1), idx_t(i, ny - 1), idx_t(i + 1, ny - 1)])

    cutter = trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=True)
    trimesh.repair.fix_normals(cutter)
    return cutter


def build_bobbypin_mesh(base_mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Subtract the bobby-pin pass-through tunnel & groove from the solid clip body."""
    cutter = make_bobbypin_cutter(width=BOBBY_GROOVE_WIDTH)
    mesh = trimesh.boolean.difference([base_mesh, cutter], engine="manifold")
    trimesh.repair.fix_normals(mesh)
    return mesh


# ---------------------------------------------------------------------------
# Verification, Rendering & Interactive 3D Viewer
# ---------------------------------------------------------------------------


def verify_mesh(name: str, mesh: trimesh.Trimesh) -> None:
    ext = mesh.extents
    min_z = float(mesh.bounds[0][2])
    assert mesh.is_watertight, f"{name} is NOT watertight!"
    assert mesh.volume > 0, f"{name} has non-positive volume!"
    assert abs(min_z) < 1e-2, f"{name} does not rest on Z = 0 (min_z={min_z})"
    print(
        f"  [OK] {name:28s} | bbox=({ext[0]:.2f}, {ext[1]:.2f}, {ext[2]:.2f}) mm | "
        f"vol={mesh.volume:.1f} mm^3 | watertight={mesh.is_watertight} | bodies={mesh.body_count}"
    )


def render_bobbypin_preview(
    boundary: np.ndarray, mesh_bobbypin: trimesh.Trimesh, out_path: Path
) -> None:
    """Render 3-panel visual verification of the bobby-pin pass-through variant."""
    fig = plt.figure(figsize=(14, 4.6), dpi=160)
    fig.patch.set_facecolor("#181a20")

    ax1 = fig.add_subplot(1, 3, 1)
    ax1.set_facecolor("#1e2129")
    b_closed = np.vstack([boundary, boundary[0]])
    ax1.fill(b_closed[:, 0], b_closed[:, 1], color="#f39c42", alpha=0.88, label="Clip Body (19.8 × 25.0 mm)")
    ax1.plot(b_closed[:, 0], b_closed[:, 1], color="#ffd296", linewidth=2)
    ax1.fill([-1.6, -1.6, 1.6, 1.6], [-12.5, 12.5, 12.5, -12.5], color="#20232a", label="Pass-Through Groove (3.2 mm wide)")
    ax1.fill([-1.6, -1.6, 1.6, 1.6], [-3.0, 3.5, 3.5, -3.0], color="#f39c42", edgecolor="#00ffcc", linewidth=2.0, label="Flush Retaining Bridge (6.5 mm)")
    ax1.plot([0, 0], [13.5, -15.5], color="#00ffcc", linewidth=3.0, alpha=0.85, label="Standard 50 mm Bobby Pin")
    ax1.set_aspect("equal")
    ax1.set_xlim(-15, 15)
    ax1.set_ylim(-16.5, 15.5)
    ax1.grid(True, color="#333846", linestyle=":", linewidth=0.8)
    ax1.tick_params(colors="#aaa", labelsize=8)
    ax1.set_xlabel("Width (mm)", color="#ccc", fontsize=9)
    ax1.set_ylabel("Length (mm)", color="#ccc", fontsize=9)
    ax1.set_title("Bottom View: Pass-Through Bobby Pin Loop", color="white", fontsize=11, pad=10)
    ax1.legend(loc="lower center", fontsize=7.5, facecolor="#222", edgecolor="#555", labelcolor="white")

    ax2 = fig.add_subplot(1, 3, 2)
    ax2.set_facecolor("#1e2129")
    slice_2d = mesh_bobbypin.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    if slice_2d is not None:
        for entity in slice_2d.entities:
            pts = slice_2d.vertices[entity.points]
            ax2.plot(pts[:, 1], pts[:, 2], color="#ffd296", linewidth=2.0)

    ax2.fill_between([-3.0, 3.5], [0.0, 0.0], [BOBBY_BRIDGE_HEIGHT, BOBBY_BRIDGE_HEIGHT], color="#f39c42", alpha=0.85, label="Retaining Bridge (Prints Flat on Bed)")
    by_in = np.linspace(13.0, -15.5, 100)
    bz_in = 1.45 * np.exp(-((by_in - 0.25) / 5.5) ** 4) + 0.45
    ax2.plot(by_in, bz_in, color="#00ffcc", linewidth=2.8, label="Bobby Pin Flat Leg (Through Tunnel)")
    by_out = np.linspace(13.0, -15.5, 140)
    bz_out = -0.55 + 0.18 * np.sin(by_out * 1.8)
    ax2.plot(by_out, bz_out, color="#00ffcc", linewidth=2.2, linestyle="--", label="Bobby Pin Wavy Leg (Clamps Hair)")
    theta_u = np.linspace(-0.5 * np.pi, 0.5 * np.pi, 30)
    ax2.plot(13.0 + 0.6 * np.cos(theta_u), -0.05 + 0.5 * np.sin(theta_u), color="#00ffcc", linewidth=2.8)
    ax2.set_aspect("equal")
    ax2.set_xlim(-16.5, 15.5)
    ax2.set_ylim(-3.5, 8.5)
    ax2.grid(True, color="#333846", linestyle=":", linewidth=0.8)
    ax2.tick_params(colors="#aaa", labelsize=8)
    ax2.set_xlabel("Length Y (mm)", color="#ccc", fontsize=9)
    ax2.set_ylabel("Height Z (mm)", color="#ccc", fontsize=9)
    ax2.set_title("Center Cross-Section (X = 0 mm)", color="white", fontsize=11, pad=10)
    ax2.legend(loc="upper left", fontsize=7.5, facecolor="#222", edgecolor="#555", labelcolor="white")

    ax3 = fig.add_subplot(1, 3, 3, projection="3d")
    ax3.set_facecolor("#181a20")
    v = mesh_bobbypin.vertices.copy()
    f = mesh_bobbypin.faces
    norms = mesh_bobbypin.face_normals.copy()
    v[:, 2] = CLIP_HEIGHT - v[:, 2]
    norms[:, 2] = -norms[:, 2]
    light_top = np.array([-0.4, 0.4, 0.82])
    light_top /= np.linalg.norm(light_top)
    intensity = np.clip(norms @ light_top, 0.2, 1.0)
    base_col = np.array([0.96, 0.58, 0.22])
    face_colors = np.clip(intensity[:, None] * base_col, 0.0, 1.0)
    poly = Poly3DCollection(v[f], facecolors=face_colors, edgecolors="none", linewidths=0)
    ax3.add_collection3d(poly)
    ax3.set_xlim(-13, 13)
    ax3.set_ylim(-13, 13)
    ax3.set_zlim(0, 13)
    ax3.view_init(elev=40, azim=-45)
    ax3.set_axis_off()
    ax3.set_title("Underside 3D View (Pass-Through Bridge)", color="white", fontsize=11, pad=10)

    plt.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)


def render_variants_overview(
    meshes: dict[str, trimesh.Trimesh], out_path: Path
) -> None:
    """Render underside 3D views of all three clip variants side by side."""
    fig = plt.figure(figsize=(14, 4.6), dpi=160)
    fig.patch.set_facecolor("#181a20")
    fig.suptitle(
        "Triangular Hairclip — Underside Comparison (19.8 × 25.0 × 5.8 mm)",
        color="white",
        fontsize=13,
        fontweight="bold",
    )

    titles = {
        "triangular_hairclip_bobbypin": "Bobby-Pin Pass-Through Bridge",
        "triangular_hairclip_recessed": "Recessed Alligator-Clip Channel",
        "triangular_hairclip_flat": "Flat-Back (Glue-On)",
    }

    light_top = np.array([-0.4, 0.4, 0.82])
    light_top /= np.linalg.norm(light_top)
    base_col = np.array([0.96, 0.58, 0.22])

    for idx, (key, mesh) in enumerate(meshes.items(), start=1):
        ax = fig.add_subplot(1, 3, idx, projection="3d")
        ax.set_facecolor("#181a20")
        v = mesh.vertices.copy()
        norms = mesh.face_normals.copy()
        v[:, 2] = CLIP_HEIGHT - v[:, 2]
        norms[:, 2] = -norms[:, 2]
        intensity = np.clip(norms @ light_top, 0.2, 1.0)
        face_colors = np.clip(intensity[:, None] * base_col, 0.0, 1.0)
        poly = Poly3DCollection(v[mesh.faces], facecolors=face_colors, edgecolors="none", linewidths=0)
        ax.add_collection3d(poly)
        ax.set_xlim(-13, 13)
        ax.set_ylim(-13, 13)
        ax.set_zlim(0, 13)
        ax.view_init(elev=38, azim=-48)
        ax.set_axis_off()
        ax.set_title(titles[key], color="white", fontsize=11, pad=8)

    plt.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)


def export_interactive_viewer(
    meshes: dict[str, trimesh.Trimesh], glb_path: Path, html_path: Path
) -> None:
    """Export all 3 variants arranged side-by-side into a Y-up GLB + self-contained Three.js viewer."""
    scene = trimesh.Scene()
    rot_z_to_y = trimesh.transformations.rotation_matrix(-math.pi / 2.0, [1, 0, 0])

    # Place Bobby-Pin in center (0), Recessed on left (-28), Flat on right (+28)
    offsets = {
        "triangular_hairclip_recessed": -28.0,
        "triangular_hairclip_bobbypin": 0.0,
        "triangular_hairclip_flat": 28.0,
    }
    colors = {
        "triangular_hairclip_bobbypin": [245, 148, 56, 255],
        "triangular_hairclip_recessed": [235, 128, 46, 255],
        "triangular_hairclip_flat": [224, 114, 40, 255],
    }

    for name, mesh in meshes.items():
        m = mesh.copy()
        m.apply_translation([offsets[name], 0.0, 0.0])
        m.apply_transform(rot_z_to_y)
        mat = trimesh.visual.material.PBRMaterial(
            baseColorFactor=colors[name], metallicFactor=0.04, roughnessFactor=0.32
        )
        m.visual = trimesh.visual.TextureVisuals(material=mat)
        scene.add_geometry(m, node_name=name)

    glb_bytes = scene.export(file_type="glb")
    glb_path.write_bytes(glb_bytes)
    glb_b64 = base64.b64encode(glb_bytes).decode("ascii")

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Triangular Hairclip — Interactive 3D Preview</title>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{ background: #111827; color: #f3f4f6; font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; overflow: hidden; }}
    #hud {{
      position: fixed; top: 14px; left: 14px; background: rgba(17, 24, 39, 0.84);
      backdrop-filter: blur(8px); border: 1px solid rgba(255, 255, 255, 0.12);
      border-radius: 10px; padding: 12px 14px; max-width: 320px; z-index: 10; font-size: 13px; line-height: 1.45;
    }}
    #hud h1 {{ font-size: 15px; margin-bottom: 4px; }}
    #hud .meta {{ color: #9ca3af; font-size: 12px; margin-bottom: 10px; }}
    .btns {{ display: flex; flex-wrap: wrap; gap: 6px; margin-top: 8px; }}
    button {{ background: #1f2937; color: #e5e7eb; border: 1px solid #374151; border-radius: 6px; padding: 5px 9px; font-size: 12px; cursor: pointer; }}
    button:hover {{ background: #374151; }}
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
    <h1>Triangular Hairclip</h1>
    <div class="meta">
      19.8 &times; 25.0 &times; 5.8 mm<br/>
      Left: Alligator Recess &middot; Center: Bobby-Pin Bridge &middot; Right: Flat Back<br/>
      Left-drag: Orbit &middot; Right-drag: Pan &middot; Scroll: Zoom
    </div>
    <div class="btns">
      <button data-view="iso">Top Iso</button>
      <button data-view="under">Underside</button>
      <button data-view="top">Top</button>
      <button data-view="side">Side</button>
    </div>
  </div>
  <script type="module">
    import * as THREE from 'three';
    import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';
    import {{ GLTFLoader }} from 'three/addons/loaders/GLTFLoader.js';

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111827);
    const camera = new THREE.PerspectiveCamera(40, innerWidth / innerHeight, 0.5, 1000);
    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setSize(innerWidth, innerHeight);
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    document.body.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 3, 0);

    scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.2));
    const key = new THREE.DirectionalLight(0xffffff, 1.8);
    key.position.set(60, 100, 80);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xcbd5e1, 1.1);
    fill.position.set(-60, -80, -60);
    scene.add(fill);

    const grid = new THREE.GridHelper(120, 24, 0x475569, 0x1e293b);
    scene.add(grid);

    const b64 = "{glb_b64}";
    const raw = Uint8Array.from(atob(b64), c => c.charCodeAt(0)).buffer;
    new GLTFLoader().parse(raw, '', (gltf) => {{
      scene.add(gltf.scene);
      setView('under');
    }});

    function setView(name) {{
      controls.target.set(0, 3, 0);
      const pos = {{
        iso:   new THREE.Vector3(45, 52, 65),
        under: new THREE.Vector3(38, -42, 55),
        top:   new THREE.Vector3(0, 95, 1),
        side:  new THREE.Vector3(95, 12, 0),
      }}[name];
      camera.position.copy(pos);
      controls.update();
    }}

    document.querySelectorAll('button[data-view]').forEach(btn => {{
      btn.addEventListener('click', () => setView(btn.dataset.view));
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


def main() -> None:
    EXPORTS_DIR.mkdir(parents=True, exist_ok=True)
    RENDERS_DIR.mkdir(parents=True, exist_ok=True)

    boundary = generate_curved_triangle_boundary()
    mesh_flat = build_hairclip_mesh(boundary, height=CLIP_HEIGHT, with_recess=False)
    mesh_recessed = build_hairclip_mesh(boundary, height=CLIP_HEIGHT, with_recess=True)
    mesh_bobbypin = build_bobbypin_mesh(mesh_flat)

    meshes = {
        "triangular_hairclip_bobbypin": mesh_bobbypin,
        "triangular_hairclip_recessed": mesh_recessed,
        "triangular_hairclip_flat": mesh_flat,
    }

    print("=== Exporting & Verifying Hairclip Variants ===")
    for name, mesh in meshes.items():
        verify_mesh(name, mesh)
        mesh.export(EXPORTS_DIR / f"{name}.stl")

    print("\n=== Generating Renders & Interactive Viewer ===")
    render_bobbypin_preview(boundary, mesh_bobbypin, RENDERS_DIR / "bobbypin_preview.png")
    render_variants_overview(meshes, RENDERS_DIR / "variants_overview.png")
    export_interactive_viewer(
        meshes,
        RENDERS_DIR / "assembly_preview.glb",
        RENDERS_DIR / "assembly_viewer.html",
    )
    print("  Saved renders & interactive viewer to:", RENDERS_DIR)


if __name__ == "__main__":
    main()
