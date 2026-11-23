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

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from cadlib import CameraView, ViewerItem, export_interactive_viewer, verify_mesh

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
    verts = np.array([[0.0, y_bot], [w0, y_top], [-w0, y_top]])

    boundary_pts = []
    for i in range(3):
        p0, p1, p_prev = verts[i], verts[(i + 1) % 3], verts[(i - 1) % 3]

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
        boundary_pts.append(p0 + r_offset * np.column_stack([np.cos(angs), np.sin(angs)]))

        t = np.linspace(0, 1, 180, endpoint=False)[:, None]
        pts = (1 - t) ** 2 * p0 + 2 * (1 - t) * t * ctrl_curr + t**2 * p1
        dpts = 2 * (1 - t) * (ctrl_curr - p0) + 2 * t * (p1 - ctrl_curr)
        norms = np.column_stack([dpts[:, 1], -dpts[:, 0]]) / np.linalg.norm(dpts, axis=1, keepdims=True)
        boundary_pts.append(pts + r_offset * norms)

    b = np.vstack(boundary_pts)
    diffs = np.diff(np.vstack([b, b[0]]), axis=0)
    cum_len = np.concatenate([[0], np.cumsum(np.linalg.norm(diffs, axis=1))])
    s_uniform = np.linspace(0, cum_len[-1], 360, endpoint=False)
    b = np.column_stack([
        np.interp(s_uniform, cum_len, np.append(b[:, 0], b[0, 0])),
        np.interp(s_uniform, cum_len, np.append(b[:, 1], b[0, 1])),
    ])

    b[:, 0] -= 0.5 * (b[:, 0].min() + b[:, 0].max())
    b[:, 1] -= 0.5 * (b[:, 1].min() + b[:, 1].max())
    b[:, 0] *= width / (b[:, 0].max() - b[:, 0].min())
    b[:, 1] *= length / (b[:, 1].max() - b[:, 1].min())
    return b


def recess_depth_fn(x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray | float:
    """Open-top alligator clip channel (6.4 mm wide × 20.0 mm long × 0.85 mm deep)."""
    dx = np.abs(x) - (RECESS_HALF_WIDTH - 0.4)
    dy = RECESS_Y_MIN - y - 0.4
    d_slot = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0)) + np.minimum(np.maximum(dx, dy), 0.0) - 0.4
    return np.clip((0.5 - d_slot) * (RECESS_DEPTH / 0.5), 0.0, RECESS_DEPTH)


def build_hairclip_mesh(
    boundary: np.ndarray, height: float = CLIP_HEIGHT, with_recess: bool = False
) -> trimesh.Trimesh:
    """Build the domed hairclip mesh resting flat on Z = 0."""
    center = boundary.mean(axis=0)
    rel_b = boundary - center
    n_pts = len(boundary)

    phi = np.linspace(0.0, 0.5 * np.pi, 60, endpoint=False)
    u_top = np.cos(phi)
    z_dome = height * np.power(np.sin(phi), 0.75)
    u_bot = np.linspace(1.0, 0.0, 46)[1:-1]

    vertices = []
    for u, zd in zip(u_top, z_dome):
        ring_xy = center + u * rel_b
        z_ring = np.maximum(zd, recess_depth_fn(ring_xy[:, 0], ring_xy[:, 1])) if with_recess else np.full(n_pts, zd)
        vertices.append(np.column_stack([ring_xy, z_ring]))

    for u in u_bot:
        ring_xy = center + u * rel_b
        z_vals = recess_depth_fn(ring_xy[:, 0], ring_xy[:, 1]) if with_recess else np.zeros(n_pts)
        vertices.append(np.column_stack([ring_xy, z_vals]))

    m_top, m_bot = len(u_top), len(u_bot)
    vertices_arr = np.vstack(vertices)
    idx_top_apex = len(vertices_arr)
    idx_bot_apex = len(vertices_arr) + 1
    bot_center_z = float(recess_depth_fn(center[0], center[1])) if with_recess else 0.0
    vertices_arr = np.vstack([
        vertices_arr,
        [[center[0], center[1], height]],
        [[center[0], center[1], bot_center_z]],
    ])

    faces = []
    for r in range(m_top - 1):
        b0, b1 = r * n_pts, (r + 1) * n_pts
        for i in range(n_pts):
            inext = (i + 1) % n_pts
            faces.append([b0 + i, b0 + inext, b1 + inext])
            faces.append([b0 + i, b1 + inext, b1 + i])
    base_last_top = (m_top - 1) * n_pts
    for i in range(n_pts):
        faces.append([base_last_top + i, base_last_top + (i + 1) % n_pts, idx_top_apex])

    bot_rings = [0] + [m_top + b_idx for b_idx in range(m_bot)]
    for r0, r1 in zip(bot_rings[:-1], bot_rings[1:]):
        b0, b1 = r0 * n_pts, r1 * n_pts
        for i in range(n_pts):
            inext = (i + 1) % n_pts
            faces.append([b0 + inext, b0 + i, b1 + inext])
            faces.append([b1 + inext, b0 + i, b1 + i])
    base_last_bot = bot_rings[-1] * n_pts
    for i in range(n_pts):
        faces.append([base_last_bot + (i + 1) % n_pts, base_last_bot + i, idx_bot_apex])

    mesh = trimesh.Trimesh(vertices=vertices_arr, faces=np.array(faces, dtype=np.int64), process=True)
    trimesh.repair.fix_normals(mesh)
    return mesh


def make_bobbypin_cutter(width: float = BOBBY_GROOVE_WIDTH) -> trimesh.Trimesh:
    """Construct manifold boolean cutter for the pass-through bobby-pin groove & tunnel."""
    ny, nx = 140, 21
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
        mask_bot = (y >= -8.5) & (y < -5.5)
        z_c[mask_bot] = 1.10 + (BOBBY_TUNNEL_CEILING - 1.10) * ((y[mask_bot] + 8.5) / 3.0)
        z_c[y < -8.5] = 1.10
        mask_top = (y > 6.0) & (y <= 9.0)
        z_c[mask_top] = BOBBY_TUNNEL_CEILING - (BOBBY_TUNNEL_CEILING - 1.10) * ((y[mask_top] - 6.0) / 3.0)
        z_c[y > 9.0] = 1.10
        return z_c + BOBBY_ROOF_PITCH * (1.0 - np.abs(x_u))

    uu, yy = np.meshgrid(xs_unit, ys, indexing="ij")
    xx = uu * half_width_fn(yy)
    verts = np.vstack([
        np.column_stack([xx.ravel(), yy.ravel(), z_bot_fn(yy).ravel()]),
        np.column_stack([xx.ravel(), yy.ravel(), z_top_fn(uu, yy).ravel()]),
    ])
    n_grid = nx * ny
    idx_b = lambda i, j: i * ny + j
    idx_t = lambda i, j: n_grid + i * ny + j

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
    mesh = trimesh.boolean.difference([base_mesh, make_bobbypin_cutter()], engine="manifold")
    trimesh.repair.fix_normals(mesh)
    return mesh


# ---------------------------------------------------------------------------
# Renders
# ---------------------------------------------------------------------------


def _plot_underside_3d(ax, mesh: trimesh.Trimesh, title: str, elev: float = 38, azim: float = -48) -> None:
    ax.set_facecolor("#181a20")
    v = mesh.vertices.copy()
    norms = mesh.face_normals.copy()
    v[:, 2] = CLIP_HEIGHT - v[:, 2]
    norms[:, 2] = -norms[:, 2]
    light_top = np.array([-0.4, 0.4, 0.82])
    light_top /= np.linalg.norm(light_top)
    intensity = np.clip(norms @ light_top, 0.2, 1.0)
    face_colors = np.clip(intensity[:, None] * np.array([0.96, 0.58, 0.22]), 0.0, 1.0)
    ax.add_collection3d(Poly3DCollection(v[mesh.faces], facecolors=face_colors, edgecolors="none", linewidths=0))
    ax.set_xlim(-13, 13)
    ax.set_ylim(-13, 13)
    ax.set_zlim(0, 13)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(title, color="white", fontsize=11, pad=10)


def render_bobbypin_preview(
    boundary: np.ndarray, mesh_bobbypin: trimesh.Trimesh, out_path: Path
) -> None:
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
    ax2.plot(by_in, 1.45 * np.exp(-((by_in - 0.25) / 5.5) ** 4) + 0.45, color="#00ffcc", linewidth=2.8, label="Bobby Pin Flat Leg (Through Tunnel)")
    by_out = np.linspace(13.0, -15.5, 140)
    ax2.plot(by_out, -0.55 + 0.18 * np.sin(by_out * 1.8), color="#00ffcc", linewidth=2.2, linestyle="--", label="Bobby Pin Wavy Leg (Clamps Hair)")
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

    _plot_underside_3d(fig.add_subplot(1, 3, 3, projection="3d"), mesh_bobbypin, "Underside 3D View (Pass-Through Bridge)", elev=40, azim=-45)
    plt.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)


def render_variants_overview(meshes: dict[str, trimesh.Trimesh], out_path: Path) -> None:
    fig = plt.figure(figsize=(14, 4.6), dpi=160)
    fig.patch.set_facecolor("#181a20")
    fig.suptitle(
        "Triangular Hairclip — Underside Comparison (19.8 × 25.0 × 5.8 mm)",
        color="white", fontsize=13, fontweight="bold",
    )
    titles = {
        "triangular_hairclip_bobbypin": "Bobby-Pin Pass-Through Bridge",
        "triangular_hairclip_recessed": "Recessed Alligator-Clip Channel",
        "triangular_hairclip_flat": "Flat-Back (Glue-On)",
    }
    for idx, (key, mesh) in enumerate(meshes.items(), start=1):
        _plot_underside_3d(fig.add_subplot(1, 3, idx, projection="3d"), mesh, titles[key])
    plt.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)


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

    viewer_specs = [
        ("triangular_hairclip_recessed", -28.0, "#EB802E"),
        ("triangular_hairclip_bobbypin", 0.0, "#F59438"),
        ("triangular_hairclip_flat", 28.0, "#E07228"),
    ]
    viewer_items = []
    for name, x_off, color in viewer_specs:
        m = meshes[name].copy()
        m.apply_translation([x_off, 0.0, 0.0])
        viewer_items.append(ViewerItem(m, color, metallic=0.04, roughness=0.32, group=name))

    export_interactive_viewer(
        viewer_items,
        RENDERS_DIR / "assembly_preview.glb",
        RENDERS_DIR / "assembly_viewer.html",
        title="Triangular Hairclip — Interactive 3D Preview",
        heading="Triangular Hairclip",
        subtitle_html="19.8 &times; 25.0 &times; 5.8 mm<br/>Left: Alligator Recess &middot; Center: Bobby-Pin Bridge &middot; Right: Flat Back",
        target=(0.0, 3.0, 0.0),
        views=[
            CameraView("iso", "Top Iso", (45, 52, 65), ortho=False),
            CameraView("under", "Underside", (38, -42, 55), ortho=False),
            CameraView("top", "Top", (0, 105, 0.01), ortho=True),
            CameraView("front", "Front", (0, 3, 105), ortho=True),
            CameraView("side", "Side", (105, 3, 0), ortho=True),
        ],
        default_view="under",
        grid_size=120.0,
        grid_divisions=24,
    )
    print("  Saved renders & interactive viewer to:", RENDERS_DIR)


if __name__ == "__main__":
    main()
