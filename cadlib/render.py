"""Matplotlib 3D mesh shading and axis-scaling helpers."""

from __future__ import annotations

import matplotlib.colors
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def add_mesh_to_ax(
    ax,
    mesh: trimesh.Trimesh,
    color: str,
    alpha: float = 1.0,
    edgecolor: str = "#222222",
    linewidth: float = 0.08,
    light_dir: tuple[float, float, float] = (0.4, -0.5, 0.76),
) -> None:
    """Add a shaded trimesh to a matplotlib 3D Axes."""
    verts = mesh.vertices[mesh.faces]
    normals = mesh.face_normals
    light = np.array(light_dir, dtype=float)
    light /= np.linalg.norm(light)
    intensity = np.clip(0.45 + 0.55 * np.abs(normals @ light), 0.35, 1.0)

    base_rgb = np.array(matplotlib.colors.to_rgb(color))
    face_colors = np.column_stack(
        [base_rgb * intensity[:, None], np.full(len(normals), alpha)]
    )
    poly = Poly3DCollection(
        verts, facecolors=face_colors, edgecolors=edgecolor, linewidths=linewidth
    )
    ax.add_collection3d(poly)


def set_equal_3d_axes(
    ax, bounds_min: np.ndarray, bounds_max: np.ndarray, pad: float = 10.0
) -> None:
    """Set equal-aspect cubic limits on a matplotlib 3D Axes."""
    center = (bounds_min + bounds_max) / 2.0
    max_range = float(np.max(bounds_max - bounds_min)) / 2.0 + pad
    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)
