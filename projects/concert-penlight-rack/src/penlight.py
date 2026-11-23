"""Detailed parametric 3D reference model of a standard concert penlight.

Dimensions extracted from calibrated reference photos (IMG_4481 / IMG_4482):
- Cylindrical light tube: ~160 mm long, 30.0 mm diameter, domed tip
- Chrome transition collar: ~20.5 mm long, 34.5 mm body (35.0 mm trim rings)
- Sculpted ergonomic handle: ~86 mm long, tapering from 32.0 mm -> 28.2 mm waist
  -> 32.4 mm palm swell -> rounded dome pommel (~266.5 mm total length)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import trimesh

# ---------------------------------------------------------------------------
# Measured Penlight Profile Parameters (mm)
# ---------------------------------------------------------------------------

TOTAL_LENGTH = 266.5
TUBE_LENGTH = 160.0
TUBE_DIAMETER = 30.0

COLLAR_LENGTH = 20.5
COLLAR_DIAMETER = 34.5
COLLAR_RING_DIAMETER = 35.0

HANDLE_LENGTH = 86.0
HANDLE_NECK_DIAMETER = 32.0
HANDLE_WAIST_DIAMETER = 28.2
HANDLE_SWELL_DIAMETER = 32.4
# Optional ovality ratio (depth / width) for sculpted handles; 1.0 = circular
HANDLE_OVALITY = 1.0


@dataclass(frozen=True)
class ProfileStation:
    """Axial station along the penlight (z = 0 at pommel bottom, z = TOTAL_LENGTH at tube tip)."""

    z: float
    radius: float
    section: str  # "handle", "collar", or "tube"


def get_profile_stations() -> list[ProfileStation]:
    """Return axial profile stations from pommel bottom (z=0) to tube tip (z=TOTAL_LENGTH)."""
    stations: list[ProfileStation] = []

    # 1. Rounded pommel dome (z = 0..24 mm, rising smoothly to palm swell r=16.2 mm)
    r_swell = HANDLE_SWELL_DIAMETER / 2.0
    dome_len = 24.0
    for i in range(13):
        t = i / 12.0  # 0 at tip, 1 at swell peak
        # Superellipse dome for realistic rounded handle butt
        z = dome_len * t
        r = r_swell * (1.0 - (1.0 - t) ** 2.2) ** 0.45
        stations.append(ProfileStation(z=z, radius=max(r, 0.2), section="handle"))

    # 2. Ergonomic curve from palm swell (z=24) -> waist (z=58) -> collar neck (z=86)
    z_swell = 24.0
    z_waist = 58.0
    z_neck = HANDLE_LENGTH  # 86.0
    r_waist = HANDLE_WAIST_DIAMETER / 2.0
    r_neck = HANDLE_NECK_DIAMETER / 2.0

    # Smooth cosine interpolation: swell -> waist
    for i in range(1, 11):
        t = i / 10.0
        z = z_swell + (z_waist - z_swell) * t
        r = r_waist + (r_swell - r_waist) * 0.5 * (1.0 + math.cos(math.pi * t))
        stations.append(ProfileStation(z=z, radius=r, section="handle"))

    # Smooth cosine interpolation: waist -> neck
    for i in range(1, 11):
        t = i / 10.0
        z = z_waist + (z_neck - z_waist) * t
        r = r_waist + (r_neck - r_waist) * 0.5 * (1.0 - math.cos(math.pi * t))
        stations.append(ProfileStation(z=z, radius=r, section="handle"))

    # 3. Chrome collar (z = 86.0 .. 106.5 mm) with subtle raised end trim rings
    z_c0 = HANDLE_LENGTH
    z_c1 = HANDLE_LENGTH + COLLAR_LENGTH
    r_col = COLLAR_DIAMETER / 2.0
    r_ring = COLLAR_RING_DIAMETER / 2.0
    collar_pts = [
        (z_c0 + 0.1, r_neck),
        (z_c0 + 0.8, r_ring),
        (z_c0 + 2.5, r_ring),
        (z_c0 + 3.2, r_col),
        (z_c1 - 3.2, r_col),
        (z_c1 - 2.5, r_ring),
        (z_c1 - 0.8, r_ring),
        (z_c1 - 0.1, TUBE_DIAMETER / 2.0),
    ]
    for z, r in collar_pts:
        stations.append(ProfileStation(z=z, radius=r, section="collar"))

    # 4. Cylindrical light tube (z = 106.5 .. 260.5 mm) + rounded top cap (260.5 .. 266.5 mm)
    r_tube = TUBE_DIAMETER / 2.0
    z_t_end = TOTAL_LENGTH - 6.0
    stations.append(ProfileStation(z=z_c1, radius=r_tube, section="tube"))
    stations.append(ProfileStation(z=z_t_end, radius=r_tube, section="tube"))

    for i in range(1, 9):
        t = i / 8.0
        z = z_t_end + 6.0 * t
        r = r_tube * math.sqrt(max(0.0, 1.0 - t * t))
        stations.append(ProfileStation(z=z, radius=max(r, 0.2), section="tube"))

    return stations


def radius_at_z(z: float) -> float:
    """Interpolate outer radius at axial coordinate z (0 = pommel, TOTAL_LENGTH = tip)."""
    stations = get_profile_stations()
    zs = [s.z for s in stations]
    rs = [s.radius for s in stations]
    return float(np.interp(z, zs, rs))


def build_penlight_meshes(
    tube_color: str = "#FF4D8D",
    handle_color: str = "#F4A6C1",
    collar_color: str = "#D1D5DB",
    sections: int = 36,
) -> list[tuple[trimesh.Trimesh, str, float]]:
    """Build multi-part revolved meshes oriented along +X, centered at X = 0.

    Returns list of (mesh, hex_color, alpha) tuples for handle, collar, and tube.
    """
    stations = get_profile_stations()
    angles = np.linspace(0.0, 2.0 * math.pi, sections, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    grouped: dict[str, list[ProfileStation]] = {"handle": [], "collar": [], "tube": []}
    for st in stations:
        grouped[st.section].append(st)

    # Connect adjacent section boundaries cleanly
    grouped["collar"].insert(0, grouped["handle"][-1])
    grouped["tube"].insert(0, grouped["collar"][-1])

    style = {
        "handle": (handle_color, 0.98),
        "collar": (collar_color, 1.0),
        "tube": (tube_color, 0.85),
    }

    result: list[tuple[trimesh.Trimesh, str, float]] = []
    half_len = TOTAL_LENGTH / 2.0

    for name, st_list in grouped.items():
        rings: list[np.ndarray] = []
        for st in st_list:
            x = st.z - half_len
            ry = st.radius
            rz = st.radius * (HANDLE_OVALITY if name == "handle" else 1.0)
            ring = np.column_stack([np.full(sections, x), ry * cos_a, rz * sin_a])
            rings.append(ring)

        verts = np.vstack(rings)
        faces: list[list[int]] = []
        n_rings = len(rings)
        for i in range(n_rings - 1):
            base0 = i * sections
            base1 = (i + 1) * sections
            for j in range(sections):
                jn = (j + 1) % sections
                faces.append([base0 + j, base0 + jn, base1 + jn])
                faces.append([base0 + j, base1 + jn, base1 + j])

        # Cap start and end rings
        c0_idx = len(verts)
        c1_idx = len(verts) + 1
        c0 = np.array([[st_list[0].z - half_len, 0.0, 0.0]])
        c1 = np.array([[st_list[-1].z - half_len, 0.0, 0.0]])
        verts = np.vstack([verts, c0, c1])
        for j in range(sections):
            jn = (j + 1) % sections
            faces.append([c0_idx, jn, j])
            last = (n_rings - 1) * sections
            faces.append([c1_idx, last + j, last + jn])

        mesh = trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=True)
        mesh.fix_normals()
        color, alpha = style[name]
        result.append((mesh, color, alpha))

    return result


if __name__ == "__main__":
    # Quick diagnostic: check penlight diameter at the two cradle spans
    # Current rack: INNER_SPAN = 160 mm, TOWER_THICKNESS = 18 mm -> cradles at X in [-98..-80] and [+80..+98]
    half = TOTAL_LENGTH / 2.0
    print(f"Total modeled penlight length: {TOTAL_LENGTH:.1f} mm")
    for label, x0, x1 in [("Handle cradle (-98..-80)", -98.0, -80.0), ("Tube cradle (+80..+98)", 80.0, 98.0)]:
        zs = np.linspace(x0 + half, x1 + half, 20)
        ds = [2.0 * radius_at_z(z) for z in zs]
        print(f"  {label}: z={zs[0]:.1f}..{zs[-1]:.1f} mm -> diameter {min(ds):.2f}..{max(ds):.2f} mm")
    print(f"  Max chrome collar diameter: {COLLAR_RING_DIAMETER:.2f} mm (at z={HANDLE_LENGTH:.1f}..{HANDLE_LENGTH+COLLAR_LENGTH:.1f} mm)")
    print(f"  Max handle swell diameter : {HANDLE_SWELL_DIAMETER:.2f} mm (at z=24.0 mm)")
