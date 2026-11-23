"""Mesh printability and manifoldness verification utilities."""

from __future__ import annotations

from pathlib import Path

import trimesh


def verify_mesh(
    name: str,
    mesh: trimesh.Trimesh,
    build_plate_mm: float = 256.0,
    z_tolerance: float = 1e-2,
    verbose: bool = True,
) -> dict[str, object]:
    """Verify a mesh is watertight, positive-volume, rests on Z = 0, and fits the build plate."""
    bounds = mesh.bounds
    extents = mesh.extents
    min_z = float(bounds[0][2])
    fits_plate = all(float(dim) <= build_plate_mm for dim in extents)
    rests_on_z0 = abs(min_z) < z_tolerance

    info = {
        "name": name,
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "volume_mm3": float(mesh.volume),
        "volume_cm3": float(mesh.volume / 1000.0),
        "extents_mm": tuple(round(float(x), 2) for x in extents),
        "min_z_mm": round(min_z, 4),
        "body_count": int(mesh.body_count),
        "fits_plate": fits_plate,
        "rests_on_z0": rests_on_z0,
    }

    assert info["watertight"], f"{name} is NOT watertight!"
    assert info["volume_mm3"] > 0, f"{name} has non-positive volume!"
    assert fits_plate, f"{name} exceeds {build_plate_mm} mm build plate: {extents}"
    assert rests_on_z0, f"{name} does not rest cleanly on Z = 0 (min_z={min_z})"

    if verbose:
        print(
            f"  [OK] {name:28s} | bbox={info['extents_mm']} mm | "
            f"vol={info['volume_cm3']:.2f} cm^3 | watertight={info['watertight']} | "
            f"z0={info['min_z_mm']} mm"
        )
    return info


def verify_stl(
    path: Path,
    build_plate_mm: float = 256.0,
    z_tolerance: float = 1e-2,
    verbose: bool = True,
) -> dict[str, object]:
    """Load an exported STL file and run standard printability assertions."""
    mesh = trimesh.load_mesh(path, force="mesh")
    return verify_mesh(
        path.stem,
        mesh,
        build_plate_mm=build_plate_mm,
        z_tolerance=z_tolerance,
        verbose=verbose,
    )
