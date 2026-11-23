"""3MF print-plate layout, overlap verification, and top-down preview rendering."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from build123d import Location, Mesher, Part


def place_on_plate(part: Part, target_min_x: float, target_min_y: float, label: str) -> Part:
    """Return a copy of `part` shifted so its bounding box min sits at (target_min_x, target_min_y, 0)."""
    bb = part.bounding_box()
    moved = part.moved(Location((target_min_x - bb.min.X, target_min_y - bb.min.Y, -bb.min.Z)))
    moved.label = label
    return moved


def center_layout(
    items: list[tuple[str, Part]], build_plate_mm: float = 256.0
) -> list[tuple[str, Part]]:
    """Center a list of placed parts on a square build plate."""
    min_x = min(p.bounding_box().min.X for _, p in items)
    max_x = max(p.bounding_box().max.X for _, p in items)
    min_y = min(p.bounding_box().min.Y for _, p in items)
    max_y = max(p.bounding_box().max.Y for _, p in items)
    dx = (build_plate_mm - (max_x - min_x)) / 2.0 - min_x
    dy = (build_plate_mm - (max_y - min_y)) / 2.0 - min_y
    centered: list[tuple[str, Part]] = []
    for name, p in items:
        mp = p.moved(Location((dx, dy, 0.0)))
        mp.label = name
        centered.append((name, mp))
    return centered


def export_3mf_plates(
    plates: dict[str, list[tuple[str, Part]]],
    out_dir: Path,
    build_plate_mm: float = 256.0,
    min_margin_mm: float = 5.0,
) -> None:
    """Verify zero overlaps & safe margins on each plate, then export .3mf files."""
    print(f"\n=== Exporting & Verifying 3MF Print Plates ({build_plate_mm:.0f}x{build_plate_mm:.0f} mm) ===")
    for plate_name, items in plates.items():
        boxes = [(name, p.bounding_box()) for name, p in items]
        for i in range(len(boxes)):
            n1, b1 = boxes[i]
            assert b1.min.X >= min_margin_mm and b1.max.X <= build_plate_mm - min_margin_mm, (
                f"{plate_name}/{n1} X out of safe plate bounds!"
            )
            assert b1.min.Y >= min_margin_mm and b1.max.Y <= build_plate_mm - min_margin_mm, (
                f"{plate_name}/{n1} Y out of safe plate bounds!"
            )
            assert abs(b1.min.Z) < 1e-3, f"{plate_name}/{n1} not on Z=0!"
            for j in range(i + 1, len(boxes)):
                n2, b2 = boxes[j]
                overlap_x = b1.min.X < b2.max.X and b2.min.X < b1.max.X
                overlap_y = b1.min.Y < b2.max.Y and b2.min.Y < b1.max.Y
                assert not (overlap_x and overlap_y), (
                    f"Overlap on {plate_name} between {n1} and {n2}!"
                )

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


def render_print_plates(
    plates: dict[str, list[tuple[str, Part]]],
    out_path: Path,
    color_map: dict[str, str],
    build_plate_mm: float = 256.0,
    title: str = "Ready-to-Slice 3MF Build Plates (256 × 256 mm Bed)",
) -> None:
    """Render top-down layout diagrams of all 3MF print plates."""
    n = len(plates)
    fig, axes = plt.subplots(1, n, figsize=(5.0 * n, 5.5), dpi=150)
    if n == 1:
        axes = [axes]
    fig.suptitle(title, fontsize=13, fontweight="bold")

    for ax, (plate_name, items) in zip(axes, plates.items()):
        ax.add_patch(
            plt.Rectangle(
                (0, 0), build_plate_mm, build_plate_mm,
                facecolor="#F8FAFC", edgecolor="#1E293B", linewidth=2.0,
            )
        )
        ax.add_patch(
            plt.Rectangle(
                (10, 10), build_plate_mm - 20, build_plate_mm - 20,
                facecolor="none", edgecolor="#94A3B8", linestyle="--", linewidth=0.9,
            )
        )

        for name, p in items:
            bb = p.bounding_box()
            color = next((c for k, c in color_map.items() if name.startswith(k)), "#64748B")
            ax.add_patch(
                plt.Rectangle(
                    (bb.min.X, bb.min.Y), bb.size.X, bb.size.Y,
                    facecolor=color, edgecolor="#1E293B", alpha=0.72, linewidth=1.1,
                )
            )
            ax.text(
                (bb.min.X + bb.max.X) / 2.0,
                (bb.min.Y + bb.max.Y) / 2.0,
                name.replace("_", "\n"),
                ha="center", va="center", fontsize=6.5, color="#0F172A", fontweight="bold",
            )

        ax.set_title(f"{plate_name}.3mf\n({len(items)} parts)", fontsize=10, fontweight="bold")
        ax.set_aspect("equal")
        ax.set_xlim(-6, build_plate_mm + 6)
        ax.set_ylim(-6, build_plate_mm + 6)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.grid(True, linestyle=":", alpha=0.4)

    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
