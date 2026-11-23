# AGENTS.md

This repository stores and shares parametric, 3D-printable projects. Follow these guidelines when creating, modifying, or verifying designs.

## Environment & Tooling

- **Tool & venv management**: Managed by [`mise.toml`](mise.toml) and [`pyproject.toml`](pyproject.toml) using `uv`.
- **Dependencies**: Add required Python CAD, geometry, or rendering libraries with `uv add <package>` (for example: `build123d`, `cadquery`, `trimesh`, `numpy`, `matplotlib`) rather than installing manually with `pip`.
- **Shared library**: Reuse helpers in [`cadlib/`](cadlib/) (`verify_mesh`, `verify_stl`, `export_interactive_viewer`, `export_3mf_plates`, `render_print_plates`, `add_mesh_to_ax`, `set_equal_3d_axes`) instead of duplicating verification, 3MF plate layout, or Three.js viewer boilerplate across projects.
- **Running scripts**: Execute scripts inside the project virtual environment via `uv run python <script>` or directly when the `mise`-managed `.venv` is active.

## Project Organization

Place every project in `projects/<project-name>/` using lowercase kebab-case names:

```text
projects/<project-name>/
├── README.md      # Purpose, key dimensions, assembly notes, and recommended print settings
├── src/           # Parametric CAD scripts that deterministically generate all parts
├── exports/       # Generated .stl, .3mf, and/or .step files ready for slicing
└── renders/       # Preview images (isometric + orthographic views) for visual verification
```

- Keep all design work **code-first and parametric**: define key dimensions, wall thicknesses, and tolerances as named constants or parameters at the top of the design script.
- Regenerate files in `exports/` (and `renders/` when applicable) whenever source geometry changes so exports never drift out of sync with `src/`.
- Interactive 3D preview files (`renders/assembly_viewer.html`, `renders/assembly_preview.glb`) are gitignored and automatically built & published via [`.github/workflows/verify.yml`](.github/workflows/verify.yml).

## Design for Additive Manufacturing (DfAM)

Unless the user specifies a different printer or process, assume standard desktop FDM 3D printing on a **256 × 256 mm** build plate (0.4 mm nozzle, e.g., Bambu Lab A1 / P1 / X1):

- **Units**: Always work in **millimeters (mm)**.
- **Orientation**: Orient parts in their intended print orientation with the bottom resting cleanly on the build plane (`Z = 0`).
- **Build-plate packing**: Always generate ready-to-slice `.3mf` build plates packed to **maximize bed utilization and minimize the number of plates/trips** (the user prints at a remote print farm with camera monitoring). Keep safe bed margins and ~6–8 mm part spacing, and avoid separate small test-fit plates unless explicitly requested.
- **Tolerances & clearances**:
  - Press / tight fit: ~0.1–0.15 mm per side
  - Snug / slip fit: ~0.2–0.3 mm per side
  - Loose / moving parts: ~0.3–0.5 mm per side
- **Wall thickness**: Keep structural walls at least 1.2–1.6 mm thick (3–4 perimeters with a 0.4 mm nozzle).
- **Overhangs & supports**: Design to print without supports whenever practical—keep overhangs at or below 45°, prefer chamfers over steep bottom fillets, and use teardrop/chamfered tops for horizontal holes.
- **Bed adhesion & stress**: Maximize flat build-plate contact, add fillets to internal load-bearing corners, and consider small bottom chamfers (0.2–0.4 mm) on mating edges to compensate for elephant's foot.

## Verification Checklist

Before considering a design complete:

1. **Deterministic build**: Confirm the generation script runs cleanly from scratch and writes all expected files to `exports/`.
2. **Geometry & mesh checks**: Verify exported meshes are watertight/manifold, have no inverted normals or zero-volume shells, and match the expected bounding-box dimensions.
3. **Visual inspection**: Render multi-angle views (e.g., isometric, front, side, top, and cross-section if there are internal features) and inspect them to catch floating geometry, unintended intersections, or thin/fragile features.
4. **Documentation**: Update the project's `README.md` with dimensions, part list, assembly instructions (if multi-part), and recommended print settings (material, layer height, wall count, infill, supports, and orientation).
