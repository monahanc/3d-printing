# 3D Printing

My printable projects.
Since I am a software developer, I am not so good at 3D modeling—so many of these projects are designed with the help of AI agents, covering parametric modeling, printability checks, mesh exports, and documentation.

## Repository Structure

Each 3D-printable project lives in its own directory under `projects/`:

```text
projects/
└── <project-name>/
    ├── README.md      # Overview, dimensions, assembly notes, and print settings
    ├── src/           # Parametric CAD source scripts
    ├── exports/       # Print-ready meshes (.stl, .3mf) and CAD files (.step)
    └── renders/       # Multi-angle preview images for visual inspection
```

## Environment Setup

This repository uses [mise](https://mise.jdx.dev/) to manage tool versions (Python and `uv`) and [uv](https://docs.astral.sh/uv/) to manage the Python virtual environment and dependencies defined in [`pyproject.toml`](pyproject.toml).

1. Trust and install the configured tools:

   ```bash
   mise trust
   mise install
   ```

2. Sync the virtual environment (`.venv`) from `pyproject.toml`:

   ```bash
   mise run sync
   ```

   `mise` automatically creates and activates `.venv` via `uv` when you enter the repository directory.

3. Add Python CAD or mesh packages as needed:

   ```bash
   uv add <package>
   ```

## For AI Agents

See [`AGENTS.md`](AGENTS.md) for design conventions, Design for Additive Manufacturing (DfAM) guidelines, verification steps, and project organization standards.

## License

This repository is dual-licensed (see [`LICENSE`](LICENSE)):

- **Source code** (`src/`, tooling, configuration): [MIT License](LICENSE-MIT)
- **3D models, exported meshes, renders, and documentation** (`exports/`, `renders/`, `*.md`): [CC BY 4.0](LICENSE-CC-BY-4.0)
