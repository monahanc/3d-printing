"""Self-contained Three.js interactive 3D viewer and Y-up GLB exporter."""

from __future__ import annotations

import base64
import json
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.colors
import trimesh


@dataclass(frozen=True)
class ViewerItem:
    """One mesh in the interactive 3D scene (in CAD Z-up coordinates)."""

    mesh: trimesh.Trimesh
    color: str
    alpha: float = 1.0
    metallic: float = 0.08
    roughness: float = 0.55
    group: str = "part"


@dataclass(frozen=True)
class CameraView:
    """Named camera preset (coordinates in Three.js Y-up space)."""

    key: str
    label: str
    position: tuple[float, float, float]


@dataclass(frozen=True)
class LimitBox:
    """Optional red wireframe reference box (e.g. max-height ceiling plane)."""

    label: str
    size: tuple[float, float, float]
    position: tuple[float, float, float]


def export_interactive_viewer(
    items: list[ViewerItem],
    glb_path: Path,
    html_path: Path,
    *,
    title: str,
    heading: str,
    subtitle_html: str,
    target: tuple[float, float, float],
    views: list[CameraView],
    default_view: str | None = None,
    grid_size: float = 200.0,
    grid_divisions: int = 20,
    toggle_groups: dict[str, str] | None = None,
    limit_box: LimitBox | None = None,
) -> None:
    """Export CAD Z-up meshes into a Y-up GLB and self-contained Three.js HTML viewer."""
    scene = trimesh.Scene()
    rot_z_to_y = trimesh.transformations.rotation_matrix(-math.pi / 2.0, [1, 0, 0])

    for idx, item in enumerate(items):
        m = item.mesh.copy()
        m.apply_transform(rot_z_to_y)
        r, g, b = matplotlib.colors.to_rgb(item.color)
        rgba = [int(r * 255), int(g * 255), int(b * 255), int(item.alpha * 255)]

        mat_kwargs: dict[str, object] = {
            "baseColorFactor": rgba,
            "metallicFactor": item.metallic,
            "roughnessFactor": item.roughness,
        }
        if item.alpha < 1.0:
            mat_kwargs["alphaMode"] = "BLEND"

        m.visual = trimesh.visual.TextureVisuals(
            material=trimesh.visual.material.PBRMaterial(**mat_kwargs)
        )
        scene.add_geometry(m, node_name=f"{item.group}_{idx}")

    glb_bytes = scene.export(file_type="glb")
    glb_path.write_bytes(glb_bytes)
    glb_b64 = base64.b64encode(glb_bytes).decode("ascii")

    init_view = default_view or views[0].key
    views_js = json.dumps({v.key: list(v.position) for v in views})
    buttons_html = "\n      ".join(
        f'<button data-view="{v.key}">{v.label}</button>' for v in views
    )

    toggle_groups = toggle_groups or {}
    toggles_html_parts = [
        f'<label><input type="checkbox" data-group="{grp}" checked /> {lbl}</label>'
        for grp, lbl in toggle_groups.items()
    ]
    if limit_box is not None:
        toggles_html_parts.append(
            f'<label><input type="checkbox" id="toggleLimit" checked /> {limit_box.label}</label>'
        )
    toggles_html = "\n    ".join(toggles_html_parts)

    if limit_box is not None:
        sx, sy, sz = limit_box.size
        px, py, pz = limit_box.position
        limit_js = f"""
    const limitGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry({sx}, {sy}, {sz}));
    const limitMat = new THREE.LineBasicMaterial({{ color: 0xef4444 }});
    const limitBox = new THREE.LineSegments(limitGeo, limitMat);
    limitBox.position.set({px}, {py}, {pz});
    scene.add(limitBox);
    document.getElementById('toggleLimit').addEventListener('change', (e) => {{
      limitBox.visible = e.target.checked;
    }});"""
    else:
        limit_js = ""

    tx, ty, tz = target
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>{title}</title>
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
      max-width: 320px;
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
    <h1>{heading}</h1>
    <div class="meta">
      {subtitle_html}<br/>
      Left-drag: Orbit &middot; Right-drag: Pan &middot; Scroll: Zoom
    </div>
    {toggles_html}
    <div class="btns">
      {buttons_html}
    </div>
  </div>
  <script type="module">
    import * as THREE from 'three';
    import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';
    import {{ GLTFLoader }} from 'three/addons/loaders/GLTFLoader.js';

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111827);

    const camera = new THREE.PerspectiveCamera(40, innerWidth / innerHeight, 0.5, 5000);
    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setSize(innerWidth, innerHeight);
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    document.body.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    const target = new THREE.Vector3({tx}, {ty}, {tz});
    controls.target.copy(target);

    scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.15));
    const key = new THREE.DirectionalLight(0xffffff, 1.8);
    key.position.set(250, 420, 320);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xcbd5e1, 0.95);
    fill.position.set(-280, -180, -240);
    scene.add(fill);

    const grid = new THREE.GridHelper({grid_size}, {grid_divisions}, 0x475569, 0x1e293b);
    scene.add(grid);
    {limit_js}

    const b64 = "{glb_b64}";
    const raw = Uint8Array.from(atob(b64), c => c.charCodeAt(0)).buffer;
    const groupNodes = {{}};

    new GLTFLoader().parse(raw, '', (gltf) => {{
      const root = gltf.scene;
      root.traverse((obj) => {{
        if (obj.name && obj.name.includes('_')) {{
          const grp = obj.name.slice(0, obj.name.lastIndexOf('_'));
          (groupNodes[grp] ||= []).push(obj);
        }}
      }});
      scene.add(root);
      setView({json.dumps(init_view)});
    }});

    const views = {views_js};
    function setView(name) {{
      const p = views[name];
      if (!p) return;
      controls.target.copy(target);
      camera.position.set(p[0], p[1], p[2]);
      controls.update();
    }}

    document.querySelectorAll('button[data-view]').forEach(btn => {{
      btn.addEventListener('click', () => setView(btn.dataset.view));
    }});
    document.querySelectorAll('input[data-group]').forEach(cb => {{
      cb.addEventListener('change', (e) => {{
        const nodes = groupNodes[cb.dataset.group] || [];
        nodes.forEach(n => {{ n.visible = e.target.checked; }});
      }});
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
