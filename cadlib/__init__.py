"""Shared utilities for parametric 3D-printable projects."""

from cadlib.plates import center_layout, export_3mf_plates, place_on_plate, render_print_plates
from cadlib.render import add_mesh_to_ax, set_equal_3d_axes
from cadlib.verify import verify_mesh, verify_stl
from cadlib.viewer import CameraView, LimitBox, ViewerItem, export_interactive_viewer

__all__ = [
    "CameraView",
    "LimitBox",
    "ViewerItem",
    "add_mesh_to_ax",
    "center_layout",
    "export_3mf_plates",
    "export_interactive_viewer",
    "place_on_plate",
    "render_print_plates",
    "set_equal_3d_axes",
    "verify_mesh",
    "verify_stl",
]
