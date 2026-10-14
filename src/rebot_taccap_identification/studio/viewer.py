"""3-D URDF viewer for the identification studio (pyqtgraph.opengl).

Dark background, ground grid, RGB base
axes, orbit camera that starts in the +X/-Y/+Z quadrant looking at the arm,
wheel zoom and a home view. Meshes are decimated once for display; each
movable link is one mesh item whose transform is the link pose.
"""

from __future__ import annotations

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import shaders
from PySide6.QtGui import QMatrix4x4, QVector3D

from ..visual import load_visuals, read_stl, simplify_triangles

# Two-sided Lambert with a key and a fill light (in view space), so CAD meshes with
# mixed face winding are lit from both sides instead of turning black.
shaders.ShaderProgram("twosided", [
    shaders.VertexShader("""
        uniform mat4 u_mvp;
        uniform mat3 u_normal;
        attribute vec4 a_position;
        attribute vec3 a_normal;
        attribute vec4 a_color;
        varying vec4 v_color;
        varying vec3 v_normal;
        void main() {
            v_normal = normalize(u_normal * a_normal);
            v_color = a_color;
            gl_Position = u_mvp * a_position;
        }
    """),
    shaders.FragmentShader("""
        #ifdef GL_ES
        precision mediump float;
        #endif
        varying vec4 v_color;
        varying vec3 v_normal;
        void main() {
            float key = abs(dot(v_normal, normalize(vec3(0.4, 0.6, 1.0))));
            float fill = abs(dot(v_normal, normalize(vec3(-0.7, -0.2, 0.4))));
            vec3 rgb = v_color.rgb * (0.28 + 0.62 * key + 0.22 * fill);
            gl_FragColor = vec4(rgb, v_color.a);
        }
    """),
])

LINK_TONES = ((0.80, 0.84, 0.88), (0.62, 0.66, 0.72))
HOME_AZIMUTH, HOME_ELEVATION = -45.0, 28.0


def arm_reach(robot) -> float:
    """Distance from the base to the farthest joint origin at the zero pose, plus 15 % (at least 0.2 m)."""
    _, origins, _ = robot._joint_geometry_batch(np.zeros((1, robot.dof)))
    reach = float(np.linalg.norm(origins[0], axis=1).max())
    return max(reach * 1.15, 0.2)


class RobotView(gl.GLViewWidget):
    """3-D view of the arm (meshes per link, optional payload)."""

    def __init__(self, robot, tip_link: str | None, mesh_cell: float = 0.002, parent=None):
        super().__init__(parent)
        self.setBackgroundColor((20, 23, 28))
        self.mesh_cell = mesh_cell
        self.scene_items: list = []
        self.load_robot(robot, tip_link)

    def load_robot(self, robot, tip_link: str | None) -> None:
        for item in self.scene_items:
            self.removeItem(item)
        self.scene_items = []
        self.robot = robot
        self.reach = arm_reach(robot)              # camera and grid scale with the arm
        grid = gl.GLGridItem()
        size = float(np.ceil(4 * self.reach * 10) / 10)
        spacing = 0.1 if size <= 3 else 0.25
        grid.setSize(size, size)
        grid.setSpacing(spacing, spacing)
        grid.setColor((70, 74, 82, 255))
        self._add(grid)
        for axis, color in ((0, (0.85, 0.15, 0.15, 1)), (1, (0.15, 0.85, 0.25, 1)), (2, (0.15, 0.35, 0.95, 1))):
            end = np.zeros(3)
            end[axis] = 0.3 * self.reach
            self._add(gl.GLLinePlotItem(pos=np.array([np.zeros(3), end]), color=color, width=3, antialias=True))

        self.link_items: dict[int, gl.GLMeshItem] = {}
        grouped: dict[int, list[np.ndarray]] = {}
        for visual in load_visuals(robot, tip_link):
            if visual.kind != "mesh" or visual.path is None or not visual.path.is_file():
                continue
            triangles = simplify_triangles(read_stl(visual.path) * visual.scale, self.mesh_cell)
            local = triangles.reshape(-1, 3) @ visual.transform[:3, :3].T + visual.transform[:3, 3]
            grouped.setdefault(visual.owner, []).append(local.reshape(-1, 3, 3))
        for owner, parts in grouped.items():
            tone = LINK_TONES[(owner + 2) % 2]
            item = gl.GLMeshItem(vertexes=np.vstack(parts).astype(np.float32), smooth=False, shader="twosided",
                                 color=(*tone, 1.0), drawEdges=False)
            self.link_items[owner] = item
            self._add(item)
        self.tool_item = None
        self.home()
        self.set_pose(np.zeros(robot.dof))

    def _add(self, item) -> None:
        self.addItem(item)
        self.scene_items.append(item)

    def home(self) -> None:
        self.setCameraPosition(pos=QVector3D(0.15 * self.reach, 0.0, 0.45 * self.reach), distance=6.0 * self.reach,
                               azimuth=HOME_AZIMUTH, elevation=HOME_ELEVATION)

    def link_frames(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        rotations, origins, _ = self.robot._joint_geometry_batch(np.atleast_2d(np.asarray(q, dtype=float)))
        return rotations, origins

    def set_tool(self, visuals) -> None:
        """Show a tool (mesh visuals rigidly attached to the last link) or hide it (``None``)."""
        if self.tool_item is not None:
            self.removeItem(self.tool_item)
            self.scene_items.remove(self.tool_item)
            self.tool_item = None
        parts = []
        for visual in visuals or []:
            if visual.kind != "mesh" or visual.path is None or not visual.path.is_file():
                continue
            triangles = simplify_triangles(read_stl(visual.path) * visual.scale, self.mesh_cell)
            parts.append((triangles.reshape(-1, 3) @ visual.transform[:3, :3].T + visual.transform[:3, 3]).reshape(-1, 3, 3))
        if parts:
            self.tool_item = gl.GLMeshItem(vertexes=np.vstack(parts).astype(np.float32), smooth=False,
                                           shader="twosided", color=(0.85, 0.6, 0.25, 1.0), drawEdges=False)
            self._add(self.tool_item)

    def set_pose(self, q: np.ndarray) -> None:
        rotations, origins = self.link_frames(q)
        if self.tool_item is not None:
            R, t = rotations[0, -1], origins[0, -1]
            self.tool_item.setTransform(QMatrix4x4(*R[0], t[0], *R[1], t[1], *R[2], t[2], 0, 0, 0, 1))
        for owner, item in self.link_items.items():
            matrix = QMatrix4x4()
            if owner >= 0:
                R, t = rotations[0, owner], origins[0, owner]
                matrix = QMatrix4x4(*R[0], t[0], *R[1], t[1], *R[2], t[2], 0, 0, 0, 1)
            item.setTransform(matrix)
