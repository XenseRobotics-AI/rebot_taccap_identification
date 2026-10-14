"""URDF visual geometry and approximate collision clearance along a joint path.

Visual meshes are attached to the movable link that owns them (fixed children
are merged, links below ``tip_link`` are dropped), matching ``URDFRobotModel``.
Clearance is estimated from points sampled on the mesh surfaces, so distances
are accurate to roughly the sample spacing.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from .urdf import URDFRobotModel, _origin, _vector


@dataclass
class Visual:
    owner: int                # -1 for the root link, else movable joint index
    link: str
    transform: np.ndarray     # 4x4, visual frame in the owner link frame
    kind: str                 # mesh | box | cylinder | sphere
    path: Path | None = None
    scale: np.ndarray | None = None
    size: np.ndarray | None = None   # box: xyz; cylinder: (radius, length); sphere: (radius,)
    color: tuple[float, float, float, float] = (0.75, 0.75, 0.75, 1.0)


def _homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rotation, translation
    return T


def _resolve_mesh(filename: str, urdf_path: Path) -> Path:
    if filename.startswith("file://"):
        return Path(filename[7:])
    if filename.startswith("package://"):
        package, _, relative = filename[10:].partition("/")
        if package == "rebot_taccap_identification":
            return Path(__file__).resolve().parent / relative
        for parent in urdf_path.resolve().parents:
            if parent.name == package:
                return parent / relative
            if (parent / package).is_dir():
                return parent / package / relative
        # Common layout: <package>/urdf/robot.urdf with meshes next to urdf/.
        return urdf_path.resolve().parents[1] / relative
    path = Path(filename)
    if path.is_absolute():
        return path
    # Relative to the URDF; exporters often put meshes/ next to urdf/ instead.
    candidates = [parent / path for parent in urdf_path.resolve().parents[:3]]
    return next((c for c in candidates if c.is_file()), candidates[0])


def load_visuals(robot: URDFRobotModel, tip_link: str | None = None, element: str = "visual",
                 tool: tuple[float, float] | bool | None = None) -> list[Visual]:
    """Visual (or collision) geometry of the links kept by ``robot``.

    ``tool`` adds a payload rigidly attached to the last link, for collision checks
    with e.g. a gripper mounted: ``True`` keeps the URDF links below ``tip_link``
    (all their joints frozen at 0), ``(length, radius)`` [m] adds a cylinder along the
    last link's z axis (for a tool the URDF does not describe).
    """
    root = ET.parse(robot.source).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    materials = {m.get("name"): m for m in root.findall("material")}
    children: dict[str, list[ET.Element]] = {name: [] for name in links}
    for joint in root.findall("joint"):
        children[joint.find("parent").get("link")].append(joint)
    owner_of = {name: index for index, name in enumerate(robot.link_names)}
    visuals: list[Visual] = []

    def color_of(item: ET.Element) -> tuple[float, float, float, float]:
        material = item.find("material")
        if material is not None:
            color = material.find("color")
            if color is None and material.get("name") in materials:
                color = materials[material.get("name")].find("color")
            if color is not None and color.get("rgba"):
                return tuple(_vector(color.get("rgba"), 4, "material rgba"))  # type: ignore[return-value]
        return (0.75, 0.75, 0.75, 1.0)

    def walk(name: str, owner: int, T: np.ndarray) -> None:
        for item in links[name].findall(element):
            local = T @ _homogeneous(*_origin(item))
            geometry = item.find("geometry")
            shape = geometry[0] if geometry is not None and len(geometry) else None
            if shape is None:
                continue
            visual = Visual(owner, name, local, shape.tag, color=color_of(item))
            if shape.tag == "mesh":
                visual.path = _resolve_mesh(shape.get("filename", ""), robot.source)
                visual.scale = _vector(shape.get("scale", "1 1 1"), 3, "mesh scale")
            elif shape.tag == "box":
                visual.size = _vector(shape.get("size", "0 0 0"), 3, "box size")
            elif shape.tag == "cylinder":
                visual.size = np.array([float(shape.get("radius", 0)), float(shape.get("length", 0))])
            elif shape.tag == "sphere":
                visual.size = np.array([float(shape.get("radius", 0))])
            else:
                continue
            visuals.append(visual)
        if name == tip_link and tool is not True:
            return
        for joint in children[name]:
            child = joint.find("child").get("link")
            local = T @ _homogeneous(*_origin(joint))
            if joint.get("type") == "fixed":
                walk(child, owner, local)
            elif child in owner_of:
                walk(child, owner_of[child], np.eye(4))
            elif tool is True and owner == len(robot.link_names) - 1:
                walk(child, owner, local)          # part of the tool: rigid, joint at 0

    walk(robot.config.root_link, -1, np.eye(4))
    if isinstance(tool, tuple) and tool[0] > 0:
        length, radius = tool
        transform = np.eye(4)
        transform[2, 3] = length / 2
        visuals.append(Visual(robot.dof - 1, "tool", transform, "cylinder", size=np.array([radius, length]),
                              color=(0.9, 0.6, 0.2, 1.0)))
    return visuals


def has_tool_links(robot: URDFRobotModel, tip_link: str | None) -> bool:
    """Whether the URDF describes parts below ``tip_link`` (e.g. a gripper removed for identification)."""
    if tip_link is None:
        return False
    root = ET.parse(robot.source).getroot()
    return any(j.find("parent").get("link") == tip_link for j in root.findall("joint"))


def read_stl(path: str | Path) -> np.ndarray:
    """Triangles ``(n, 3, 3)`` from a binary or ASCII STL file."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        count = struct.unpack("<I", data[80:84])[0]
        if len(data) == 84 + 50 * count:
            record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
            return np.frombuffer(data, dtype=record, count=count, offset=84)["vertices"].astype(float)
    numbers = [line.split()[1:] for line in data.decode("ascii", "replace").splitlines()
               if line.strip().startswith("vertex")]
    return np.asarray(numbers, dtype=float).reshape(-1, 3, 3)


def _sample_triangles(triangles: np.ndarray, count: int, rng: np.random.Generator) -> np.ndarray:
    edge1, edge2 = triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(edge1, edge2), axis=1)
    if area.sum() <= 0:
        return triangles.reshape(-1, 3)[:count]
    chosen = rng.choice(area.size, size=count, p=area / area.sum())
    u, v = rng.random((2, count))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    return triangles[chosen, 0] + u[:, None] * edge1[chosen] + v[:, None] * edge2[chosen]


def _primitive_points(visual: Visual, count: int, rng: np.random.Generator) -> np.ndarray:
    if visual.kind == "box":
        points = (rng.random((count, 3)) - 0.5) * visual.size
        axis = rng.integers(0, 3, count)
        points[np.arange(count), axis] = np.sign(points[np.arange(count), axis]) * visual.size[axis] / 2
        return points
    if visual.kind == "sphere":
        direction = rng.normal(size=(count, 3))
        return visual.size[0] * direction / np.linalg.norm(direction, axis=1, keepdims=True)
    radius, length = visual.size
    angle = rng.uniform(0, 2 * np.pi, count)
    return np.column_stack([radius * np.cos(angle), radius * np.sin(angle), rng.uniform(-length / 2, length / 2, count)])


def surface_points(visuals: list[Visual], points_per_link: int = 3000, seed: int = 0) -> dict[int, np.ndarray]:
    """Surface samples per owner link, expressed in the owner frame."""
    rng = np.random.default_rng(seed)
    grouped: dict[int, list[tuple[Visual, np.ndarray | None]]] = {}
    for visual in visuals:
        triangles = None
        if visual.kind == "mesh":
            if visual.path is None or not visual.path.is_file():
                continue
            triangles = read_stl(visual.path) * visual.scale
        grouped.setdefault(visual.owner, []).append((visual, triangles))
    result: dict[int, np.ndarray] = {}
    for owner, items in grouped.items():
        areas = np.array([
            0.5 * np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1).sum() if t is not None
            else float(np.prod(np.maximum(v.size, 1e-3))) for v, t in items
        ])
        counts = np.maximum(50, np.round(points_per_link * areas / max(areas.sum(), 1e-12)).astype(int))
        chunks = []
        for (visual, triangles), count in zip(items, counts):
            local = _sample_triangles(triangles, count, rng) if triangles is not None else _primitive_points(visual, count, rng)
            chunks.append(local @ visual.transform[:3, :3].T + visual.transform[:3, 3])
        result[owner] = np.vstack(chunks)
    return result


def link_poses(robot: URDFRobotModel, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rotations ``(n, dof, 3, 3)`` and origins ``(n, dof, 3)`` of each movable link."""
    rotations, origins, _ = robot._joint_geometry_batch(np.asarray(q, dtype=float))
    return rotations, origins


def clearance_report(
    robot: URDFRobotModel, q: np.ndarray, time: np.ndarray, points: dict[int, np.ndarray], *,
    table_z: float = 0.0, clearance: float = 0.01, reference_q: np.ndarray | None = None,
) -> dict:
    """Minimum distances between non-adjacent links and to the table along ``q``.

    Pairs already closer than ``clearance`` at ``reference_q`` (default: the
    first sample) are reported but not flagged: their meshes touch by design.
    Returns per-pair minima, the lowest point above the table, and the times
    at which any check is violated.
    """
    from scipy.spatial import cKDTree

    q = np.asarray(q, dtype=float)
    rotations, origins = link_poses(robot, q)
    n = q.shape[0]
    owners = sorted(points)

    def pose(owner: int) -> tuple[np.ndarray, np.ndarray]:
        if owner < 0:
            return np.broadcast_to(np.eye(3), (n, 3, 3)), np.zeros((n, 3))
        return rotations[:, owner], origins[:, owner]

    centers = {o: points[o].mean(axis=0) for o in owners}
    radii = {o: float(np.linalg.norm(points[o] - centers[o], axis=1).max()) for o in owners}
    trees = {o: cKDTree(points[o]) for o in owners}
    colliding = np.zeros((n, robot.dof + 1), dtype=bool)   # column 0 is the root link

    def names(owner: int) -> str:
        return robot.config.root_link if owner < 0 else robot.link_names[owner]

    def pair_distances(a: int, b: int, frames: np.ndarray, Ra, ta, Rb, tb) -> np.ndarray:
        """Nearest distance between the two point sets for each frame (batched KD queries)."""
        out = np.full(frames.size, np.inf)
        chunk = max(1, 400_000 // len(points[a]))
        for start in range(0, frames.size, chunk):
            f = frames[start:start + chunk]
            # Points of a in the frame of b: p @ (Ra^T Rb) + (ta - tb) @ Rb, as one batched matmul.
            M = np.transpose(Ra[f], (0, 2, 1)) @ Rb[f]
            c = np.einsum("fi,fij->fj", ta[f] - tb[f], Rb[f])
            in_b = points[a][None] @ M + c[:, None, :]
            queries = in_b.reshape(-1, 3)
            distance = trees[b].query(queries, k=1, distance_upper_bound=0.1,
                                      workers=-1 if len(queries) > 100_000 else 1)[0]
            out[start:start + f.size] = distance.reshape(f.size, -1).min(axis=1)
        return out

    # One or more reference poses; a pair closer than ``clearance`` in any of them is structural.
    reference = q[:1] if reference_q is None else np.atleast_2d(np.asarray(reference_q, dtype=float))
    ref_rot, ref_org = link_poses(robot, reference)

    pairs = []
    for i, a in enumerate(owners):
        for b in owners[i + 1:]:
            if abs(a - b) <= 1:
                continue
            Ra, ta = pose(a)
            Rb, tb = pose(b)
            ca = np.einsum("nij,j->ni", Ra, centers[a]) + ta
            cb = np.einsum("nij,j->ni", Rb, centers[b]) + tb
            gap = np.linalg.norm(ca - cb, axis=1) - radii[a] - radii[b]
            distance = np.maximum(gap, 0.1)           # beyond the query bound: only a lower bound
            near = np.flatnonzero(gap < 0.1)
            if near.size:
                distance[near] = np.minimum(distance[near], pair_distances(a, b, near, Ra, ta, Rb, tb))
            ref_distance = np.inf
            for r in range(reference.shape[0]):
                rA = (np.eye(3), np.zeros(3)) if a < 0 else (ref_rot[r, a], ref_org[r, a])
                rB = (np.eye(3), np.zeros(3)) if b < 0 else (ref_rot[r, b], ref_org[r, b])
                ref_world = points[a] @ rA[0].T + rA[1]
                ref_distance = min(ref_distance, float(trees[b].query((ref_world - rB[1]) @ rB[0], k=1)[0].min()))
            touching = ref_distance < clearance
            worst = int(np.argmin(distance))
            flagged = (distance < clearance) & (not touching)
            colliding[flagged, a + 1] = colliding[flagged, b + 1] = True
            pairs.append({
                "a": names(a), "b": names(b), "owner_a": a, "owner_b": b,
                "min_distance": float(distance[worst]), "time": float(time[worst]),
                "reference_distance": ref_distance, "touching_at_reference": bool(touching),
                "violated": bool(flagged.any()),
            })

    lowest, lowest_link = np.full(n, np.inf), np.full(n, -1)
    for owner in owners:
        if owner < 0:
            continue
        R, t = pose(owner)
        z = np.einsum("nj,mj->nm", R[:, 2, :], points[owner]).min(axis=1) + t[:, 2]
        below = z < table_z + clearance
        colliding[below, owner + 1] = True
        better = z < lowest
        lowest[better], lowest_link[better] = z[better], owner
    worst = int(np.argmin(lowest))
    return {
        "pairs": sorted(pairs, key=lambda p: p["min_distance"]),
        "table": {
            "min_z": float(lowest[worst]), "time": float(time[worst]),
            "link": names(int(lowest_link[worst])), "table_z": table_z,
            "violated": bool(lowest[worst] < table_z + clearance),
        },
        "events": [
            {"time": float(time[k]), "links": [names(o - 1) for o in np.flatnonzero(colliding[k])]}
            for k in np.flatnonzero(colliding.any(axis=1))
        ],
        "clearance": clearance,
    }


def simplify_triangles(triangles: np.ndarray, cell: float) -> np.ndarray:
    """Vertex-clustering decimation: merge vertices within ``cell``-sized voxels.

    Drops collapsed and duplicate faces and keeps the winding of the rest.
    Only used for display; clearance checks sample the original meshes.
    """
    if cell <= 0 or triangles.size == 0:
        return triangles
    vertices = triangles.reshape(-1, 3)
    _, inverse = np.unique(np.floor(vertices / cell).astype(np.int64), axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    counts = np.bincount(inverse)
    centers = np.column_stack([np.bincount(inverse, vertices[:, d]) / counts for d in range(3)])
    faces = inverse.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return centers[faces[np.sort(first)]]


def stl_bytes(triangles: np.ndarray) -> bytes:
    """Binary STL with face normals."""
    triangles = np.asarray(triangles, dtype=np.float32)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    record = np.zeros(len(triangles), dtype=[("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    record["normal"], record["vertices"] = normals, triangles
    return b"\0" * 80 + struct.pack("<I", len(triangles)) + record.tobytes()
