#!/usr/bin/env python3
"""Generate a simple triangular-prism wedge for snow_ramp.

Entry edge is flush with ground (z=0) so lidar/Nav2 do not see a front wall.
Ignition/DART requires vertex normals on mesh collision — without them the
physics engine segfaults and spawn never completes.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Sequence, Tuple

L = 60.0  # length along +X (m) — trimmed to ~snow_tree_RR4 (world x≈80)
W = 110.0  # width along Y (m)
PITCH = 0.07  # rad (~4 deg)
H = L * math.tan(PITCH)
HALF_W = W / 2.0

# Origin at downhill entry center on ground.
VERTS: List[Tuple[float, float, float]] = [
    (0.0, -HALF_W, 0.0),  # 1 entry left
    (0.0, HALF_W, 0.0),  # 2 entry right
    (L, -HALF_W, 0.0),  # 3 exit left bottom
    (L, HALF_W, 0.0),  # 4 exit right bottom
    (L, -HALF_W, H),  # 5 exit left top
    (L, HALF_W, H),  # 6 exit right top
]

# CCW winding when viewed from outside the solid.
FACES_VISUAL: List[Tuple[int, int, int]] = [
    (1, 4, 3),
    (1, 2, 4),
    (1, 6, 2),
    (1, 5, 6),
    (3, 6, 5),
    (3, 4, 6),
    (1, 3, 5),
    (2, 6, 4),
]

# Closed solid with floor slightly above snow_ground (z=FLOOR_Z).
# Open-bottom collision let VelocityControl robots tunnel under the slope.
FLOOR_Z = 0.05
VERTS_COLLISION: List[Tuple[float, float, float]] = [
    (0.0, -HALF_W, FLOOR_Z),
    (0.0, HALF_W, FLOOR_Z),
    (L, -HALF_W, FLOOR_Z),
    (L, HALF_W, FLOOR_Z),
    (L, -HALF_W, H),
    (L, HALF_W, H),
]
FACES_COLLISION: List[Tuple[int, int, int]] = [
    (1, 6, 2),
    (1, 5, 6),
    (3, 6, 5),
    (3, 4, 6),
    (1, 3, 5),
    (2, 6, 4),
    (1, 2, 4),
    (1, 4, 3),
]


def _face_normal(
    verts: Sequence[Tuple[float, float, float]], i1: int, i2: int, i3: int
) -> Tuple[float, float, float]:
    v0 = verts[i1 - 1]
    v1 = verts[i2 - 1]
    v2 = verts[i3 - 1]
    ax, ay, az = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
    bx, by, bz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
    nx = ay * bz - az * by
    ny = az * bx - ax * bz
    nz = ax * by - ay * bx
    length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return (nx / length, ny / length, nz / length)


def write_obj(
    path: Path,
    verts: Sequence[Tuple[float, float, float]],
    faces: Sequence[Tuple[int, int, int]],
    name: str,
) -> None:
    normals = [_face_normal(verts, *face) for face in faces]
    lines = [
        f"# {name}",
        f"# L={L} W={W} pitch={PITCH} rad H={H:.6f}",
        "mtllib snow_ramp.mtl",
        f"o {name}",
        "usemtl snow",
    ]
    for v in verts:
        lines.append(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}")
    for n in normals:
        lines.append(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}")
    for idx, face in enumerate(faces, start=1):
        i, j, k = face
        lines.append(f"f {i}//{idx} {j}//{idx} {k}//{idx}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {path} faces={len(faces)}")


def main() -> None:
    out_dir = Path(__file__).resolve().parent
    mtl_path = out_dir / "snow_ramp.mtl"

    mtl_path.write_text(
        "# snow_ramp snow material\n"
        "newmtl snow\n"
        "Ka 0.90 0.92 0.95\n"
        "Kd 0.95 0.97 1.00\n"
        "Ks 0.15 0.15 0.15\n"
        "Ns 10\n"
        "d 1.0\n"
        "illum 2\n",
        encoding="utf-8",
    )

    write_obj(out_dir / "snow_ramp.obj", VERTS, FACES_VISUAL, "snow_ramp_wedge")
    write_obj(
        out_dir / "snow_ramp_collision.obj",
        VERTS_COLLISION,
        FACES_COLLISION,
        "snow_ramp_collision",
    )
    print(f"Wrote {mtl_path}")
    print(f"Vertices={len(VERTS)} H={H:.6f}")


if __name__ == "__main__":
    main()
