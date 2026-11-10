#!/usr/bin/env python3
"""Generate collision/visual STL meshes for the DH PGEA-50-26 gripper.

Official STEP/STL CAD is behind DH-Robotics' download form
(https://en.dh-robotics.com/product/pgea). These boxes match the published
envelope 118 x 55 x 29 mm and the CS3AS tool offset (x=0, y=0, z=118 mm).
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

Vertex = Tuple[float, float, float]


def _normal(a: Vertex, b: Vertex, c: Vertex) -> Vertex:
  ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
  vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
  nx = uy * vz - uz * vy
  ny = uz * vx - ux * vz
  nz = ux * vy - uy * vx
  n = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
  return (nx / n, ny / n, nz / n)


def _box(center: Sequence[float], size: Sequence[float]) -> List[Tuple[Vertex, Vertex, Vertex]]:
  cx, cy, cz = center
  sx, sy, sz = (0.5 * size[0], 0.5 * size[1], 0.5 * size[2])
  p = {
    "nxl": (cx - sx, cy - sy, cz - sz),
    "nxh": (cx - sx, cy - sy, cz + sz),
    "nhl": (cx - sx, cy + sy, cz - sz),
    "nhh": (cx - sx, cy + sy, cz + sz),
    "pxl": (cx + sx, cy - sy, cz - sz),
    "pxh": (cx + sx, cy - sy, cz + sz),
    "phl": (cx + sx, cy + sy, cz - sz),
    "phh": (cx + sx, cy + sy, cz + sz),
  }
  faces = [
    (p["pxl"], p["phl"], p["phh"], p["pxh"]),  # +X
    (p["nhl"], p["nxl"], p["nxh"], p["nhh"]),  # -X
    (p["nhl"], p["nhh"], p["phh"], p["phl"]),  # +Y
    (p["nxl"], p["pxl"], p["pxh"], p["nxh"]),  # -Y
    (p["nxh"], p["pxh"], p["phh"], p["nhh"]),  # +Z
    (p["nxl"], p["nhl"], p["phl"], p["pxl"]),  # -Z
  ]
  tris = []
  for a, b, c, d in faces:
    tris.append((a, b, c))
    tris.append((a, c, d))
  return tris


def write_ascii_stl(path: Path, name: str, triangles: Iterable[Tuple[Vertex, Vertex, Vertex]]) -> None:
  lines = [f"solid {name}"]
  for a, b, c in triangles:
    nx, ny, nz = _normal(a, b, c)
    lines.append(f"  facet normal {nx:.6e} {ny:.6e} {nz:.6e}")
    lines.append("    outer loop")
    for v in (a, b, c):
      lines.append(f"      vertex {v[0]:.6e} {v[1]:.6e} {v[2]:.6e}")
    lines.append("    endloop")
    lines.append("  endfacet")
  lines.append(f"endsolid {name}\n")
  path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
  here = Path(__file__).resolve().parent
  mesh_dir = here / "meshes"
  mesh_dir.mkdir(exist_ok=True)

  # URDF units are metres. Origin is the CS3AS flange / gripper mount face.
  # Body occupies most of the 118 mm length; fingertips sit near z=118 mm.
  body = _box(center=(0.0, 0.0, 0.050), size=(0.055, 0.029, 0.100))
  finger = _box(center=(0.0, 0.0, 0.009), size=(0.012, 0.010, 0.018))

  write_ascii_stl(mesh_dir / "body.stl", "pgea_50_26_body", body)
  write_ascii_stl(mesh_dir / "finger.stl", "pgea_50_26_finger", finger)
  print(f"wrote {mesh_dir / 'body.stl'}")
  print(f"wrote {mesh_dir / 'finger.stl'}")


if __name__ == "__main__":
  main()
