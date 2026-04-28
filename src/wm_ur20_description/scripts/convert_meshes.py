#!/usr/bin/env python3
"""Convert UR20 visual meshes from DAE to OBJ and copy collision STLs."""
import glob
import os
import shutil
import trimesh

UR_MESH_BASE = os.path.expanduser(
    "~/Downloads/Universal_Robots_ROS2_Description-humble/meshes/ur20"
)
OUT_BASE = os.path.expanduser(
    "~/AME_504/wm_assembly_ws/src/wm_ur20_description/meshes"
)

os.makedirs(f"{OUT_BASE}/visual", exist_ok=True)
os.makedirs(f"{OUT_BASE}/collision", exist_ok=True)

converted = 0
for dae_path in glob.glob(f"{UR_MESH_BASE}/visual/*.dae"):
    name = os.path.splitext(os.path.basename(dae_path))[0]
    obj_path = f"{OUT_BASE}/visual/{name}.obj"
    mesh = trimesh.load(dae_path, force="mesh")
    # Merge coincident vertices so adjacent faces share them →
    # vertex_normals becomes an angle-weighted average → smooth shading in MuJoCo
    mesh.merge_vertices()
    mesh.fix_normals()
    # Explicitly request normals in the OBJ (trimesh defaults to None = omit)
    obj_data = trimesh.exchange.obj.export_obj(mesh, include_normals=True)
    with open(obj_path, "wb" if isinstance(obj_data, bytes) else "w") as f:
        f.write(obj_data)
    vn_count = len(mesh.vertex_normals)
    print(f"  {name}.dae -> {name}.obj  ({len(mesh.vertices)} verts, {vn_count} normals)")
    converted += 1

print(f"\nConverted {converted} visual meshes.")

copied = 0
for stl_path in glob.glob(f"{UR_MESH_BASE}/collision/*.stl"):
    dest = f"{OUT_BASE}/collision/{os.path.basename(stl_path)}"
    shutil.copy2(stl_path, dest)
    copied += 1

print(f"Copied {copied} collision STL files.")
