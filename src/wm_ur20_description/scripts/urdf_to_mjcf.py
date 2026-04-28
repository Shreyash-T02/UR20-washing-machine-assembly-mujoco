#!/usr/bin/env python3
"""Convert the patched UR20 URDF to MuJoCo MJCF."""
import os
import mujoco

os.chdir(os.path.expanduser("~/AME_504/wm_assembly_ws/src/wm_ur20_description"))
m = mujoco.MjModel.from_xml_path("ur20_flat.urdf")
out = "mjcf/ur20_converted.xml"
mujoco.mj_saveLastXML(out, m)
print(f"Saved: {out}  ({m.njnt} joints, {m.nbody} bodies)")
