#!/usr/bin/env python3
"""Validate UR20 actuator gains: hold a stressed config, check steady-state error."""
import os
import numpy as np
import mujoco

os.chdir(os.path.expanduser("~/AME_504/wm_assembly_ws/src/wm_ur20_description"))
m = mujoco.MjModel.from_xml_path("mjcf/ur20.xml")
d = mujoco.MjData(m)

# Stressed config: arm extended horizontally (worst gravity torque on shoulder)
target = np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])

mujoco.mj_resetData(m, d)
d.qpos[:] = target
d.ctrl[:] = target  # position actuators: ctrl = target angle

# Simulate for 5 seconds (2500 steps at 0.002 s/step)
for _ in range(2500):
    mujoco.mj_step(m, d)

error = np.abs(d.qpos - target)
print("Stressed config gain check (5 s simulation):")
joint_names = [m.joint(i).name for i in range(m.njnt)]
for name, err in zip(joint_names, error):
    status = "OK" if err < 0.02 else "FAIL"
    print(f"  {name:<28} error={err:.5f} rad  [{status}]")

max_err = np.max(error)
max_vel = np.max(np.abs(d.qvel))
print(f"\nMax position error : {max_err:.5f} rad  ({'PASS' if max_err < 0.02 else 'FAIL'} threshold 0.02 rad)")
print(f"Max joint velocity : {max_vel:.6f} rad/s  ({'PASS' if max_vel < 0.01 else 'FAIL'} threshold 0.01 rad/s)")
