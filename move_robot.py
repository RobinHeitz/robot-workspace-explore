"""
This module should demonstrate how to 'sense' collision objects. The goal is that the user uses free-drive to move along the part where no collisions are to be expected.
Based on the geometry of the robot, the 'useable' workspace can be extracted. The inverse of that should approximate the collision object.
"""

import os
from datetime import datetime

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift

col1 = [1, 0, 1, 0.4]
col2 = [1, 1, 0, 0.4]
col3 = [0, 1, 1, 0.4]


def get_mesh_path(file):

    return os.path.join(
        os.getcwd(),
        "./robotics-toolbox-python/rtb-data/rtbdata/xacro/franka_description/meshes/visual/",
        file,
    )


def create_collision_shapes(q, env, robot):
    """Create collision shape for one joint configuration q"""

    # Link 0
    l0 = 0.3
    c0 = sg.Cylinder(0.07, l0, pose=robot.base * sm.SE3.Tz(l0 / 2), color=col1)
    env.add(c0)
    c = sg.Sphere(0.09, pose=robot.base * sm.SE3.Tx(-0.1) * sm.SE3.Tz(0.05), color=col1)
    env.add(c)

    # Link 1
    l1 = 0.3
    T1 = robot.fkine(q, robot.links[1], robot.links[0])
    a = sg.Axes(length=0.3, pose=T1)
    # env.add(a)
    s = sg.Sphere(0.07, pose=T1, color=col2)
    env.add(s)
    s = sg.Sphere(0.07, pose=T1 * sm.SE3.Ty(0.07), color=col2)
    env.add(s)
    s = sg.Sphere(0.07, pose=T1 * sm.SE3.Ty(-0.07), color=col2)
    env.add(s)

    # Link 2
    l2 = 0.3
    T2 = robot.fkine(q, robot.links[2], robot.links[0])
    a = sg.Axes(length=0.3, pose=T2)
    env.add(a)
    c = sg.Cylinder(
        0.07, l2, pose=T2 * sm.SE3.Rx(np.pi / 2) * sm.SE3.Tz(l2 / 2), color=col3
    )
    env.add(c)

    # link 3
    l3 = 0.3
    T3 = robot.fkine(q, robot.links[3], robot.links[0])
    a = sg.Axes(length=0.3, pose=T3)
    env.add(a)
    s1 = sg.Sphere(0.07, pose=T3 * sm.SE3.Ty(0.07) * sm.SE3.Tx(0.07), color=col1)
    s2 = sg.Sphere(0.07, pose=T3 * sm.SE3.Ty(-0.07) * sm.SE3.Tx(0.07), color=col1)
    s3 = sg.Sphere(0.07, pose=T3 * sm.SE3.Tx(0.07), color=col1)
    env.add(s1)
    env.add(s2)
    env.add(s3)

    # link 4
    l4 = 0.44
    T4 = robot.fkine(q, robot.links[4], robot.links[0])
    a = sg.Axes(length=0.3, pose=T4)
    # env.add(a)
    c = sg.Cylinder(
        0.09,
        l4,
        pose=T4
        * sm.SE3.Rx(np.pi / 2)
        * sm.SE3.Tx(-0.09)
        * sm.SE3.Tz(-l4 / 2)
        * sm.SE3.Ty(-0.02),
        color=col2,
    )
    env.add(c)

    # Link 5
    T5 = robot.fkine(q, robot.links[5], robot.links[0])
    a = sg.Axes(length=0.3, pose=T5)
    # env.add(a)
    s = sg.Sphere(0.07, pose=T5, color=col3)
    env.add(s)

    s = sg.Sphere(0.07, pose=T5 * sm.SE3.Ty(0.1), color=col3)
    env.add(s)

    # Link 6
    l6 = 0.35
    T6 = robot.fkine(q, robot.links[6], robot.links[0])
    a = sg.Axes(length=0.3, pose=T6)
    env.add(a)
    c = sg.Cylinder(
        0.07,
        l6,
        pose=T6 * sm.SE3.Rx(np.pi / 2) * sm.SE3.Tz(l6 / 2 - 0.1) * sm.SE3.Tx(0.09),
        color=col1,
    )
    env.add(c)


def main():
    """Moves robot and stores the trajectory."""

    col = [1, 0, 1]
    env = swift.Swift()
    env.launch(realtime=True)

    panda = rtb.models.Panda()
    Tbase = sm.SE3.Tx(-0.3) * sm.SE3.Ty(-0.3)
    panda.base = Tbase
    # panda.q = panda.qr
    env.add(panda, robot_alpha=1.0, collision_alpha=0.2)

    # use_traj = True
    mode = "None"

    if mode == "tra":
        dt = 0.1
        traj = rtb.tools.trajectory.jtraj(panda.qz, panda.qr, 20)

        for q in traj.q:
            panda.q = q
            env.step(dt)

        create_collision_shapes(traj.q[-1], env, panda)
        env.hold()

    elif mode == "gain":
        Tep = panda.fkine(panda.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.3) * sm.SE3.Tz(0.35)

        arrived = False
        dt = 0.01

        while not arrived:
            v, arrived = rtb.p_servo(
                panda.fkine(panda.q), Tep, gain=3.0, threshold=0.01
            )
            J = panda.jacobe(panda.q)
            panda.qd = np.linalg.pinv(J) @ v
            env.step(dt)

        print("arrived")
        create_collision_shapes(panda.q, env, panda)
        env.hold()

    else:
        create_collision_shapes(panda.q, env, panda)
        env.hold()


if __name__ == "__main__":
    main()
