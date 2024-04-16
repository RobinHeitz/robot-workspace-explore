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

from fake_controller import FakeController

cols = [
    [1, 0, 1, 0.4],
    [1, 1, 0, 0.4],
    [0, 1, 1, 0.4],
    [0.2, 0.8, 0, 0.4],
    [0.2, 0, 0.8, 0.4],
    [0, 0.5, 0.8, 0.4],
    [0.4, 0.5, 0.8, 0.4],
]


def get_mesh_path(file):

    return os.path.join(
        os.getcwd(),
        "./robotics-toolbox-python/rtb-data/rtbdata/xacro/franka_description/meshes/visual/",
        file,
    )


def draw_axes(q, env, robot):
    """Draw joint axes for current joint configuration"""
    for i in range(1, 9):
        world = robot.links[0]
        link = robot.links[i]
        T = robot.fkine(q, link, world)
        a = sg.Axes(length=0.3, pose=T)
        env.add(a)


def create_collision_shapes_ur5(q, env, robot):
    """Create collision shapes for UR5"""
    world = robot.links[0]

    # Base Link
    link = robot.links[1]
    T = robot.fkine(q, link, world)
    l = 0.16
    c = sg.Cylinder(0.06, l, pose=T * sm.SE3.Tz(l / 2), color=cols[0])
    env.add(c)

    # Shoulder Link
    link = robot.links[2]
    T = robot.fkine(q, link, world)
    l = 0.21
    c = sg.Cylinder(
        0.07, l, pose=T * sm.SE3.Rx(-np.pi / 2) * sm.SE3.Tz(l / 2), color=cols[1]
    )
    env.add(c)

    # Upper Arm Link
    link = robot.links[3]
    T = robot.fkine(q, link, world)
    l = 0.50
    c = sg.Cylinder(0.07, l, pose=T * sm.SE3.Tz(l / 2), color=cols[2])
    env.add(c)

    # Forearm Link
    link = robot.links[4]
    T = robot.fkine(q, link, world)
    l = 0.50
    c = sg.Cylinder(0.05, l, pose=T * sm.SE3.Tz(l / 2), color=cols[3])
    env.add(c)
    s = sg.Sphere(0.07, pose=T * sm.SE3.Ty(0.04), color=cols[3])
    env.add(s)

    # Wrist 1
    link = robot.links[5]
    T = robot.fkine(q, link, world)
    l = 0.20
    c = sg.Cylinder(
        0.04, l, pose=T * sm.SE3.Rx(-np.pi / 2) * sm.SE3.Tz(l / 2 - 0.06), color=cols[4]
    )
    env.add(c)

    # Wrist 2
    link = robot.links[6]
    T = robot.fkine(q, link, world)
    l = 0.20
    c = sg.Cylinder(0.04, l, pose=T * sm.SE3.Tz(l / 2 - 0.06), color=cols[5])
    env.add(c)

    # Wrist 3
    link = robot.links[7]
    T = robot.fkine(q, link, world)
    l = 0.15
    c = sg.Cylinder(
        0.04, l, pose=T * sm.SE3.Rx(np.pi / 2) * sm.SE3.Tz(-0.01), color=cols[6]
    )
    env.add(c)


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

    env = swift.Swift()
    env.launch(realtime=True)

    panda = rtb.models.Panda()
    Tbase = sm.SE3.Tx(-0.3) * sm.SE3.Ty(-0.3)
    panda.base = Tbase
    # panda.q = panda.qr
    env.add(panda, robot_alpha=1.0, collision_alpha=0.2)

    # UR5
    controller = FakeController()

    ur = rtb.models.UR5()
    ur.q = controller.q_start
    BaseUR = sm.SE3.Tx(0.3) * sm.SE3.Ty(0.3)
    ur.base = BaseUR
    env.add(ur, robot_alpha=0.8, collision_alpha=0)

    # use_traj = True
    mode = "ur"

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

    elif mode == "ur":
        start = datetime.now()
        dt = 0.05
        while True:
            if (datetime.now() - start).total_seconds() > 2.7:
                break
            ur.q = controller.get_joint_angles()
            env.step(dt)

        draw_axes(ur.q, env, ur)
        create_collision_shapes_ur5(ur.q, env, ur)
        env.hold()

    else:
        create_collision_shapes(panda.q, env, panda)
        env.hold()


if __name__ == "__main__":
    main()
