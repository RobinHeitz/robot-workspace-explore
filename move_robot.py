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


def add_collision_shape(robot, env):
    print("### add collision shape")
    q0 = robot.q[0]
    link0 = robot.links[0]
    a0 = link0.A(q0)
    c0 = sg.Cylinder()

    s0 = sg.Sphere(radius=0.08, base=a0, color=[1, 0, 1, 0.4])
    env.add(s0)

    q1 = robot.q[1]
    link1 = robot.links[1]
    a1 = link1.A(q1)
    s1 = sg.Sphere(radius=0.08, base=a1, color=[1, 0, 1, 0.4])
    env.add(s1)

    q2 = robot.q[2]
    link2 = robot.links[2]
    a2 = link2.A(q2)
    # print(a0)
    # print(a1)
    print(a1 * a2)
    s2 = sg.Sphere(radius=0.08, base=a2 * a1, color=[1, 0, 1, 0.4])
    env.add(s2)


def get_mesh_path(file):

    return os.path.join(
        os.getcwd(),
        "./robotics-toolbox-python/rtb-data/rtbdata/xacro/franka_description/meshes/visual/",
        file,
    )


def create_collision_shapes2(q, env, robot):
    """Create collision shape for one joint configuration q"""
    print("create_collision_shapes")

    T_prev = robot.base
    for i, link in enumerate(robot.links):
        if i == 5:
            break

        q = robot.q[i]
        a = link.A(q)
        T_cur = T_prev * a

        axes = sg.Axes(length=0.25, pose=T_cur)
        env.add(axes)

        T_prev = T_cur

        print("### Link index ", i, " joint for index: ", q)
        print(T_cur)

    s = sg.Sphere(radius=0.2, pose=sm.SE3(), color=[1, 0, 1, 0.4])
    env.add(s)


def create_collision_shapes(q, env, robot):
    """Create collision shape for one joint configuration q"""
    print("create_collision_shapes")

    for i, link in enumerate(robot.links):
        if i == 7:
            break

        T = robot.fkine(q, link, robot.links[0])
        if i in [0, 1]:
            axes = sg.Axes(length=0.25, pose=T)
            env.add(axes)

        print("### Link index ", i)
        print(T)

    s = sg.Sphere(radius=0.2, pose=sm.SE3(), color=[1, 0, 1, 0.4])
    env.add(s)


def main():
    """Moves robot and stores the trajectory."""

    col = [1, 0, 1]
    env = swift.Swift()
    env.launch(realtime=True)

    panda = rtb.models.Panda()
    Tbase = sm.SE3.Tx(-0.3) * sm.SE3.Ty(-0.3)
    panda.base = Tbase
    panda.q = panda.qr
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
