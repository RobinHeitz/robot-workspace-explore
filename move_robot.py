import os

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift

# create swift instance (visualization)


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


col = [1, 0, 1]
env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
# env.add(panda)
env.add(panda, robot_alpha=1.0, collision_alpha=0.2)
panda.q = panda.qr
# panda.qd = [0.1, 0, 0, 0, 0, 0, 0]

# Messing around with sphere etc.
lTep = (
    sm.SE3.Tx(0.45)
    * sm.SE3.Ty(0.25)
    * sm.SE3.Tz(0.3)
    * sm.SE3.Rx(np.pi)
    * sm.SE3.Rz(np.pi / 2)
)

l_target = sg.Sphere(0.02, color=[0.2, 0.4, 0.65, 1.0], pose=lTep)
l_target_frame = sg.Axes(0.1, base=lTep)
env.add(l_target)
env.add(l_target_frame)

# Set goal pose
Tep = panda.fkine(panda.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.3) * sm.SE3.Tz(0.35)

axes = sg.Axes(length=0.1, base=Tep)
env.add(axes)

arrived = False
dt = 0.01

# env.hold()


while not arrived:
    v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep, gain=8.1, threshold=0.01)
    J = panda.jacobe(panda.q)
    panda.qd = np.linalg.pinv(J) @ v
    env.step(dt)

add_collision_shape(panda, env)

env.hold()
