import os

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift

# create swift instance (visualization)


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
env.add(panda, robot_alpha=0.0, collision_alpha=0.5)
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

print(panda.links[0])
print(panda.links[1])
print(panda.links[2])
print(panda.links[3])
print(panda.links[4])
print(panda.links[5])

# env.hold()


while not arrived:
    v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep, gain=8.1, threshold=0.01)
    J = panda.jacobe(panda.q)
    panda.qd = np.linalg.pinv(J) @ v
    env.step(dt)

print(panda.q)

# link0 = sg.Cylinder(radius=0.06, length=0.03, pose=T0, color=col)


# link0 = sg.Mesh(
#     get_mesh_path("hand.dae"),
#     base=lTep,
#     color=[0, 1, 0.4],
# )

env.hold()
