import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift

# create swift instance (visualization)
env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
# env.add(panda)
env.add(panda, robot_alpha=1.0, collision_alpha=0.5)
panda.q = panda.qr
# panda.qd = [0.1, 0, 0, 0, 0, 0, 0]

# Set goal pose
Tep = panda.fkine(panda.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.3) * sm.SE3.Tz(0.35)

axes = sg.Axes(length=0.1, base=Tep)
env.add(axes)

arrived = False
dt = 0.01

while not arrived:
    v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep, gain=0.1, threshold=0.01)
    J = panda.jacobe(panda.q)
    panda.qd = np.linalg.pinv(J) @ v
    env.step(dt)

env.hold()

