from urchin import URDF
import pyrender

robot = URDF.load('tests/data/ur5/ur5.urdf') # UR5 ist zufällig auch das Demo Modell von urchin

joint_angles = [0, 0, 1, 0, 0, 0]

fk = robot.visual_trimesh_fk(joint_angles) # Computes the poses of the URDF's visual trimeshes using fk.

# An diesem Punkt haben wir die Daten so, dass wir in  https://github.com/mikedh/trimesh weiterarbeiten könnten.

# Die trimesh Daten visualisiert in pyrender
scene = pyrender.Scene()
for tm in fk:
    pose = fk[tm]
    mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
    scene.add(mesh, pose=pose)
pyrender.Viewer(scene, use_raymond_lighting=True)