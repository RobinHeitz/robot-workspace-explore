import os
import time
from math import pi, sin

import pybullet as p
import pybullet_data


def controller(time_step):
    rest_pose = [0, -pi / 2, pi / 2, -pi / 2, pi / 2, pi]
    rest_pose[0] += 2 * pi * sin(time_step / 300)
    return rest_pose


def get_link_states(robot):

    p.getLinkState(
        robot,
        2,
    )


def main():

    print("Main")
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=2.5,
        cameraYaw=0,
        cameraPitch=-40,
        cameraTargetPosition=[0, -0.4, 0.2],
    )
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(False)
    time_step = 1 / 240.0
    p.setTimeStep(time_step)

    plane = p.loadURDF("urdf/plane/plane.urdf")

    robot = p.loadURDF(
        "urdf/ur10.urdf",
        [0, 0, 0],
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True,
    )

    col_obj = p.loadURDF(
        "urdf/cube.urdf",
        [0.3, 0.3, 0.73],
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True,
        globalScaling=1.0,
    )

    # p.loadURDF(
    #     "urdf/cube.urdf",
    #     [0, 0, 0],
    #     p.getQuaternionFromEuler([0, 0, 0]),
    #     useFixedBase=True,
    #     globalScaling=10.0,
    # )
    # Set initial pos of robot
    rest_pose = [0, -pi / 2, pi / 2, -pi / 2, pi / 2, pi]

    for joint, angle in enumerate(rest_pose):
        p.resetJointState(robot, joint, angle)

    link_state = p.getLinkState(robot, 2)

    p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=1.4,
    )

    collisionShapeId = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName="duck_vhacd.obj",
    )

    for i in range(400):
        new_joint_state = controller(i)
        for joint, angle in enumerate(new_joint_state):
            p.resetJointState(robot, joint, angle)
        p.stepSimulation()
        time.sleep(time_step)

    p.disconnect()


if __name__ == "__main__":
    main()
