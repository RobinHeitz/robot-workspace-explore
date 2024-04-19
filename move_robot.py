"""
This module should demonstrate how to 'sense' collision objects. The goal is that the user uses free-drive to move 
along the part where no collisions are to be expected. Based on the geometry of the robot, the 'useable' workspace 
can be extracted. The inverse of that should approximate the collision object.

Basic shapes (cylinder & sphere) model the robot arm and are visualized with swift. 
openpyscad uses these standard geometries to create a .scad file, which is a desgin specific language for the 
script-based CAD software Openscad. This description of an 3d-object is then executed inside a docker container
(with openscad installed). The output is a .stl file at ./scad_files.
"""

import argparse
import os
from datetime import datetime

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift

from controllers.controller_proto import ControllerProtocol
from controllers.fake_controller import FakeController
from controllers.ur_controller import URController
from mesh_generation import create_mesh


def get_colors(alpha=0.4, unique=True):
    if not unique:
        def_col = [1, 1, 1, alpha]
        return [def_col.copy() for _ in range(8)]

    return [
        [1, 0, 1, alpha],
        [1, 1, 0, alpha],
        [0, 1, 1, alpha],
        [0.2, 0.8, 0, alpha],
        [0.2, 0, 0.8, alpha],
        [0, 0.5, 0.8, alpha],
        [0.4, 0.5, 0.8, alpha],
    ]


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
    shapes = list()
    world = robot.links[0]

    # Base Link
    link = robot.links[1]
    T = robot.fkine(q, link, world)
    l = 0.16
    c = sg.Cylinder(0.06, l, pose=T * sm.SE3.Tz(l / 2), color=cols[0])
    shapes.append(c.to_dict())
    env.add(c)

    # Shoulder Link
    link = robot.links[2]
    T = robot.fkine(q, link, world)
    l = 0.21
    c = sg.Cylinder(
        0.07, l, pose=T * sm.SE3.Rx(-np.pi / 2) * sm.SE3.Tz(l / 2), color=cols[1]
    )
    shapes.append(c.to_dict())
    env.add(c)

    # Upper Arm Link
    link = robot.links[3]
    T = robot.fkine(q, link, world)
    l = 0.50
    c = sg.Cylinder(0.07, l, pose=T * sm.SE3.Tz(l / 2), color=cols[2])
    shapes.append(c.to_dict())
    env.add(c)

    # Forearm Link
    link = robot.links[4]
    T = robot.fkine(q, link, world)
    l = 0.50
    c = sg.Cylinder(0.05, l, pose=T * sm.SE3.Tz(l / 2), color=cols[3])
    shapes.append(c.to_dict())
    env.add(c)
    s = sg.Sphere(0.07, pose=T * sm.SE3.Ty(0.04), color=cols[3])
    env.add(s)
    shapes.append(s.to_dict())

    # Wrist 1
    link = robot.links[5]
    T = robot.fkine(q, link, world)
    l = 0.20
    c = sg.Cylinder(
        0.04, l, pose=T * sm.SE3.Rx(-np.pi / 2) * sm.SE3.Tz(l / 2 - 0.06), color=cols[4]
    )
    env.add(c)
    shapes.append(c.to_dict())

    # Wrist 2
    link = robot.links[6]
    T = robot.fkine(q, link, world)
    l = 0.20
    c = sg.Cylinder(0.04, l, pose=T * sm.SE3.Tz(l / 2 - 0.06), color=cols[5])
    env.add(c)
    shapes.append(c.to_dict())

    # Wrist 3
    link = robot.links[7]
    T = robot.fkine(q, link, world)
    l = 0.15
    c = sg.Cylinder(
        0.04, l, pose=T * sm.SE3.Rx(np.pi / 2) * sm.SE3.Tz(-0.01), color=cols[6]
    )
    env.add(c)
    shapes.append(c.to_dict())

    return shapes


def main(controller: ControllerProtocol, duration, stl, *args, **kwargs):
    """
    Moves robot and stores the trajectory.

    Params:
    - controller: Controller which provides joint states and can buffer the trajectory.
    - duration (float): Duration how long the joint capturing should last.
    - stl (bool): If set, a .stl file is created in the end and saved at ./scad_files/
    """
    print(f"main(): {duration=} | {stl=}")

    # Add robot to environment 'swift'
    env = swift.Swift()
    env.launch(realtime=True)

    ur = rtb.models.UR5()
    ur.q = controller.get_joint_angles()
    # ur.q = controller.q_start
    BaseUR = sm.SE3.Tx(0.3) * sm.SE3.Ty(0.3)
    ur.base = BaseUR
    env.add(ur, robot_alpha=0.8, collision_alpha=0)

    start = datetime.now()
    dt = 0.01

    controller.start_buffering()
    while True:
        if (datetime.now() - start).total_seconds() > duration:
            break
        ur.q = controller.get_joint_angles()
        env.step(dt)

    controller.stop_buffering()
    draw_axes(ur.q, env, ur)
    trajectories = controller.get_buffer()

    only_last_pose = kwargs.get("once", False)
    shapes = []

    if only_last_pose:
        shapes_last_pose = create_collision_shapes_ur5(trajectories[-1], env, ur)
        shapes.extend(shapes_last_pose)

    else:
        print(
            f"*** Start drawing collision shapes for {len(trajectories)} trajectories"
        )
        for q in trajectories:
            shapes_cur_pose = create_collision_shapes_ur5(q, env, ur)
            shapes.extend(shapes_cur_pose)

    if stl:
        create_mesh(shapes)
    env.hold()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f",
        "--fake-control",
        action="store_true",
        help="Use fake controller for joint trajectory generation.",
    )

    parser.add_argument(
        "-s",
        "--stl",
        action="store_true",
        default=False,
        help="If set, generates .stl file with the collision shapes seen in the visualization tool.",
    )

    parser.add_argument(
        "-r",
        "--rate",
        type=int,
        default=10,
        help="Sample frequency of the buffer in Hz. 20 means the delay between each joint configuration is 1/20 seconds",
    )

    parser.add_argument(
        "-a",
        "--alpha",
        type=float,
        default=1.0,
        help="Alpha value from the collision shapes",
    )

    parser.add_argument(
        "-c", "--color", action="store_true", help="Use different colors per link"
    )

    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=2.0,
        help="Duration of simulation [seconds]",
    )

    parser.add_argument(
        "-o",
        "--once",
        action="store_true",
        help="Only visualize last pose of robot instead of complete trajectory.",
    )

    args = parser.parse_args()
    kwargs = vars(args)

    cols = get_colors(alpha=kwargs["alpha"], unique=kwargs["color"])

    # Load controller
    fake_controller = kwargs.pop("fake_control")
    if fake_controller:
        controller = FakeController(sample_frequency_hz=kwargs.get("rate", 10))
        # controller2 = FakeController2(sample_frequency_hz=kwargs.get("rate", 10))
    else:
        controller = URController()

    main(controller, **kwargs)
    print("### Finished main()")
