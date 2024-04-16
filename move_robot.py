"""
This module should demonstrate how to 'sense' collision objects. The goal is that the user uses free-drive to move along the part where no collisions are to be expected.
Based on the geometry of the robot, the 'useable' workspace can be extracted. The inverse of that should approximate the collision object.
"""

import argparse
import os
from datetime import datetime

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift

from fake_controller import FakeController
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


def main(controller, duration, *args, **kwargs):
    """Moves robot and stores the trajectory."""

    # Add robot to environment 'swift'
    env = swift.Swift()
    env.launch(realtime=True)

    ur = rtb.models.UR5()
    ur.q = controller.q_start
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
    if only_last_pose:
        shapes = create_collision_shapes_ur5(trajectories[-1], env, ur)
    else:
        print(
            f"*** Start drawing collision shapes for {len(trajectories)} trajectories"
        )
        for q in trajectories:
            create_collision_shapes_ur5(q, env, ur)

    create_mesh(shapes)
    env.hold()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f",
        "--fake-controller",
        action="store_true",
        help="Use fake controller for joint trajectory generation.",
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
    fake_controller = kwargs.pop("fake_controller")
    if fake_controller:
        controller = FakeController(buffer_duration=0.2)
    else:
        print("There is no real controller yet! Use -f")
        exit()

    main(controller, **kwargs)
