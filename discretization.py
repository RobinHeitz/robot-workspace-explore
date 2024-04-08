import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift

RES = 0.1


class DiscretePoint:
    def __init__(self, x, y, z, edge_size):
        """Init DiscretePoint.

        - x: Position in space
        - y: Position in space
        - z: Position in space
        - edge_size: Edge size of cuboid
        """
        self.x = x
        self.y = y
        self.z = z
        self.edge_size = edge_size

    def size(self):
        return [self.x, self.y, self.z]

    def __repr__(self):
        return f"DP: [{self.x}|{self.y}|{self.z}]"

    def __str__(self):
        return self.__repr__()


def discretize_space(size, edge_size=0.1) -> list[DiscretePoint]:
    """Returns a list of points representing a discretized space.

    Args:
    - size: List with components [x, y, z] representing the space to discretize
    -edge_size: Initial edge size of cubes.
    """

    num_x = int(size[0] / edge_size)
    num_y = int(size[1] / edge_size)
    num_z = int(size[2] / edge_size)

    points = []

    for x in range(num_x):
        for y in range(num_y):
            for z in range(num_z):
                points.append(
                    DiscretePoint(
                        x * edge_size, y * edge_size, z * edge_size, edge_size
                    )
                )
    return points


def add_obstacles(env, points):
    obstacles = []
    for p in points:
        cuboid = sg.Cuboid(p.size())
        obstacles.append(cuboid)
        env.add(cuboid)
    return obstacles


def main():
    points = discretize_space([1, 1, 1], edge_size=0.2)

    # Create env
    env = swift.Swift()
    env.launch(realtime=True)

    # Create robot
    robot = rtb.models.Panda()
    robot.q = robot.qr
    env.add(robot, robot_alpha=0.2, collision_alpha=1.0)

    sg.Sphere(0.1,base=None, collision=True)


    sphere = sg.Sphere(0.1, sm.SE3(1, 0, 0))
    env.add(sphere, alpha=0.1)

    # obstacles = add_obstacles(env, points)
    #
    # for i, obst in enumerate(obstacles):
    #     collides = robot.iscollided(robot.q, obst)
    #     print("Does the robot collides: ", collides, i)

    # print("Does the robot collides: ", iscollided)
    #
    # print("robot: num links", robot.nlinks)
    # print("Robot has collision:", robot.hascollision)

    # fkine_all = robot.fkine_all(robot.q)
    # print(fkine_all)

    env.hold()


if __name__ == "__main__":
    main()
