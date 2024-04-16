"""
This file creates a bigger mesh based on standard objects like Spheres, Cylinders etc.
Openscad expects [mm] as it seems.
"""

import math
import os
import subprocess
import time

import matplotlib.pyplot as plt
import numpy as np
import openpyscad as ops
import spatialgeometry as sg


def quaternion_to_euler_angle(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return [X, Y, Z]


def _create_sphere(t, q, radius):
    eulers = quaternion_to_euler_angle(*q)
    s = ops.Sphere(radius * 1000, center=[i * 1000 for i in t]).translate(
        [i * 1000 for i in t]
    )
    return s


def _create_cylinder(t, q, radius, length):
    eulers = quaternion_to_euler_angle(*q)
    c = (
        ops.Cylinder(length * 1000, radius * 1000, center=True)
        .rotate(eulers)
        .translate([i * 1000 for i in t])
    )
    return c


def create_mesh(sg_shapes):
    print("create mesh")
    u = ops.Union()

    for shape in sg_shapes:
        t = shape["t"]
        q = shape["q"]
        if shape["stype"] == "cylinder":
            length = shape["length"]
            radius = shape["radius"]
            cyl = _create_cylinder(t, q, radius, length)
            u.append(cyl)

        elif shape["stype"] == "sphere":
            radius = shape["radius"]
            s = _create_sphere(t, q, radius)
            u.append(s)
        else:
            raise Exception(f"stype {shape['stype']} is not implemented yet!")

    u.write("scad_files/mesh.scad")
    run_docker_cmd()


def run_docker_cmd():
    print("*** start running docker container")
    cmd = "docker run --rm -v /Users/robin/dev/robot-workspace-explore/scad_files/:/scad_files/  openscad/openscad:latest openscad -o /scad_files/output.stl /scad_files/mesh.scad"

    process = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
    )
    process.wait()
    print("*** finished docker command!")


if __name__ == "__main__":
    print("Testing stuff out here...")

    u = ops.Union()
    u.append(ops.Sphere(1000))

    u.append(ops.Cylinder(5000, 500, center=True))
    u.write("./scad_files/test_basic.scad")
