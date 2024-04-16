"""
This file creates a bigger mesh based on standard objects like Spheres, Cylinders etc.
"""

import matplotlib.pyplot as plt
import numpy as np
import openpyscad as ops

c1 = ops.Cube([10, 20, 10])
s = ops.Sphere(10)

(c1 + s).write("scad_files/test.scad")
