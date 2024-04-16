# Using Python's Robotics Toolbox

This repo uses different tools to visualize a robot. The robot is modeled with simple shapes like `spheres` and `cylinders`.
Based on the trajectory of the robot, these simple shapes are rendered within the visualization tool (`swift`).

`Openscad` is used to create a a single .stl file based on these overlapping shapes.
Therefore, first a `.scad` code file is created by a python library called `openpyscad`.
To generate an .stl file, it needs to be executed within the command line (if openscad is installed on the machine locally).

In my example, we use a docker container which executed the `.scad` code for us and saves the output as .stl file.

An example docker command looks like this:

```
docker run --rm -v ./scad_files:/input_files -v ./scad_output:/output_files openscad/openscad:latest openscad -o /output_files/test_output.stl /input_files/test.scad
```

## Setup

Create a new python env & activate

```
python3 -m venv env
source env/bin/activate
```

Clone & install `roboticstoolbox-python`

```
git clone https://github.com/petercorke/robotics-toolbox-python.git
cd robotics-toolbox-python
```

Fix import error of scipy according to [this issue](https://github.com/petercorke/robotics-toolbox-python/issues/412)
Then, install it:

```
pip3 install -e .
```

Run `move_robot.py`, visualization should be shown in new browser tab
