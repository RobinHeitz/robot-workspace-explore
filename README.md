# Using Python's Robotics Toolbox

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
