"""Imitates joint angles that are obtained from free-drive-mode of a real robot."""

import threading
import time
from datetime import datetime

import numpy as np


class FakeController:

    q_start = [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]

    def __init__(self, freq=4):
        self._q = self.q_start.copy()
        self.freq = freq

        self.start_time = datetime.now()

        self.thread = threading.Thread(target=self._alter_joints)
        self.thread.daemon = True
        self.thread.start()

    def _alter_joints(self):
        dt = 0.01
        while True:
            total_secs = (datetime.now() - self.start_time).total_seconds()

            # Joint 0
            angle = np.pi / self.freq * total_secs
            angle = angle % (2 * np.pi)
            angle -= np.pi
            self._q[0] = angle

            # Joint 2
            deviation = 0.4 * np.sin(np.pi / self.freq * total_secs)
            self._q[2] = self.q_start[2] + deviation

            time.sleep(dt)

    def get_joint_angles(self):
        return self._q


if __name__ == "__main__":
    f = FakeController()
    while True:
        print(f.get_joint_angles())
        time.sleep(1)
