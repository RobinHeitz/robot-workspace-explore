"""Imitates joint angles that are obtained from free-drive-mode of a real robot."""

import threading
import time
from datetime import datetime

import numpy as np


class FakeController:

    q_start = [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
    j0_freq = 4
    j2_freq = 4

    def __init__(self, buffer_duration):
        self._q = self.q_start.copy()
        self.stop_flag = False

        self.buffer_duration = buffer_duration
        self.buffer = []

        self.t = threading.Thread(target=self._alter_joints, daemon=True)

    def start_buffering(self):
        self.t.start()

    def stop_buffering(self):
        self.stop_flag = True

    def _alter_joints(self):
        start_time = datetime.now()
        local_start = datetime.now()

        while not self.stop_flag:
            total_secs = (datetime.now() - start_time).total_seconds()
            new_q = self._q.copy()

            # Joint 0
            angle = np.pi / self.j0_freq * total_secs
            angle = angle % (2 * np.pi)
            angle -= np.pi
            new_q[0] = angle

            # Joint 2
            deviation = 0.7 * np.sin(np.pi / self.j2_freq * total_secs)
            new_q[2] = self.q_start[2] + deviation

            if (datetime.now() - local_start).total_seconds() >= self.buffer_duration:
                self.buffer.append(new_q.copy())
                local_start = datetime.now()

            self._q = new_q

    def get_joint_angles(self):
        return self._q

    def get_buffer(self):
        print(f"Buffer has length of {len(self.buffer)}")
        return self.buffer


if __name__ == "__main__":
    f = FakeController(buffer_duration=0.02)
    f.start_buffering()
    time.sleep(3.33)
    f.stop_buffering()
    time.sleep(1)
    print(len(f.get_buffer()))
