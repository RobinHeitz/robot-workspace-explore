"""
Connects to a real UR and reads the joint state into the bufer.
"""

import threading
import time
from datetime import datetime

import urx


class URController:
    __robot_ip = "192.168.0.10"
    __wait_after_init_in_secs = 2

    def __init__(self, sample_frequency_hz=10):
        self.__rob = self.__connect()
        self.stop_flag = False
        self.buffer = []
        self.sample_frequency_hz = sample_frequency_hz

        self.t = threading.Thread(target=self.__read_joints, daemon=True)

    def __read_joints(self):
        local_time = datetime.now()

        while not self.stop_flag:
            self.buffer.append(self.get_joint_angles().copy())
            print(self.buffer[-1])
            time.sleep(1 / self.sample_frequency_hz)

    def __connect(self):
        """
        Connect robot
        """
        print("Connection with the robot is loading...")
        attempts = 0
        while attempts < 10:
            try:
                rob = urx.Robot(self.__robot_ip, use_rt=True)
                time.sleep(self.__wait_after_init_in_secs)
                return rob
            except Exception as e:
                print(f"Error occurred: {e}")
                print("A new attempt is made to establish the connection.")
                attempts += 1
                time.sleep(5)
        raise Exception("Connection to the robot could not be established.")

    def get_joint_angles(self):
        """
        Return the actual joint angles of the robot
        """
        return self.__rob.rtmon.q_actual()

    def disconnect(self):
        """
        Disconnect the robot
        """
        self.__rob.close()
        print("Robot disconnected.")

    def start_buffering(self):
        self.t.start()

    def stop_buffering(self):
        self.stop_flag = True

    def get_buffer(self):
        print(f"Buffer has length of {len(self.buffer)}")
        return self.buffer


if __name__ == "__main__":
    c = URController()
    time.sleep(1)
    print(c.get_joint_angles())
    c.start_buffering()

    time.sleep(15)
    print("buffer length:", len(c.get_buffer()))

    c.stop_buffering()
    c.disconnect()
