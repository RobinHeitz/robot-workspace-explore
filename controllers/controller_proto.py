"""
This protocol defines the expected interface of a controller, similar to a subclass.
"""

from typing import Protocol


class ControllerProtocol(Protocol):

    def start_buffering(self):
        pass

    def stop_buffering(self):
        pass

    def get_joint_angles(self):
        pass

    def get_buffer(self):
        pass
