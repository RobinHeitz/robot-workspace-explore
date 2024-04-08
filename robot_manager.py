import urx # pip install urx
import time

class Robot:
    '''
    Manager for the robot connection
    '''
    def __init__(self, robot_ip="192.168.0.10", wait_after_init_in_secs=2):
        self.__robot_ip = robot_ip
        self.__wait_after_init_in_secs = wait_after_init_in_secs
        self.__rob = self.__connect()

    def __connect(self):
        '''
        Connect robot
        '''
        print("Connection with the robot is loading...")
        attempts = 0
        while attempts < 10:
            try:
                rob = urx.Robot(self.__robot_ip,use_rt=True)
                time.sleep(self.__wait_after_init_in_secs)
                return rob
            except Exception as e:
                print(f"Error occurred: {e}")
                print("A new attempt is made to establish the connection.")
                attempts += 1
                time.sleep(5)
        raise Exception("Connection to the robot could not be established.")
    
    def get_joint_angles(self):
        '''
        Return the actual joint angles of the robot
        '''
        return self.__rob.rtmon.q_actual()

    def disconnect(self):
        '''
        Disconnect the robot
        '''
        self.__rob.close()
        print("Robot disconnected.")

