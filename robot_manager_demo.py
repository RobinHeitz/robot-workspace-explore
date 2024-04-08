import robot_manager as rm

ur5 = rm.Robot()
print(ur5.get_joint_angles())
ur5.disconnect()