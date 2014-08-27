import rospy

from pr2 import arm
from pr2_sim import simulator

def test_mantis():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right', sim=sim)
    larm = arm.Arm('left', sim=sim)
    
    rarm.go_to_posture('mantis', block=False)
    larm.go_to_posture('mantis', block=False)
    
def test_gripper():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right', sim=sim)
    larm = arm.Arm('left', sim=sim)
    
    rarm.open_gripper()
    larm.open_gripper()

if __name__ == '__main__':
    rospy.init_node('pr2_arm', anonymous=True)
    test_mantis()
#     test_gripper()