#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys

def successful():
    pub.publish('successful grasp')

def missed():
    pub.publish('missed grasp')
    
def incorrect():
    pub.publish('incorrect grasp identified')

def dropped():
    pub.publish('dropped object')

def quit():
    sys.exit(0)

def custom():
    command = raw_input('enter a custom command > ')
    pub.publish(command)

commands = {'s': successful,
            'm': missed,
            'i': incorrect,
            'd': dropped,
            'q': quit,
            'c': custom}

commands_print = 's : successful grasp\n' + \
                 'm : missed grasp\n' + \
                 'i : incorrect grasp identified\n' + \
                 'd : dropped en route\n' + \
                 'c : custom\n'

def manual_logger():
    global pub
    rospy.init_node('manual_logger', anonymous=True)
    pub = rospy.Publisher("/experiment_log", String)
    while not rospy.is_shutdown():
        print(commands_print)
        command = raw_input('enter a command for logging > ')
        if command in commands:
            commands[command]()
        else:
            print "Unrecognized command: {0}".format(command)

if __name__ == '__main__':
    manual_logger()
