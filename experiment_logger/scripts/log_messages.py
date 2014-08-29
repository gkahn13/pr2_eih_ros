#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
from datetime import datetime

messages = []

def string_callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    messages.append(data.data + '\n')

def write_out():
    messages.append("End of experiment" + '\n')
    messages.append(str(datetime.now()) + '\n')
    print
    print "writing to log file: {0}".format(log_file)
    with open(log_file, "a+") as file:
        file.writelines(messages)
        
    print "done"

def log_message(string):
    messages.append(string + '\n')
    
    
def logger():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('log_messages', anonymous=True)

    rospy.Subscriber("experiment_log", String, string_callback)

    rospy.on_shutdown(write_out)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    global log_file
    log_file = "default_log.txt"
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    messages.append("-------------------" + '\n')
    messages.append("Beginning new experiment" + '\n')
    messages.append(str(datetime.now()) + '\n')
    logger()
