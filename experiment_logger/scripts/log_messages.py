#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
from datetime import datetime

messages = []

def string_callback(data):
    rospy.loginfo(rospy.get_caller_id()+"\n%s",data.data)
    log_message(data.data)

def write_out():
    log_message("End of experiment")
    print
    print "writing to log file: {0}".format(log_file)
    with open(log_file, "a+") as file:
        file.writelines(messages)
        
    print "done"

def log_message(string):
    messages.append("[" + str(datetime.now()) + "] " + string + '\n')

def log_raw(string):
    messages.append(string)    
    
def logger():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('log_messages', anonymous=True)

    rospy.Subscriber("/experiment_log", String, string_callback)

    rospy.on_shutdown(write_out)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    try:
        global log_file
        log_file = "default_log.txt"
        if len(sys.argv) > 1:
            log_file = sys.argv[1]
        log_raw("-------------------\n")
        log_message("Beginning new experiment")
        logger()
    except:
        write_out()
        raise
