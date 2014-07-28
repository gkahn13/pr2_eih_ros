#!/usr/bin/env python

import rospy
import std_msgs
import roslib
roslib.load_manifest('tfx')
roslib.load_manifest('ar_pose')
import tfx
from ar_pose.msg import ARMarkers, ARMarker
from geometry_msgs.msg import Pose
import rospkg
import os


def head_callback(data):
    global head_pose
    #if not head_pose and data.markers:
    if data.markers:
        head_pose = data.markers[0].pose.pose

def hand_callback(data):
    global hand_pose
    #if not hand_pose and data.markers:
    if data.markers:
        hand_pose = data.markers[0].pose.pose

def kinect_transform_publisher():
    global head_pose, hand_pose
    head_pose = None
    hand_pose = None
    dummy = tfx.transform([0,0,0])
    #pub = rospy.Publisher('kinect_transform', )
    pub = tfx.TransformBroadcaster()
    rospy.init_node('kinect_transform_publisher', anonymous=True)

    rospy.Subscriber("/hand_kinect_ar_kinect_pose", ARMarkers, hand_callback)
    rospy.Subscriber("/head_kinect_ar_kinect_pose", ARMarkers, head_callback)
    transformer = tfx.TransformListener()
    r = rospy.Rate(1)

    #head_pose = True # REMOVE THIS
    #head_pose = False # REMOVE THIS
    i = 0
    transforms = []
    while not rospy.is_shutdown() and i < 50:
        if head_pose and hand_pose:
            head_transform = tfx.transform(head_pose, parent='camera_rgb_optical_frame', child='ar_frame')#.as_transform()
            hand_transform = tfx.transform(hand_pose, parent='hand_kinect_optical_frame', child='ar_frame')#.as_transform()
            #head_to_ar = tfx.transform(transformer.lookupTransform('ar_frame', 'camera_rgb_optical_frame', rospy.Time()), parent='camera_rgb_optical_frame', child='ar_frame')
            #ar_to_hand = tfx.transform(transformer.lookupTransform('hand_kinect_optical_frame', 'ar_frame', rospy.Time()), parent='ar_frame', child='hand_kinect_optical_frame')
            head_to_hand = tfx.transform(head_transform.matrix * hand_transform.inverse().matrix, parent='camera_rgb_optical_frame', child='hand_kinect_optical_frame')
            #head_to_hand = tfx.transform(head_to_ar.inverse().matrix * ar_to_hand.matrix, parent='camera_rgb_optical_frame', child='hand_kinect_optical_frame')
            #rospy.loginfo(head_transform)
            #rospy.loginfo(hand_transform.inverse())
            #rospy.loginfo(head_to_ar)
            #rospy.loginfo(ar_to_hand)
            #print head_to_hand
            wrist_to_head = tfx.transform(transformer.lookupTransform('r_gripper_tool_frame', 'camera_rgb_optical_frame', rospy.Time()), child = 'camera_rgb_optical_frame', parent = 'r_gripper_tool_frame')
            wrist_to_hand = tfx.transform(wrist_to_head.matrix * head_to_hand.matrix, parent='r_gripper_tool_frame', child='hand_kinect_optical_frame')
            #print wrist_to_head
            print wrist_to_hand
            print
            #rospy.loginfo(wrist_to_head)
            #rospy.loginfo(wrist_to_hand)
            #pub.sendTransform(wrist_to_hand.position, wrist_to_hand.rotation, rospy.Time(), wrist_to_hand.child, wrist_to_hand.parent)
            #pub.sendTransform(head_to_hand.position, head_to_hand.rotation, rospy.Time(), head_to_hand.child, head_to_hand.parent)
            transforms.append(wrist_to_hand)
            i += 1
        r.sleep()
    transform = transforms[0]
    for i in range(1, len(transforms)):
        transform = transform.interpolate(transforms[i], 1 / len(transforms))
    transform.child = "hand_kinect_optical_frame"
    print transform
    rospack = rospkg.RosPack()
    os.chdir(rospack.get_path("kinect_transform"))
    s = '<launch>\n'
    s += '<node pkg="tf" type="static_transform_publisher" name="static3" args="'
    for value in transform.position.list + transform.quaternion.list:
        s += str(value) + ' '
    s += transform.parent + ' ' + transform.child + ' '
    s += '100" />\n'
    s += '</launch>'
    try:
        f = open("launch/wrist_to_hand.launch", "w")
        f.write(s)
    finally:
        f.close()
    
        
if __name__ == '__main__':
    try:
        kinect_transform_publisher()
    except rospy.ROSInterruptException:
        pass
