#!/usr/bin/env python

# this node subscribes to handle list and publishes pose array

import rospy, roslib, rosbag
import tf
roslib.load_manifest('handle_detector')
roslib.load_manifest('tfx')
import geometry_msgs.msg as gm
import handle_detector.msg as hd_msg
import tfx

import threading
import numpy as np

import IPython

class HandlePosesPublisher:
    def __init__(self, handle_topic='/handle_detector/handle_list'):
        self.handle_list_msg = None
        self.handle_list_sub = rospy.Subscriber(handle_topic, hd_msg.HandleListMsg, self._handles_callback)
        self.handles_pose_pub = rospy.Publisher('/handle_detector/handle_poses', gm.PoseArray)
        self.avg_handles_pose_pub = rospy.Publisher('/handle_detector/avg_handle_poses', gm.PoseArray)
        
        self.handles_thread = threading.Thread(target=self._handles_loop)
        self.handles_thread.start()
                    
    def _handles_callback(self, msg):
        self.handle_list_msg = msg
        
    def _handles_loop(self):
        """
        For each handle in HandleListMsg,
        calculate average pose
        """
        while not rospy.is_shutdown():
            rospy.sleep(.01)
            
            handle_list_msg = self.handle_list_msg
            if handle_list_msg is None:
                continue
            
            pose_array = gm.PoseArray()
            pose_array.header.frame_id = handle_list_msg.header.frame_id
            pose_array.header.stamp = rospy.Time.now()
            
            avg_pose_array = gm.PoseArray()
            avg_pose_array.header.frame_id = handle_list_msg.header.frame_id
            avg_pose_array.header.stamp = rospy.Time.now()
    
            cam_to_base = tfx.lookupTransform('base_link', handle_list_msg.header.frame_id).matrix[:3,:3]
            switch = np.matrix([[0, 1, 0],
                                [1, 0, 0],
                                [0, 0, 1]])        
            for handle in handle_list_msg.handles:
                all_poses = [tfx.pose(cylinder.pose, stamp=rospy.Time.now(), frame=handle_list_msg.header.frame_id) for cylinder in handle.cylinders]
                
                rotated_poses = [tfx.pose(p.position, tfx.tb_angles(p.orientation.matrix*switch)) for p in all_poses]
                filtered_poses = list()
                for rot_pose in rotated_poses:
                    r_base = cam_to_base*rot_pose.orientation.matrix
                    if r_base[0,0] > 0:
                        if r_base[2,2] > 0:
                            rot_pose.orientation = tfx.tb_angles(rot_pose.orientation.matrix*tfx.tb_angles(0,0,180).matrix) 
                        filtered_poses.append(rot_pose)
                
                pose_array.poses += [pose.msg.Pose() for pose in filtered_poses]
                
                if len(filtered_poses) > 0:
                    avg_position = sum([p.position.array for p in filtered_poses])/float(len(filtered_poses))
                    avg_quat = sum([p.orientation.quaternion for p in filtered_poses])/float(len(filtered_poses))
                    avg_pose_array.poses.append(tfx.pose(avg_position, avg_quat).msg.Pose())
                
                
            if len(pose_array.poses) > 0:
                self.handles_pose_pub.publish(pose_array)
                self.avg_handles_pose_pub.publish(avg_pose_array)
                
            
    def _handles_callback(self, msg):
        self.handle_list_msg = msg
                        
    
if __name__ == '__main__':
    rospy.init_node('handle_poses_publisher', anonymous=True)
    
    handle_poses_publisher = HandlePosesPublisher()
    rospy.spin()
