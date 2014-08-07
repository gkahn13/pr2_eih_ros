#!/usr/bin/env python

# this node publishes 20 affordances at a time, so it will be manageable for rviz to display them

import rospy, roslib
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('handle_detector')
import sensor_msgs.msg as sm
import geometry_msgs.msg as gm
import handle_detector.msg as hd_msg
import visualization_msgs.msg as vis_msg

class AffordancePublisher:

    def __init__(self, num_published=20):
        self.sub = rospy.Subscriber('/handle_detector/visualization_all_affordances', vis_msg.MarkerArray, self._affordances_callback)
        self.pub = rospy.Publisher('/handle_detector/visualization_some_affordances', vis_msg.MarkerArray)
        self.current_start = 0
        self.n = num_published
        self.last_msg = None

    def run(self):
        while not rospy.is_shutdown():
            if self.last_msg:
                rospy.loginfo('Press enter to publish')
                raw_input()
                current_msg = vis_msg.MarkerArray()
                current_msg.markers = self.last_msg.markers[self.current_start:self.current_start+self.n]
                for marker in current_msg.markers:
                    marker.lifetime = rospy.Duration(0)
                    marker.header.stamp = rospy.Time()
                self.pub.publish(current_msg)
                self._update()
                print "published"
                print "{0} out of {1} published".format(self.current_start+self.n, len(self.last_msg.markers))

    def _affordances_callback(self, msg):
        print "received message"
        self.last_msg = msg

    def _update(self):
        self.current_start += self.n
        if self.current_start >= len(self.last_msg.markers):
            self.current_start = 0



if __name__ == '__main__':
    rospy.init_node('publish_affordances', anonymous=True)
    affordances_publisher = AffordancePublisher()
    affordances_publisher.run()
    
