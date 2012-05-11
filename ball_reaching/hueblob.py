#!/usr/bin/env python

# Copyright (c) 2012 CNRS-LAAS
# Author: Florent Lamiraux

import roslib; roslib.load_manifest ('tf')
import rospy
import tf
from geometry_msgs.msg import TransformStamped

def publish () :
    print ("Starting hueblob")
    cameraName = "camera_bottom_left_optical"
    rospy.init_node ('tf_hueblob')
    pub_world = rospy.Publisher ('/wide/blobs/rose/transform_base_link',
                                 TransformStamped)
    pub_camera = rospy.Publisher ('/wide/blobs/rose/transform',
                                  TransformStamped)
    transformer = tf.Transformer ()
    broadcaster = tf.TransformBroadcaster ()
    listener = tf.TransformListener()
    M = TransformStamped ()
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        if listener.frameExists("roseball") and \
                listener.frameExists("base_link"):
            try:
                t = listener.getLatestCommonTime("roseball", "base_link")
                (translation, quaternion) = \
                    listener.lookupTransform("base_link", "roseball", t)
                # Position of rose ball in base_link frame as a topic
                M.header.stamp = rospy.Time.now ()
                (M.transform.translation.x,
                 M.transform.translation.y,
                 M.transform.translation.z) = translation
                (M.transform.rotation.x,
                 M.transform.rotation.y,
                 M.transform.rotation.z,
                 M.transform.rotation.w) = quaternion
                pub_world.publish (M)
                rate.sleep ()
            except:
                pass
            
if __name__ == '__main__':
    try:
        publish ()
    except rospy.ROSInterruptException:
        pass

