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
    pub_world = rospy.Publisher ('/wide/blobs/rose/transform_world',
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
                listener.frameExists("world"):
            try:
                t = listener.getLatestCommonTime("roseball", "world")
                (translation, quaternion) = \
                    listener.lookupTransform("world", "roseball", t)
                print ("t={0}".format (t))
                broadcaster.sendTransform (translation, quaternion,
                                           rospy.Time.now (),
                                           "roseball", "world")

                # Position of rose ball in world frame as a topic
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
                print ("No common time found.")
        else:
            if not listener.frameExists("roseball"):
                print ("roseball frame does not seem to exist.")
            if not listener.frameExists("world"):
                print ("world frame does not seem to exist.")

if __name__ == '__main__':
    try:
        publish ()
    except rospy.ROSInterruptException:
        pass

