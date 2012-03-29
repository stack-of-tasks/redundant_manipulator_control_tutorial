#!/usr/bin/env python

# Copyright (c) 2012 CNRS-LAAS
# Author: Florent Lamiraux

import roslib; roslib.load_manifest ('tf')
import rospy
import tf
from geometry_msgs.msg import TransformStamped

def publish () :
    cameraName = "camera_top_right_optical"
    rospy.init_node ('tf_hueblob_simu')
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
        # Position of rose ball as a tf
        x = 0.4; y = -0.2; z = 1.2
        broadcaster.sendTransform ((x, y, z), (0., 0., 0., 1.),
                                   rospy.Time.now (),
                                   "rose_ball", "world")
        # Position of rose ball in world frame as a topic
        M.header.stamp = rospy.Time.now ()
        M.transform.translation.x = x
        M.transform.translation.y = y
        M.transform.translation.z = z
        M.transform.rotation.x = 0.0
        M.transform.rotation.y = 0.0
        M.transform.rotation.z = 0.0
        M.transform.rotation.w = 1.0
        pub_world.publish (M)

        rate.sleep ()

        if listener.frameExists("rose_ball") and \
                listener.frameExists(cameraName):
            try:
                t = listener.getLatestCommonTime("rose_ball", cameraName)
                (translation, quaternion) = \
                    listener.lookupTransform(cameraName, "rose_ball",
                                             t)
                (M.transform.translation.x,
                 M.transform.translation.y,
                 M.transform.translation.z) = translation
                print ("translation = %s"%str(translation))
                (M.transform.rotation.x,
                 M.transform.rotation.y,
                 M.transform.rotation.z,
                 M.transform.rotation.w) = quaternion
                pub_camera.publish (M)
            except:
                pass

if __name__ == '__main__':
    try:
        publish ()
    except rospy.ROSInterruptException:
        pass

