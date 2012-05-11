#! /usr/bin/env python

from math import sqrt
import roslib; roslib.load_manifest('ball_reaching')
import rospy
import actionlib
import ball_reaching.msg
from geometry_msgs.msg import TransformStamped


class WatchZone (object):
    """
    Action that consist of waiting for the rose ball to enter a sphere centered
    on the goal of the action.
    """
    radius = .05
    _feedback = ball_reaching.msg.watch_zoneActionFeedback ()
    _result = ball_reaching.msg.watch_zoneResult ()

    def __init__ (self, name):
        self.x, self.y, self.z = (None, None, None)
        self._action_name = name
        self._as = \
            actionlib.SimpleActionServer (self._action_name,
                                          ball_reaching.msg.watch_zoneAction,
                                          execute_cb = self.execute_cb)
        self._as.start ()
        self.log_topic_cb = 0
       # Read position of rose ball in topic
        rospy.Subscriber ('/wide/blobs/rose/transform_base_link',
                          TransformStamped, lambda x: self.topic_cb (x))


    def execute_cb (self, goal):
        rospy.loginfo ("Entering WatchZone.execute_cb")
        rate = rospy.Rate (10)
        x0 = goal.xCenter; y0 = goal.yCenter; z0 = goal.zCenter
        log = 0
        success = False
        while not success:
            if self.x:
                self._feedback.feedback.distance = \
                    sqrt ((self.x - x0)*(self.x - x0) +
                          (self.y - y0)*(self.y - y0) +
                          (self.z - z0)*(self.z - z0))
                if (self._feedback.feedback.distance < .05) :
                    self._result.distance = self._feedback.feedback.distance
                    success = True
                log += 1
                if (log == 10):
                    log = 0
                    rospy.loginfo ("watch_zone: distance = {0}".format
                                   (self._feedback.feedback.distance))
            rate.sleep ()

        rospy.loginfo ("Ball entered the watched zone.")
        self._as.set_succeeded (self._result)

    def topic_cb (self, M):
        """
        Callback called when a new data has been published in
        rose_ball/transform_base_link topic.
        """
        self.x = M.transform.translation.x
        self.y = M.transform.translation.y
        self.z = M.transform.translation.z
        self.log_topic_cb += 1

if __name__ == "__main__":
    rospy.init_node ('watch_zone')
    WatchZone (rospy.get_name ())
    rospy.spin ()
