#!/usr/bin/env python

# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux

import roslib; roslib.load_manifest ('ball_reaching')
import rospy
import actionlib

roslib.load_manifest ('dynamic_graph_bridge')
roslib.load_manifest('openhrp_bridge')
from dynamic_graph_bridge.srv import RunCommand
from std_srvs.srv import Empty

from ball_reaching.msg import *

if __name__ == '__main__':
    rospy.init_node ('supervisor')
    client = actionlib.SimpleActionClient ('watch_zone', watch_zoneAction)
    client.wait_for_server ()

    # Initialize "run_command" client and stack of tasks
    rospy.wait_for_service ('run_command')
    run_command = rospy.ServiceProxy ('run_command', RunCommand)
    result =\
        run_command ("from dynamic_graph.sot.reaching.reach_ball import Motion;"
                     + "m = Motion (robot, solver); m.trackBall ()")
    rospy.loginfo ("stdout: " + result.stdout)
    rospy.loginfo ("stderr: " + result.stderr)
    goal = watch_zoneGoal (xCenter = 0.67, yCenter = -0.09, zCenter = 0.53)
    # Fill in the goal here
    client.send_goal (goal)
    client.wait_for_result (rospy.Duration.from_sec(150.0))
    rospy.loginfo ("Ball entered watched zone.")
    #Trigger arm motion
    run_command ("m.catchBall ()")
