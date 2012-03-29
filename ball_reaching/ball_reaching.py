# Copyright (c) 2012 CNRS-LAAS
# Author: Florent Lamiraux

#!/usr/bin/env python

import time
from dynamic_graph import plug
from dynamic_graph.sot.core.operator import Compose_R_and_T,\
    Multiply_of_matrixHomo
from dynamic_graph.sot.core import TaskPD, FeaturePosition, FeaturePosture
from dynamic_graph.sot.core import FeatureVisualPoint
from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas
from dynamic_graph.sot.dynamics.solver import Solver
from dynamic_graph.sot.motion_planner import VispPointProjection
from dynamic_graph.sot.reaching import CubicInterpolationSE3
from dynamic_graph.ros import Ros

I4 = reduce(lambda m, i: m + (i*(0.,)+(1.,)+ (3-i)*(0.,),), range(4), ())

# Prologue
if __name__ == '__main__':
    robot = Hrp2Laas ('robot')
    # Initialize the zmp signal to the current com.
    _com = robot.dynamic.com.value
    robot.device.zmp.value = (_com[0], _com[1], 0.)
    solver = Solver(robot)
    s = ['left-ankle', 'right-ankle']
    for i in s:
        robot.dynamic.signal(i).recompute(robot.dynamic.signal(i).time + 1)
        robot.features[i].reference.value = \
            robot.dynamic.signal(i).value
        robot.features[i]._feature.selec.value = '111111'
        robot.tasks[i].controlGain.value = 180.

    robot.featureComDes.errorIN.value = robot.dynamic.com.value
    robot.featureComDes.selec.value = '111'
    robot.comTask.controlGain.value = 180.
    robot.balanceTask.controlGain.value = 180.


    # Push com and feet tasks.
    #
    # The robot is currently in half-sitting, so this script freezes com
    # and feet position so that the robot will remain stable while the
    # user program is starting.
    solver.push(robot.balanceTask)

class Motion (object):
    def __init__ (self, robot, solver):
        self.robot = robot
        self.solver = solver
        self.ros = Ros (robot)
        self.ros.rosExport.add ('matrixHomoStamped', 'ballInCamera',
                                '/wide/blobs/rose/transform')
        self.ros.rosExport.add ('matrixHomoStamped', 'ballInWorld',
                                '/wide/blobs/rose/transform_world')
        self.rightGripperId = 28
        # Right hand task
        self.featureRightHand = \
            FeaturePosition ('featureRightHand',
                             robot.frames ['rightHand'].position,
                             robot.frames ['rightHand'].jacobian)
        self.featureRightHand.selec.value = '000111'

        self.interpolation = CubicInterpolationSE3 ('interpolation')
        self.interpolation.setSamplingPeriod (0.005)
        plug (self.interpolation.reference,
              self.featureRightHand.signal('reference'))
        plug (self.ros.rosExport.ballInWorld, self.interpolation.goal)
        plug (self.featureRightHand.position, self.interpolation.init)

        self.taskRightHand = TaskPD ('taskRightHand')
        self.taskRightHand.add (self.featureRightHand.name)
        #plug (interpolation.errorDot, taskRightHand.errorDot)
        self.taskRightHand.controlGain.value = 180.0
        self.solver.push (self.taskRightHand)

        # Waist task
        self.robot.waist.selec.value = '011000'
        self.robot.waist.value = I4
        self.solver.push (self.robot.tasks ['waist'])

        # Ball tracking
        refBallInCamera = FeatureVisualPoint ('featureBallInCameraRef')
        refBallInCamera.xy.value = (0., 0.)
        self.pinholeProjection = VispPointProjection('pinholeProjection')
        plug (self.ros.rosExport.ballInCamera, self.pinholeProjection.cMo)
        plug (self.ros.rosExport.ballInCameraTimestamp,
              self.pinholeProjection.cMoTimestamp)
        self.ballInCamera = FeatureVisualPoint ('featureBallInCamera')
        plug (self.pinholeProjection.xy, self.ballInCamera.xy)
        self.ballInCamera.Z.value = 1.
        self.ballInCamera.setReference (refBallInCamera.name)
        self.ballInCamera.selec.value = '11'
        plug (self.robot.frames ['cameraTopRight'].jacobian,
              self.ballInCamera.Jq)

        self.taskBallTracking = TaskPD ('taskBallTracking')
        self.taskBallTracking.add (self.ballInCamera.name)
        self.taskBallTracking.controlGain.value = 1.0

        # Posture task
        self.featurePosture = FeaturePosture ('featurePosture')
        plug (self.robot.device.state, self.featurePosture.state)
        self.featurePosture.setPosture (self.robot.halfSitting)
        self.postureTask = TaskPD ('postureTask')
        self.postureTask.add (self.featurePosture.name)
        self.postureTask.controlGain.value = 1.

    def start (self):
        #self.solver.push (self.taskBallTracking)
        # open hand
        for dof in range (6, self.robot.dimension):
            self.featurePosture.selectDof (dof, False)
        self.featurePosture.selectDof (self.rightGripperId, True)
        config = list (self.robot.halfSitting [::])
        config [self.rightGripperId] = 0.7
        self.featurePosture.setPosture (tuple (config))
        self.solver.push (self.postureTask)

        self.waitForBall ()
        # move hand
        handInitPos = self.robot.frames ['rightHand'].position.value
        self.interpolation.start (2.)
        time.sleep (2.)
        # close hand
        self.solver.sot.remove (self.taskBallTracking.name)
        config [self.rightGripperId] = 0.3
        self.featurePosture.setPosture (tuple (config))
        self.postureTask.controlGain.value = 4.
        time.sleep (.5)
        # move hand back
        self.postureTask.controlGain.value = 1.
        for dof in range (6, len (self.robot.halfSitting)):
            self.featurePosture.selectDof (dof, True)
        self.interpolation.goal.value = handInitPos
        self.interpolation.start (2.)
        time.sleep (2.0)
        self.solver.remove (self.taskBallTracking)
        self.solver.remove (self.postureTask)
        plug (self.ros.rosExport.ballInWorld, self.interpolation.goal)

    def waitForBall (self):
        pass
        
    def play(self, totalTime):
        nbSteps = int(totalTime/timeStep) + 1
        path = []

        for i in xrange(nbSteps):
            print (i)
            if i == 100:
                self.start ()
            t = timeStep*i
            self.robot.device.increment(timeStep)
            config = self.robot.device.state.value
            path.append(config)
            if hasRobotViewer:
                clt.updateElementConfig(rvName, toViewerConfig(config))
                ballPos = []
                for i in range (3):
                    ballPos.append(self.ros.rosExport.ballInWorld.value[i][3])
                ballPos.extend (3*[0])
                clt.updateElementConfig("rose_ball", ballPos)


if __name__ == '__main__':
    def toViewerConfig(config):
        return config + 10*(0.,)

    try:
        import robotviewer
        clt = robotviewer.client()
        rvName = 'hrp'
        hasRobotViewer = True
    except:
        hasRobotViewer = False
        print "no robotviewer"

    timeStep = .005
    gravity = 9.81

    m = Motion (robot, solver)
    #m.play (3.)
