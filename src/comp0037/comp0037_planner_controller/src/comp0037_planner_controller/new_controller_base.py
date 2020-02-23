#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import time
import math

# This is the base class of the controller which moves the robot to its goal.

class NewControllerBase(object):

    def __init__(self, occupancyGrid):

        rospy.wait_for_message('/robot0/odom', Odometry)

        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Specification of accuracy. The first is the Euclidean
        # distance from the target within which the robot is assumed
        # to be there. The second is the angle. The latter is turned
        # into radians for ease of the controller.
        self.distanceErrorTolerance = rospy.get_param('distance_error_tolerance', 0.05)
        self.goalAngleErrorTolerance = math.radians(rospy.get_param('goal_angle_error_tolerance', 0.1))

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()

        # Store the occupany grid - used to transform from cell
        # coordinates to world driving coordinates.
        self.occupancyGrid = occupancyGrid
        
        # This is the rate at which we broadcast updates to the simulator in Hz.
        self.rate = rospy.Rate(10)

        #changes
        self.totalAngle = 0
        self.totalDistance = 0
        self.totalDrawingTime = 0
        self.totalCallBackTime = 0
        #..........

    #changes
    def resetpara(self):
        self.totalAngle = 0
        self.totalDistance = 0
        self.totalTime = 0
        self.lastTime = 0

    def resetLine(self, path, num):
        if num == 0:
            cell = path.waypoints[0]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            return waypoint, self.pose.x-waypoint[0], self.pose.y-waypoint[1]
        
        cell = path.waypoints[num-1]
        waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
        cell_2 = path.waypoints[num]
        waypoint_2 = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell_2.coords)
        return waypoint_2, waypoint[0]-waypoint_2[0], waypoint[1]-waypoint_2[1]
    #........

    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        #changes
        start_time = time.time()
        #..........
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation
        
        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        #changes
        if pose.theta > math.pi:
            pose.theta = pose.theta - 2 * math.pi
        #self.totalAngle = self.totalAngle + abs(self.pose.theta - pose.theta)
        angle_diff = abs(self.pose.theta - pose.theta)
        if angle_diff > math.pi:
            self.totalAngle = self.totalAngle + 2 * math.pi - angle_diff
        else:
            self.totalAngle = self.totalAngle + angle_diff

        self.totalDistance = self.totalDistance + abs(sqrt(pow((pose.x - self.pose.x), 2) + pow((pose.y - self.pose.y), 2)))
        self.pose = pose

        self.totalCallBackTime += time.time() - start_time
        #print("Callback: {}".format(self.totalCallBackTime))
        #print(self.pose)
        #print("Total Angle: {}".format(self.totalAngle*180/math.pi))
        #print("Total Distance: {}".format(self.totalDistance))
        #print("Total Time: {}".format(self.totalTime))
        #self.lastTime = time.time()
        #self.totalExeTime += time.time()-exe_start_time
        #..........

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    # Handle the logic of rotating the robot to its final orientation
    def rotateToGoalOrientation(self, waypoint):
        raise NotImplementedError()

    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goalOrientation, plannerDrawer):
        #changes
        self.resetpara()
        print("Total Angle: {}".format(self.totalAngle*180/math.pi))
        print("Total Distance: {}".format(self.totalDistance))
        #..........
        
        self.plannerDrawer = plannerDrawer

        rospy.loginfo('Driving path to goal with ' + str(len(path.waypoints)) + ' waypoint(s)')
        time_before_driving = time.time()
        #changes
        pre_waypoint, dX, dY = self.resetLine(path, 0)
        #..........
        # Drive to each waypoint in turn
        for waypointNumber in range(1, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            #changes
            dX_temp = pre_waypoint[0] - waypoint[0]
            dY_temp = pre_waypoint[1] - waypoint[1]
            if dX_temp != dX or dY_temp != dY:
            #..........
                rospy.loginfo("Driving to waypoint (%f, %f)", pre_waypoint[0], pre_waypoint[1])
                self.driveToWaypoint(pre_waypoint)
                #print("Total Angle: {}".format(self.totalAngle*180/math.pi))
                #print("Total Distance: {}".format(self.totalDistance))
                pre_waypoint, dX, dY = self.resetLine(path, waypointNumber)
            else:
                pre_waypoint = waypoint
            # Handle ^C
            if rospy.is_shutdown() is True:
                break
        
        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')

        # Finish off by rotating the robot to the final configuration
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)

        total_driving_time = time.time()-time_before_driving-self.totalDrawingTime
        print("Total Driving Time: {}".format(total_driving_time))
        print("Total Callback Time: {}".format(self.totalCallBackTime))
        print("Total Angle: {}".format(self.totalAngle*180/math.pi))
        print("Total Distance: {}".format(self.totalDistance))
