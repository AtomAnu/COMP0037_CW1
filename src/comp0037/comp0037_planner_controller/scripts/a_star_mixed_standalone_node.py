#! /usr/bin/env python

# Import the needed types.
from comp0037_planner_controller.a_star_mixed import AStarMixedPlanner
from comp0037_planner_controller.occupancy_grid import OccupancyGrid
import map_getter
import rospy

from nav_msgs.srv import GetMap
# Initialise node
rospy.init_node('A_Star_Euclidean_standalone', anonymous=True)

# Mapgetter  helps load maps off the map server
"""
****** UNCOMMENT THE TWO LINES BELOW FOR ALL MAPS EXCEPT FACTORY MAP
"""
mapGetter = map_getter.MapGetter()
occupancyGrid = mapGetter.getMapFromServer()

"""
****** UNCOMMENT THE TWO LINES BELOW FOR EMPTY MAP ONLY
"""
#Add a wall to the empty map
for y in xrange(2, 57):
    occupancyGrid.setCell(45, y, 1)

"""
****** FOR FACTORY MAP ONLY
"""
#rospy.loginfo('Waiting for static_map to become available.')
#rospy.wait_for_service('static_map') 
#mapServer = rospy.ServiceProxy('static_map', GetMap)
#rospy.loginfo('Found static_map; requesting map data')

# Query the map status
#response = mapServer()
#map = response.map
#rospy.loginfo('Got map data')

# Allocate the occupancy grid and set the data from the array sent back by the map server
#occupancyGrid = OccupancyGrid(map.info.width, map.info.height, map.info.resolution)
#occupancyGrid.setScale(rospy.get_param('plan_scale', 5))
#occupancyGrid.setFromDataArrayFromMapServer(map.data)
#occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0.2)

"""
****** UP TO HERE
"""

start = rospy.get_param("start_pose")
goal = rospy.get_param("goal_pose")

# Create the planner. The first field is the title which will appear in the
# graphics window, the second the occupancy grid used.
planner = AStarMixedPlanner('A Star Euclidean Search', occupancyGrid)

# This causes the planner to slow down and pause for things like key entries
planner.setRunInteractively(True)

# This specifies the height of the window drawn showing the occupancy grid. Everything
# should scale automatically to properly preserve the aspect ratio
planner.setWindowHeightInPixels(400)

# Search and see if a path can be found. Returns True if a path from the start to the
# goal was found and False otherwise
goalReached = planner.search(start, goal)

# Extract the path. This is based on the last search carried out.
path = planner.extractPathToGoal()

print "Max Length of the Queue: " + str(planner.getMaxQueueLength())
print "Total Number of Cells Visitied: " + str(planner.getNumberOfCellVisited())
print "Final Queue Length: " + str(planner.queueLength)
print "Total Travel Cost: " + str(planner.getTotalCost(path))
print "Total Angle: " + str(planner.getTotalAngle(path))
