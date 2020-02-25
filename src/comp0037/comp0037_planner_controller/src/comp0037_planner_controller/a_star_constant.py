# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# changes
from math import *
import numpy as np
#..........

class AStarConstantPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.fifoQueue = deque()
        # changes
        self.queueLength = len(self.fifoQueue)
        self.maxQueueLength = self.queueLength
        self.nCellsDead= len(self.fifoQueue)
        #..........

    # changes
    def updateMaxQueueLength(self, num):
        self.queueLength = self.queueLength + num
        if self.queueLength > self.maxQueueLength:
            self.maxQueueLength = self.queueLength
        if num == -1:
            self.nCellsDead = self.nCellsDead + 1
    
    def getNumberOfCellVisited(self):
        return self.nCellsDead + self.queueLength - 1
    
    def getMaxQueueLength(self):
        return self.maxQueueLength

    def getTotalCost(self, path):
        cell = path.waypoints[-1].parent
        cost = self.computeLStageAdditiveCost(cell, path.waypoints[-1])
        while (cell is not None):
            cost = cost + self.computeLStageAdditiveCost(cell.parent, cell)
            cell = cell.parent

        if path.goalReached is False:
            cost = float("inf")
        
        return cost
    
    def computeAngle(self, parentCell, cell):
        dX = cell.coords[0] - parentCell.coords[0]
        dY = cell.coords[1] - parentCell.coords[1]
        return atan2(dX, dY)*180/pi

    def getTotalAngle(self, path):
        cell = path.waypoints[-1].parent
        angle = self.computeAngle(cell, path.waypoints[-1])
        total_angle = 0
        while (cell.parent is not None):
            angle_new = self.computeAngle(cell.parent, cell)
            angle_diff = np.max([angle, angle_new])-np.min([angle, angle_new])
            if angle_diff > 180:
                total_angle = total_angle + 360 - angle_diff
            else:
                total_angle = total_angle + angle_diff
            cell = cell.parent
            angle = angle_new
        
        return total_angle

    def computeCost(self, cell, parentCell):
        if (parentCell is None):
            return 0
        return parentCell.pathCost + self.computeLStageAdditiveCost(cell, parentCell)
    
    def setCost(self, cell, cost):
        cell.pathCost = cost
    
    def non_neg_constant_heuristic(self):
        return 1

    def checkParent(self):
        for cell in self.fifoQueue:
            print("c: {}".format(cell.coords))
            if (cell.parent is not None):
                print("c parent: {}".format(cell.parent.coords))

    def insert(self, cell):
        if len(self.fifoQueue) == 0:
            self.fifoQueue.append(cell)
        else:
            index = -1
            for i in range(len(self.fifoQueue)):
                cost = self.computeCost(cell, cell.parent)
                if cost + self.non_neg_constant_heuristic() <= self.fifoQueue[i].pathCost + self.non_neg_constant_heuristic():
                    index = i
                    self.setCost(cell, cost)
                    break
            if index == -1:
                self.fifoQueue.append(cell)
            else:
                self.fifoQueue.rotate(-index)
                self.fifoQueue.appendleft(cell)
                self.fifoQueue.rotate(index)
    #..........

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        # changes
        self.insert(cell)
        self.updateMaxQueueLength(1)
        #..........

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.fifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.fifoQueue.popleft()

        # changes
        self.updateMaxQueueLength(-1)
        #..........

        return cell

    def resolveDuplicate(self, cell, parentCell):
        temp = cell.pathCost
        if self.computeCost(cell, parentCell) < temp:
            cell.parent = parentCell
            if (cell in self.fifoQueue):
                self.fifoQueue.remove(cell)
                self.insert(cell)  
