from ros_pathfinder.planning.costmap import Costmap2d
from ros_pathfinder.planning.a_star import AStarPlanner
from ros_pathfinder.planning.base_planner import GridCell

import heapq
import numpy as np

class FrontierCellFinder:
    def __init__(self,start:GridCell,costmap:Costmap2d):
        
        self.costmap = costmap
        self.width = costmap.width
        self.height = costmap.height
        self.values = costmap.values
        self.numValues = len(self.values)
        self.start = self.width*start[1] + start[0]

        self.frontierCells = self.getFrontierCells()
        self.frontierZones = self.getFrontierZones()
        self.path = self.getPath()
    def getFrontierCells(self):
        frontierCellSet = set()
        for i,v in enumerate(self.values):
            if v == 0:
                if i % self.width == 0:
                    neighbors = [-self.width,-self.width+1,1,self.width,self.width+1]
                elif (i-1) % self.width == 0: 
                    neighbors = [-self.width-1,-self.width,-1,self.width-1,self.width]
                else:
                    neighbors = [-self.width-1,-self.width,-self.width+1,-1,1,self.width-1,self.width,self.width+1]

                for n in neighbors:
                    if (i+n) < 0 or (i+n)>=self.numValues:
                        pass
                    else:
                        if self.values[i+n] == -1:
                            frontierCellSet.add(i+n)

        return frontierCellSet
    def frontierZones(self):
        visitedSet = self.frontierCells
        zones = []

        while visitedSet:
            current = visitedSet.pop()
            zone = [current]
            q = [current]
            while q:
                current = q.pop()
                if current % self.width == 0:
                    neighbors = [-self.width,-self.width+1,1,self.width,self.width+1]
                elif (current-1) % self.width == 0: 
                    neighbors = [-self.width-1,-self.width,-1,self.width-1,self.width]
                else:
                    neighbors = [-self.width-1,-self.width,-self.width+1,-1,1,self.width-1,self.width,self.width+1]

                for n in neighbors:
                    if (current+n)>=0 and (current+n) < self.numValues and (current+n) in visitedSet:
                        q.append(current+n)
                        visitedSet.remove(current+n)
                        zone.append(current+n)
            zones.append(zone)
    def getPath(self):
        lowestTarget = np.inf
        lowestScore = np.inf
        bestPath = []
        for zone in self.frontierZones:
            target = np.random.choice(zone)
            #A-star to find distance from current to target
            result = AStarPlanner(self,self.costmap,self.start,target)
            cost = result.total_cost
            size = len(zone)

            #score to find best frontier zone
            score = cost * (1/size)
            if score < lowestScore:
                lowestTarget = target
                lowestScore = score
                bestPath = result.path

        return bestPath
            
        









            