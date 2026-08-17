from ros_pathfinder.planning.costmap import Costmap2d
from ros_pathfinder.planning.base_planner import GridCell
from ros_pathfinder.planning.a_star import AStarResult
from typing import Optional

import numpy as np
import math

import heapq
from collections import deque



class Djikstra():
    def __init__(self,start:GridCell,costmap:Costmap2d):
        self.width = costmap.width
        self.height = costmap.height
        self.num_values = costmap.width*costmap.height
        self.values = costmap.values

        self.start = self.width*start[1]+start[0]
        self.prev,self.dist = self.plan()
    def gridCell(self,current,width):
        x = current % width
        y = int(current/width)
        return GridCell[x,y] 
    def plan(self):
        LATERAL = 1
        DIAG = math.sqrt(2)

        cellSet = {self.start}
        
        prev = [None]*self.num_values
        dist = [np.inf]*self.num_values
        dist[self.start]=0
        q = [(dist[self.start],self.start)]
        
        heapq.heapify(q)
        for i,v in enumerate(self.values):
            if i != self.start and (v == 0 or v == -1):
                dist[i] == np.inf
                heapq.heappush(q,(dist[i],i))
                cellSet.add(i)
        while q:
            _,current = heapq.heappop(q)
            if current % self.width == 0:
                neighbors = [(-self.width,LATERAL),(-self.width+1,DIAG),(1,LATERAL),(self.width,LATERAL),(self.width+1,DIAG)]
            elif (current+1) % self.width == 0:
                neighbors = [(-self.width-1,DIAG),(-self.width,LATERAL),(-1,LATERAL),(self.width-1,DIAG),(self.width,LATERAL)]
            else:
                neighbors=[(-self.width-1,DIAG),(-self.width,LATERAL),(-self.width+1,DIAG),(-1,LATERAL),(1,LATERAL),(self.width-1,DIAG),(self.width,LATERAL),(self.width+1,DIAG)]
            for n,cost in neighbors:
                i = current + n
                if i >= self.num_values or i < 0 or i not in cellSet:
                    continue
                else:
                    new_dist = dist[current] + cost
                    if new_dist < dist[i]:
                        dist[i] = new_dist
                        prev[i] = current    
        return prev,dist
    def find_path(self,end)-> Optional[AStarResult] :
        self.end = self.width*end[1]+end[0]
        cost = self.dist[self.end]
        if self.dist[self.end] is not np.inf:
            path_list = deque([self.gridCell(self.end)])
            current = self.end
            while current != self.start:
                path_list.appendleft(self.gridCell(self.prev[current]))
                current = self.prev[current]
        else:
            path_list = None
        num_values = len(path_list)
        result = AStarResult(path_list,cost,num_values)
        return result