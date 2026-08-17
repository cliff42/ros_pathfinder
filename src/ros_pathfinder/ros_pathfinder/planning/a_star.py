from dataclasses import dataclass
from typing import Optional

from ros_pathfinder.planning.costmap import Costmap2d
from ros_pathfinder.planning.base_planner import GridCell

import math
import numpy as np
import heapq
from collections import deque

@dataclass
class AStarResult:
    path: list[GridCell]
    total_cost: float
    expanded_nodes: int


class AStarPlanner:
    def heuristic(self,current,goal,width):
        x1 = current % width
        x2 = goal % width
        y1 = int(current/width)
        y2 = int(goal/width)
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    def gridCell(self,current,width):
        x = current % width
        y = int(current/width)
        return GridCell[x,y] 

    def plan(
        self,
        costmap: Costmap2d,
        start: GridCell,
        goal: GridCell,
    ) -> Optional[AStarResult]:
        LATERAL = 1
        DIAG = math.sqrt(2)

        width = costmap.width
        height = costmap.height
        num_cells = width*height

        self.start = width*start[1] + start[0]
        self.goal = width*goal[1] + goal[0]

        cellSet = {self.start}
                
        prev = [None]*width*height
        dist = [np.inf]*width*height
        totalCost = [np.inf]*width*height

        dist[self.start]=0
        totalCost[self.start] = dist[self.start] + self.heuristic(self.start,self.goal,width)

        q = [(totalCost[self.start],self.start)]
        
        heapq.heapify(q)
        for i,v in enumerate(costmap.values):
            if i != self.start and v == 0:
                dist[i] == np.inf
                heapq.heappush(q,(dist[i],i))
                cellSet.add(i)
        while q:
            _,current = heapq.heappop(q)
            if current == 0 or current % width == 0:
                neighbors = [(-width,LATERAL),(-width+1,DIAG),(1,LATERAL),(width,LATERAL),(width+1,DIAG)]
            elif current % (width-1) == 0:
                neighbors = [(-width-1,DIAG),(-width,LATERAL),(-1,LATERAL),(width-1,DIAG),(width,LATERAL)]
            else:
                neighbors = [(-width-1,DIAG),(-width,LATERAL),(-width+1,DIAG),(-1,LATERAL),(1,LATERAL),(width-1,DIAG),(width,LATERAL),(width+1,DIAG)]
            if current == self.goal:
                break
            for n,cost in neighbors:
                i = current + n
                if i >= num_cells or i < 0 or i not in cellSet:
                    continue
                else:
                    new_dist = dist[current] + cost
                    if new_dist < dist[i]:
                        dist[i] = new_dist
                        totalCost[i] = new_dist + self.heuristic(i,self.goal,width)
                        prev[i] = current

        
        path_list = deque([self.gridCell(current,width)])
        while current != self.start:
            path_list.appendleft(self.gridCell(prev[current],width))
            current = prev[current]
        total_cost = totalCost[self.goal]
        numNodes = len(path_list)
        result = AStarResult(path_list,total_cost,numNodes)
        return result