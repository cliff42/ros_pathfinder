from abc import ABC

from ros_pathfinder.planning.costmap import Costmap2d

GridCell = tuple[int, int]

class PathPlanner(ABC):
    def plan(self, costmap: Costmap2d, start: GridCell, goal: GridCell) -> list[GridCell]:
        raise NotImplementedError