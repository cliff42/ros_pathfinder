from dataclasses import dataclass
from typing import Optional

from ros_pathfinder.planning.costmap import Costmap2d
from ros_pathfinder.planning.base_planner import GridCell

@dataclass
class AStarResult:
    path: list[GridCell]
    total_cost: float
    expanded_nodes: int


class AStarPlanner:
    def plan(
        self,
        costmap: Costmap2d,
        start: GridCell,
        goal: GridCell,
    ) -> Optional[AStarResult]:
        pass