import heapq
import math

from collections.abc import Iterator
from dataclasses import dataclass
from typing import Optional

from ros_pathfinder.planning.base_planner import GridCell
from ros_pathfinder.planning.costmap import Costmap2d


@dataclass
class AStarResult:
    path: list[GridCell]
    total_cost: float
    expanded_nodes: int


class AStarPlanner:
    _NEIGHBORS = (
        (-1, -1, math.sqrt(2.0)),
        (0, -1, 1.0),
        (1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)),
        (0, 1, 1.0),
        (1, 1, math.sqrt(2.0)),
    )

    def plan(
        self,
        costmap: Costmap2d,
        start: GridCell,
        goal: GridCell,
    ) -> Optional[AStarResult]:
        self._validate_cell(costmap, start, "start")
        self._validate_cell(costmap, goal, "goal")

        if not costmap.is_traversable(*start):
            return None
        if not costmap.is_traversable(*goal):
            return None

        open_heap = [(self._heuristic(start, goal), start)]
        came_from: dict[GridCell, GridCell] = {}
        distance_from_start: dict[GridCell, float] = {start: 0.0}
        expanded: set[GridCell] = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in expanded:
                continue

            expanded.add(current)
            if current == goal:
                return AStarResult(
                    path=self._reconstruct_path(came_from, start, goal),
                    total_cost=distance_from_start[goal],
                    expanded_nodes=len(expanded),
                )

            current_cost = distance_from_start[current]
            for neighbor, movement_cost in self._neighbors(costmap, current):
                tentative_cost = (
                    current_cost
                    + movement_cost
                    * costmap.traversal_multiplier(*neighbor)
                )
                known_cost = distance_from_start.get(neighbor, math.inf)
                if tentative_cost >= known_cost:
                    continue

                came_from[neighbor] = current
                distance_from_start[neighbor] = tentative_cost
                estimated_total_cost = (
                    tentative_cost + self._heuristic(neighbor, goal)
                )
                heapq.heappush(
                    open_heap,
                    (estimated_total_cost, neighbor),
                )

        return None

    def _neighbors(
        self,
        costmap: Costmap2d,
        current: GridCell,
    ) -> Iterator[tuple[GridCell, float]]:
        current_x, current_y = current

        for offset_x, offset_y, movement_cost in self._NEIGHBORS:
            neighbor = (current_x + offset_x, current_y + offset_y)
            if not costmap.is_traversable(*neighbor):
                continue

            if offset_x != 0 and offset_y != 0:
                horizontal = (current_x + offset_x, current_y)
                vertical = (current_x, current_y + offset_y)
                if (
                    not costmap.is_traversable(*horizontal)
                    or not costmap.is_traversable(*vertical)
                ):
                    continue

            yield neighbor, movement_cost

    @staticmethod
    def _heuristic(current: GridCell, goal: GridCell) -> float:
        return math.hypot(goal[0] - current[0], goal[1] - current[1])

    @staticmethod
    def _reconstruct_path(
        came_from: dict[GridCell, GridCell],
        start: GridCell,
        goal: GridCell,
    ) -> list[GridCell]:
        path = [goal]
        while path[-1] != start:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

    @staticmethod
    def _validate_cell(
        costmap: Costmap2d,
        cell: GridCell,
        name: str,
    ) -> None:
        if not costmap.in_bounds(*cell):
            raise ValueError(f"{name} cell {cell} is outside the costmap")
