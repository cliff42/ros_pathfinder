import math

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ros_pathfinder.planning.base_planner import GridCell
from ros_pathfinder.planning.a_star import AStarPlanner
from ros_pathfinder.planning.costmap import Costmap2d, OBSTACLE, UNKNOWN


@dataclass(frozen=True)
class ExplorationPlan:
    path: list[GridCell]
    goal: GridCell
    goal_yaw_grid_rad: Optional[float]
    frontier_size: int
    travel_cost: float
    score: float


class FrontierPlanner:
    _CLUSTER_NEIGHBORS = (
        (-1, -1),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    )

    def __init__(
        self,
        minimum_frontier_size_cells: int = 5,
        minimum_frontier_distance_m: float = 0.30,
        frontier_size_weight: float = 1.0,
    ) -> None:
        if minimum_frontier_size_cells <= 0:
            raise ValueError("minimum_frontier_size_cells must be positive")
        if minimum_frontier_distance_m < 0.0:
            raise ValueError(
                "minimum_frontier_distance_m cannot be negative"
            )
        if frontier_size_weight < 0.0:
            raise ValueError("frontier_size_weight cannot be negative")

        self._minimum_frontier_size_cells = minimum_frontier_size_cells
        self._minimum_frontier_distance_m = minimum_frontier_distance_m
        self._frontier_size_weight = frontier_size_weight
        self._path_planner = AStarPlanner()

    def plan(
        self,
        costmap: Costmap2d,
        start: GridCell,
    ) -> Optional[ExplorationPlan]:
        if not costmap.in_bounds(*start):
            raise ValueError(f"start cell {start} is outside the costmap")

        known_costmap = costmap.with_allow_unknown(False)
        if not known_costmap.is_traversable(*start):
            return None

        frontier_cells = self.find_frontier_cells(known_costmap)
        candidates = []

        for zone in self.cluster_frontier_cells(frontier_cells):
            if len(zone) < self._minimum_frontier_size_cells:
                continue

            goal = self._representative_cell(zone)
            goal_distance_m = math.hypot(
                goal[0] - start[0],
                goal[1] - start[1],
            ) * known_costmap.resolution_m
            if goal_distance_m < self._minimum_frontier_distance_m:
                continue

            zone_distance_cells = min(
                math.hypot(cell[0] - start[0], cell[1] - start[1])
                for cell in zone
            )
            candidates.append((zone_distance_cells, goal, zone))

        candidates.sort(
            key=lambda candidate: (
                candidate[0],
                -len(candidate[2]),
                candidate[1],
            ),
            reverse=True,
        )

        for _, goal, zone in candidates:
            result = self._path_planner.plan(
                costmap=known_costmap,
                start=start,
                goal=goal,
            )
            if result is None or not result.path:
                continue

            score = result.total_cost / (
                len(zone) ** self._frontier_size_weight
            )
            return ExplorationPlan(
                path=list(result.path),
                goal=goal,
                goal_yaw_grid_rad=self._unknown_direction(
                    known_costmap,
                    goal,
                ),
                frontier_size=len(zone),
                travel_cost=result.total_cost,
                score=score,
            )

        return None

    @classmethod
    def find_frontier_cells(cls, costmap: Costmap2d) -> set[GridCell]:
        values = costmap.values
        unknown = values == UNKNOWN
        known_traversable = (values != UNKNOWN) & (values < OBSTACLE)
        adjacent_to_unknown = np.zeros(values.shape, dtype=bool)

        adjacent_to_unknown[1:, :] |= unknown[:-1, :]
        adjacent_to_unknown[:-1, :] |= unknown[1:, :]
        adjacent_to_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_to_unknown[:, :-1] |= unknown[:, 1:]

        frontier_y, frontier_x = np.nonzero(
            known_traversable & adjacent_to_unknown
        )
        return set(zip(frontier_x.tolist(), frontier_y.tolist()))

    @classmethod
    def cluster_frontier_cells(
        cls,
        frontier_cells: set[GridCell],
    ) -> list[tuple[GridCell, ...]]:
        unvisited = set(frontier_cells)
        zones: list[tuple[GridCell, ...]] = []

        while unvisited:
            first = min(unvisited)
            unvisited.remove(first)
            pending = [first]
            zone = []

            while pending:
                current = pending.pop()
                zone.append(current)
                current_x, current_y = current

                for dx, dy in cls._CLUSTER_NEIGHBORS:
                    neighbor = (current_x + dx, current_y + dy)
                    if neighbor not in unvisited:
                        continue
                    unvisited.remove(neighbor)
                    pending.append(neighbor)

            zones.append(tuple(sorted(zone)))

        return zones

    @staticmethod
    def _representative_cell(zone: tuple[GridCell, ...]) -> GridCell:
        center_x = sum(cell[0] for cell in zone) / len(zone)
        center_y = sum(cell[1] for cell in zone) / len(zone)
        return min(
            zone,
            key=lambda cell: (
                (cell[0] - center_x) ** 2 + (cell[1] - center_y) ** 2,
                cell,
            ),
        )

    @classmethod
    def _unknown_direction(
        cls,
        costmap: Costmap2d,
        goal: GridCell,
    ) -> Optional[float]:
        goal_x, goal_y = goal
        directions = [
            (dx, dy)
            for dx, dy in cls._CLUSTER_NEIGHBORS
            if costmap.in_bounds(goal_x + dx, goal_y + dy)
            and int(costmap.values[goal_y + dy, goal_x + dx]) == UNKNOWN
        ]
        if not directions:
            return None

        direction_x = sum(direction[0] for direction in directions)
        direction_y = sum(direction[1] for direction in directions)
        if direction_x == 0 and direction_y == 0:
            direction_x, direction_y = directions[0]

        return math.atan2(direction_y, direction_x)
