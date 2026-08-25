import math

from dataclasses import dataclass
from typing import Optional

from ros_pathfinder.planning.a_star import AStarPlanner
from ros_pathfinder.planning.base_planner import GridCell
from ros_pathfinder.planning.costmap import Costmap2d, UNKNOWN


@dataclass(frozen=True)
class ExplorationPlan:
    path: list[GridCell]
    goal: GridCell
    goal_yaw_grid_rad: Optional[float]
    frontier_size: int
    travel_cost: float
    score: float


class FrontierPlanner:
    _CARDINAL_NEIGHBORS = (
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
    )
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

        best_plan: Optional[ExplorationPlan] = None
        frontier_cells = self.find_frontier_cells(known_costmap)

        for zone in self.cluster_frontier_cells(frontier_cells, start):
            if len(zone) < self._minimum_frontier_size_cells:
                continue

            goal = self._representative_cell(zone)
            distance_m = math.hypot(
                goal[0] - start[0],
                goal[1] - start[1],
            ) * known_costmap.resolution_m
            if distance_m < self._minimum_frontier_distance_m:
                continue

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
            candidate = ExplorationPlan(
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
            if self._is_better(candidate, best_plan):
                best_plan = candidate

        return best_plan

    @classmethod
    def find_frontier_cells(cls, costmap: Costmap2d) -> set[GridCell]:
        frontiers: set[GridCell] = set()

        for y in range(costmap.height):
            for x in range(costmap.width):
                if not costmap.is_traversable(x, y):
                    continue
                if int(costmap.values[y, x]) == UNKNOWN:
                    continue

                if any(
                    costmap.in_bounds(x + dx, y + dy)
                    and int(costmap.values[y + dy, x + dx]) == UNKNOWN
                    for dx, dy in cls._CARDINAL_NEIGHBORS
                ):
                    frontiers.add((x, y))

        return frontiers

    @classmethod
    def cluster_frontier_cells(
        cls,
        frontier_cells: set[GridCell],
        start: GridCell
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

        cloest_zone = zones[0]
        min_dist = math.inf
        for zone in zones:
            zgc = zone[0]
            dist = math.sqrt((zgc[0] - start[0])**2  + (zgc[1] - start[1])**2)
            if dist < min_dist:
                min_dist = dist
                cloest_zone = zone

        return [cloest_zone]

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

    @staticmethod
    def _is_better(
        candidate: ExplorationPlan,
        current: Optional[ExplorationPlan],
    ) -> bool:
        if current is None:
            return True

        candidate_key = (
            candidate.score,
            candidate.travel_cost,
            -candidate.frontier_size,
            candidate.goal,
        )
        current_key = (
            current.score,
            current.travel_cost,
            -current.frontier_size,
            current.goal,
        )
        return candidate_key < current_key
