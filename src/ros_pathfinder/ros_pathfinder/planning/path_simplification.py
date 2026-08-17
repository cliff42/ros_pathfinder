from collections.abc import Sequence

from ros_pathfinder.planning.base_planner import GridCell
from ros_pathfinder.planning.costmap import Costmap2d


def grid_line_is_traversable(
    costmap: Costmap2d,
    start: GridCell,
    end: GridCell,
) -> bool:
    cells = list(_bresenham_cells(start, end))

    for x, y in cells:
        if not costmap.is_traversable(x, y):
            return False

    # do not allow a diagonal transition between two blocked cardinal cells
    for (x0, y0), (x1, y1) in zip(cells, cells[1:]):
        if x0 == x1 or y0 == y1:
            continue
        if (
            not costmap.is_traversable(x1, y0)
            or not costmap.is_traversable(x0, y1)
        ):
            return False

    return True


def simplify_grid_path(
    costmap: Costmap2d,
    path: Sequence[GridCell],
) -> list[GridCell]:
    if not path:
        return []

    deduplicated = [path[0]]
    for cell in path[1:]:
        if cell != deduplicated[-1]:
            deduplicated.append(cell)

    if len(deduplicated) <= 2:
        return deduplicated

    simplified = [deduplicated[0]]
    anchor_index = 0
    final_index = len(deduplicated) - 1

    while anchor_index < final_index:
        next_index = final_index
        while next_index > anchor_index + 1:
            if grid_line_is_traversable(
                costmap,
                deduplicated[anchor_index],
                deduplicated[next_index],
            ):
                break
            next_index -= 1

        if not grid_line_is_traversable(
            costmap,
            deduplicated[anchor_index],
            deduplicated[next_index],
        ):
            raise ValueError("input path contains a blocked segment")

        simplified.append(deduplicated[next_index])
        anchor_index = next_index

    return simplified


def _bresenham_cells(start: GridCell, end: GridCell):
    x, y = start
    end_x, end_y = end

    dx = abs(end_x - x)
    dy = abs(end_y - y)
    step_x = 1 if x < end_x else -1
    step_y = 1 if y < end_y else -1
    error = dx - dy

    while True:
        yield x, y
        if x == end_x and y == end_y:
            return

        doubled_error = 2 * error
        if doubled_error > -dy:
            error -= dy
            x += step_x
        if doubled_error < dx:
            error += dx
            y += step_y
