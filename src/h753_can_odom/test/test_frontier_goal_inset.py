from h753_can_odom.frontier_explorer_node import (
    find_inset_goal_cell,
    has_map_clearance,
)


def make_frontier_map(width=15, height=15, unknown_from_col=10):
    data = []
    for _row in range(height):
        for col in range(width):
            data.append(0 if col < unknown_from_col else -1)
    cluster = [row * width + unknown_from_col - 1 for row in range(3, height - 3)]
    return data, cluster


def test_frontier_goal_is_inset_from_unknown_boundary():
    width = 15
    height = 15
    data, cluster = make_frontier_map(width, height)

    goal = find_inset_goal_cell(
        cluster,
        data,
        width,
        height,
        clearance_cells=3,
        free_threshold=20,
    )

    assert goal is not None
    _row, col = divmod(goal, width)
    assert col <= 6
    assert goal not in cluster
    assert has_map_clearance(goal, data, width, height, 3, 20)


def test_unknown_cells_are_not_considered_clearance():
    width = 15
    height = 15
    data, _cluster = make_frontier_map(width, height)
    boundary_free_cell = 7 * width + 9

    assert not has_map_clearance(
        boundary_free_cell,
        data,
        width,
        height,
        clearance_cells=2,
        free_threshold=20,
    )


def test_map_edge_is_not_safe_for_robot_center():
    width = 15
    height = 15
    data = [0] * (width * height)

    assert not has_map_clearance(7 * width, data, width, height, 2, 20)
