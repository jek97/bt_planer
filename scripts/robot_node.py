#!/usr/bin/env python3

import heapq
import json
import os
import sys
import threading
import time

import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from bt_planner.action import Move


# =============================================================================
#  Environment parsing  –  grid[y][x]: 0 = free, 1 = obstacle
# =============================================================================
def parse_environment(path):
    with open(path) as f:
        data = json.load(f)

    size_x = data['size']['x']
    size_y = data['size']['y']
    grid = [[0] * size_x for _ in range(size_y)]

    for obs in data['obstacles']:
        ox, oy = obs['x'], obs['y']
        if 0 <= ox < size_x and 0 <= oy < size_y:
            grid[oy][ox] = 1

    return size_x, size_y, grid


# =============================================================================
#  A*  –  state space is (x, y, theta) so turning cost is minimised too.
#
#  start : (x, y, theta)
#  goal  : (x, y, theta)
#
#  Returns list[str] of action commands, or None if unreachable.
#  Each action costs 1; heuristic = Manhattan distance (admissible).
# =============================================================================
_THETA_TO_DIR = {0: (0, 1), 90: (1, 0), 180: (0, -1), 270: (-1, 0)}


def _a_star(grid, size_x, size_y, start, goal):
    import itertools
    _seq = itertools.count()          # unique tie-breaker for the heap

    def h(state):
        # Manhattan distance on (x, y) – ignoring theta is still admissible
        return abs(state[0] - goal[0]) + abs(state[1] - goal[1])

    open_heap = [(h(start), next(_seq), start)]
    came_from = {}          # state -> (parent_state, action_taken)
    g         = {start: 0}
    closed    = set()       # states whose optimal cost is known

    while open_heap:
        _, _, cur = heapq.heappop(open_heap)

        if cur in closed:   # stale heap entry – skip
            continue
        closed.add(cur)

        if cur == goal:
            # Reconstruct action sequence
            actions = []
            while cur in came_from:
                cur, action = came_from[cur]
                actions.append(action)
            actions.reverse()
            return actions

        x, y, theta = cur

        # ── neighbour 1: turn_left ────────────────────────────────────────
        nbr = (x, y, (theta - 90) % 360)
        ng  = g[cur] + 1
        if nbr not in closed and ng < g.get(nbr, float('inf')):
            g[nbr] = ng
            came_from[nbr] = (cur, 'turn_left')
            heapq.heappush(open_heap, (ng + h(nbr), next(_seq), nbr))

        # ── neighbour 2: turn_right ───────────────────────────────────────
        nbr = (x, y, (theta + 90) % 360)
        ng  = g[cur] + 1
        if nbr not in closed and ng < g.get(nbr, float('inf')):
            g[nbr] = ng
            came_from[nbr] = (cur, 'turn_right')
            heapq.heappush(open_heap, (ng + h(nbr), next(_seq), nbr))

        # ── neighbour 3: straight ─────────────────────────────────────────
        dx, dy = _THETA_TO_DIR[theta]
        nx, ny = x + dx, y + dy
        if 0 <= nx < size_x and 0 <= ny < size_y and grid[ny][nx] == 0:
            nbr = (nx, ny, theta)
            ng  = g[cur] + 1
            if nbr not in closed and ng < g.get(nbr, float('inf')):
                g[nbr] = ng
                came_from[nbr] = (cur, 'straight')
                heapq.heappush(open_heap, (ng + h(nbr), next(_seq), nbr))

    return None  # no path found


# =============================================================================
#  Straight-line path helpers
# =============================================================================
_DELTA_TO_THETA = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}


def _bresenham_line(x0, y0, x1, y1):
    """
    Returns a 4-connected cell path along the straight line from
    (x0,y0) to (x1,y1) using Bresenham's algorithm.
    Diagonal steps are resolved by inserting an x-first intermediate cell.
    """
    # Generate raw Bresenham cells (may contain diagonal steps)
    raw = [(x0, y0)]
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    err = dx - dy
    x, y = x0, y0
    while (x, y) != (x1, y1):
        e2 = 2 * err
        step_x = step_y = False
        if e2 > -dy:
            err -= dy
            x += sx
            step_x = True
        if e2 < dx:
            err += dx
            y += sy
            step_y = True
        if step_x and step_y:
            raw.append((x - sx, y))   # x-first intermediate
        raw.append((x, y))

    # Deduplicate while preserving order
    seen = set()
    path = []
    for cell in raw:
        if cell not in seen:
            seen.add(cell)
            path.append(cell)
    return path


def _path_to_actions(path, start_theta):
    """Convert a list of 4-connected (x,y) waypoints to action strings."""
    actions = []
    theta = start_theta
    for i in range(len(path) - 1):
        cx, cy = path[i]
        nx, ny = path[i + 1]
        target_theta = _DELTA_TO_THETA[(nx - cx, ny - cy)]
        diff = (target_theta - theta) % 360
        if diff == 90:
            actions.append('turn_right')
        elif diff == 180:
            actions.append('turn_right')
            actions.append('turn_right')
        elif diff == 270:
            actions.append('turn_left')
        theta = target_theta
        actions.append('straight')
    return actions


# =============================================================================
#  go2Straight
#  -----------
#  Plans the path as the straight Bresenham line from start to goal,
#  completely ignoring obstacles in the map.
#
#  robot_x, robot_y, robot_theta  – current pose
#  goal_x,  goal_y                – target position (orientation ignored)
#  sim                            – False: return action list only
#                                   True:  execute in the simulator
#
#  Returns list[str] (action sequence).  Never returns None (line always exists).
# =============================================================================
def go2Straight(robot_x, robot_y, robot_theta,
                goal_x, goal_y,
                sim,
                node, client,
                grid, size_x, size_y):

    log = node.get_logger()
    log.info(f'go2Straight  start=({robot_x},{robot_y},θ={robot_theta})  '
             f'goal=({goal_x},{goal_y})')

    path    = _bresenham_line(robot_x, robot_y, goal_x, goal_y)
    actions = _path_to_actions(path, robot_theta)
    log.info(f'go2Straight: {len(path)-1} cells, {len(actions)} actions → {actions}')

    if not sim:
        return actions

    for cmd in actions:
        log.info(f'go2Straight: executing {cmd!r}')
        if not _execute_one(cmd, client, log):
            log.warn(f'go2Straight: {cmd!r} failed (obstacle hit) – aborting.')
            return actions

    log.info('go2Straight: goal reached.')
    return actions


# =============================================================================
#  BT action node: Go2StraightBehaviour
#  Reads robot_pose=(x,y,theta) and goal_pos=(x,y) from the blackboard.
#  goal orientation is not used (straight line ignores it).
#  Always runs with sim=True.  Non-blocking.
# =============================================================================
class Go2StraightBehaviour(py_trees.behaviour.Behaviour):

    def __init__(self, name, node, client, grid, size_x, size_y):
        super().__init__(name)
        self._node    = node
        self._client  = client
        self._grid    = grid
        self._size_x  = size_x
        self._size_y  = size_y

        bb = self.attach_blackboard_client(name=name)
        bb.register_key(key='robot_pose', access=py_trees.common.Access.READ)
        bb.register_key(key='goal_pos',   access=py_trees.common.Access.READ)  # (x, y)
        self._bb = bb

        self._actions       = None
        self._action_idx    = 0
        self._send_future   = None
        self._goal_handle   = None
        self._result_future = None

    def initialise(self):
        robot_pose = self._bb.robot_pose   # (x, y, theta)
        goal_pos   = self._bb.goal_pos     # (x, y)
        rx, ry, rtheta = robot_pose
        gx, gy         = goal_pos

        path             = _bresenham_line(rx, ry, gx, gy)
        self._actions    = _path_to_actions(path, rtheta)
        self._action_idx = 0
        self._node.get_logger().info(
            f'Go2Straight BT: {len(self._actions)} actions planned.')
        self._fire_next()

    def _fire_next(self):
        goal = Move.Goal()
        goal.command = self._actions[self._action_idx]
        self._send_future   = self._client.send_goal_async(goal)
        self._goal_handle   = None
        self._result_future = None

    def update(self):
        if self._action_idx >= len(self._actions):
            return py_trees.common.Status.SUCCESS

        # Phase 1 – wait for goal acceptance
        if self._goal_handle is None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            self._goal_handle = self._send_future.result()
            if not self._goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # Phase 2 – wait for result
        if not self._result_future.done():
            return py_trees.common.Status.RUNNING

        res = self._result_future.result().result
        if not res.success:
            self._node.get_logger().warn(
                f'Go2Straight BT: {self._actions[self._action_idx]!r} blocked.')
            return py_trees.common.Status.FAILURE

        self._action_idx += 1
        if self._action_idx < len(self._actions):
            self._fire_next()
            return py_trees.common.Status.RUNNING

        self._node.get_logger().info('Go2Straight BT: goal reached.')
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


# =============================================================================
#  Low-level helper: send one action and block until result.
#  Safe to call from any thread while the background executor is running.
# =============================================================================
def _execute_one(command, client, logger):
    goal = Move.Goal()
    goal.command = command
    send_future = client.send_goal_async(goal)

    while not send_future.done():
        time.sleep(0.01)

    goal_handle = send_future.result()
    if not goal_handle.accepted:
        logger.error(f'_execute_one: goal {command!r} rejected.')
        return False

    result_future = goal_handle.get_result_async()
    while not result_future.done():
        time.sleep(0.01)

    return result_future.result().result.success


# =============================================================================
#  go2A
#  -----
#  robot_x, robot_y, robot_theta  – current pose (theta ∈ {0,90,180,270})
#  goal_x,  goal_y,  goal_theta   – target pose
#  sim                            – False: return action list only
#                                   True:  also execute in the simulator
#  grid, size_x, size_y           – occupancy map
#
#  Returns list[str] (action sequence) or None if no path exists.
# =============================================================================
def go2A(robot_x, robot_y, robot_theta,
         goal_x, goal_y, goal_theta,
         sim,
         node, client,
         grid, size_x, size_y):

    log = node.get_logger()
    log.info(f'go2A  start=({robot_x},{robot_y},θ={robot_theta})  '
             f'goal=({goal_x},{goal_y},θ={goal_theta})')

    actions = _a_star(grid, size_x, size_y,
                      (robot_x, robot_y, robot_theta),
                      (goal_x,  goal_y,  goal_theta))
    if actions is None:
        log.error('go2A: A* found no path.')
        return None

    log.info(f'go2A: {len(actions)} actions → {actions}')

    if not sim:
        return actions

    for cmd in actions:
        log.info(f'go2A: executing {cmd!r}')
        if not _execute_one(cmd, client, log):
            log.warn(f'go2A: {cmd!r} failed – aborting.')
            return actions

    log.info('go2A: goal reached.')
    return actions


# =============================================================================
#  BT action node: Go2ABehaviour
#  Reads robot_pose=(x,y,theta) and goal_pose=(x,y) from the blackboard.
#  Always runs with sim=True (executes every action in the simulator).
#  Non-blocking: fires one ROS action at a time and polls futures.
# =============================================================================
class Go2ABehaviour(py_trees.behaviour.Behaviour):

    def __init__(self, name, node, client, grid, size_x, size_y):
        super().__init__(name)
        self._node    = node
        self._client  = client
        self._grid    = grid
        self._size_x  = size_x
        self._size_y  = size_y

        bb = self.attach_blackboard_client(name=name)
        bb.register_key(key='robot_pose', access=py_trees.common.Access.READ)
        bb.register_key(key='goal_pose',  access=py_trees.common.Access.READ)
        self._bb = bb

        self._actions       = None
        self._action_idx    = 0
        self._send_future   = None
        self._goal_handle   = None
        self._result_future = None

    def initialise(self):
        robot_pose = self._bb.robot_pose   # (x, y, theta)
        goal_pose  = self._bb.goal_pose    # (x, y, theta)
        rx, ry, rtheta   = robot_pose
        gx, gy, gtheta   = goal_pose

        actions = _a_star(self._grid, self._size_x, self._size_y,
                          (rx, ry, rtheta), (gx, gy, gtheta))
        if actions is None:
            self._node.get_logger().error('Go2A BT: no path found.')
            self._actions = None
            return

        self._actions    = actions
        self._action_idx = 0
        self._node.get_logger().info(
            f'Go2A BT: plan ready – {len(self._actions)} actions.')
        self._fire_next()

    def _fire_next(self):
        goal = Move.Goal()
        goal.command = self._actions[self._action_idx]
        self._send_future   = self._client.send_goal_async(goal)
        self._goal_handle   = None
        self._result_future = None

    def update(self):
        if self._actions is None:
            return py_trees.common.Status.FAILURE

        if self._action_idx >= len(self._actions):
            return py_trees.common.Status.SUCCESS

        # Phase 1 – wait for goal acceptance
        if self._goal_handle is None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            self._goal_handle = self._send_future.result()
            if not self._goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # Phase 2 – wait for result
        if not self._result_future.done():
            return py_trees.common.Status.RUNNING

        res = self._result_future.result().result
        if not res.success:
            self._node.get_logger().warn(
                f'Go2A BT: {self._actions[self._action_idx]!r} failed.')
            return py_trees.common.Status.FAILURE

        self._action_idx += 1
        if self._action_idx < len(self._actions):
            self._fire_next()
            return py_trees.common.Status.RUNNING

        self._node.get_logger().info('Go2A BT: goal reached.')
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


# =============================================================================
#  Old fixed-sequence BT  (commented out – kept for reference)
# =============================================================================
# class MoveCommand(py_trees.behaviour.Behaviour):
#     def __init__(self, name, command, node, client):
#         super().__init__(name)
#         self._command = command
#         self._node    = node
#         self._client  = client
#         self._send_future = self._goal_handle = self._result_future = None
#
#     def initialise(self):
#         self._goal_handle = self._result_future = None
#         goal = Move.Goal()
#         goal.command = self._command
#         self._send_future = self._client.send_goal_async(goal)
#
#     def update(self):
#         if self._goal_handle is None:
#             if not self._send_future.done():
#                 return py_trees.common.Status.RUNNING
#             self._goal_handle = self._send_future.result()
#             if not self._goal_handle.accepted:
#                 return py_trees.common.Status.FAILURE
#             self._result_future = self._goal_handle.get_result_async()
#             return py_trees.common.Status.RUNNING
#         if not self._result_future.done():
#             return py_trees.common.Status.RUNNING
#         res = self._result_future.result().result
#         return (py_trees.common.Status.SUCCESS if res.success
#                 else py_trees.common.Status.FAILURE)
#
#     def terminate(self, new_status):
#         if self._goal_handle is not None:
#             self._goal_handle.cancel_goal_async()
#
#
# def build_tree(node, client):
#     sequence = py_trees.composites.Sequence(name='MoveSequence', memory=True)
#     for name, cmd in [
#         ('Straight1', 'straight'), ('TurnRight', 'turn_right'),
#         ('Straight2', 'straight'), ('TurnLeft',  'turn_left'),
#         ('Straight3', 'straight'),
#     ]:
#         sequence.add_child(MoveCommand(name, cmd, node, client))
#     return sequence


# =============================================================================
#  Robot ROS2 node
# =============================================================================
class RobotNode(Node):
    def __init__(self):
        super().__init__('robot')
        self._client = ActionClient(self, Move, 'move')

    def wait_for_server(self):
        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()
        self.get_logger().info('Action server ready.')

    @property
    def client(self):
        return self._client


# =============================================================================
#  main
# =============================================================================
def main():
    rclpy.init()

    env_path = (
        sys.argv[1] if len(sys.argv) > 1
        else os.path.join(
            get_package_share_directory('bt_planner'),
            'environment', 'env.json',
        )
    )

    size_x, size_y, grid = parse_environment(env_path)

    node = RobotNode()
    node.get_logger().info(f'Map loaded: {size_x}x{size_y}')
    for row in reversed(range(size_y)):
        node.get_logger().info(
            ''.join('X' if grid[row][col] else '.' for col in range(size_x))
        )

    # Spin ROS2 in a background thread so futures are resolved without blocking
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.wait_for_server()

    # -------------------------------------------------------------------------
    #  Run go2Straight with sim=True
    #  Start: (0, 0, theta=0)   Goal: (9, 9)
    # -------------------------------------------------------------------------
    go2Straight(
        robot_x=0, robot_y=0, robot_theta=0,
        goal_x=9,  goal_y=9,
        sim=True,
        node=node, client=node.client,
        grid=grid, size_x=size_x, size_y=size_y,
    )

    # -------------------------------------------------------------------------
    #  go2A  (commented out – use go2Straight above for now)
    # -------------------------------------------------------------------------
    # go2A(
    #     robot_x=0, robot_y=0, robot_theta=0,
    #     goal_x=9,  goal_y=9,  goal_theta=0,
    #     sim=True,
    #     node=node, client=node.client,
    #     grid=grid, size_x=size_x, size_y=size_y,
    # )

    # -------------------------------------------------------------------------
    #  BT with Go2StraightBehaviour wired to blackboard – ready, not active.
    #  Uncomment to drive straight-line navigation through the behaviour tree.
    # -------------------------------------------------------------------------
    # bb = py_trees.blackboard.Client(name='main')
    # bb.register_key(key='robot_pose', access=py_trees.common.Access.WRITE)
    # bb.register_key(key='goal_pos',   access=py_trees.common.Access.WRITE)
    # bb.robot_pose = (0, 0, 0)
    # bb.goal_pos   = (9, 9)
    #
    # go2s_node = Go2StraightBehaviour(
    #     'Go2Straight', node, node.client, grid, size_x, size_y)
    # bt = py_trees.trees.BehaviourTree(root=go2s_node)
    # while rclpy.ok():
    #     bt.tick()
    #     if go2s_node.status in (py_trees.common.Status.SUCCESS,
    #                             py_trees.common.Status.FAILURE):
    #         break
    #     time.sleep(0.05)

    # -------------------------------------------------------------------------
    #  BT with Go2ABehaviour – ready, not active.
    # -------------------------------------------------------------------------
    # bb = py_trees.blackboard.Client(name='main')
    # bb.register_key(key='robot_pose', access=py_trees.common.Access.WRITE)
    # bb.register_key(key='goal_pose',  access=py_trees.common.Access.WRITE)
    # bb.robot_pose = (0, 0, 0)
    # bb.goal_pose  = (9, 9, 0)   # (x, y, theta)
    #
    # go2a_node = Go2ABehaviour('Go2A', node, node.client, grid, size_x, size_y)
    # bt = py_trees.trees.BehaviourTree(root=go2a_node)
    # while rclpy.ok():
    #     bt.tick()
    #     if go2a_node.status in (py_trees.common.Status.SUCCESS,
    #                             py_trees.common.Status.FAILURE):
    #         break
    #     time.sleep(0.05)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
