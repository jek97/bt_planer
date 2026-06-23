#!/usr/bin/env python3

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


# ---------------------------------------------
#  Environment parsing  (grid[y][x]: 0=free, 1=obstacle)
# ---------------------------------------------
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


# ---------------------------------------------
#  BT leaf: MoveCommand  (StatefulActionNode equivalent)
# ---------------------------------------------
class MoveCommand(py_trees.behaviour.Behaviour):
    def __init__(self, name, command, node, client):
        super().__init__(name)
        self._command = command
        self._node    = node
        self._client  = client
        # futures reset on each initialise()
        self._send_future   = None
        self._goal_handle   = None
        self._result_future = None

    def initialise(self):
        """Called once when the node transitions to RUNNING."""
        self._goal_handle   = None
        self._result_future = None
        goal = Move.Goal()
        goal.command = self._command
        self._send_future = self._client.send_goal_async(goal)
        self._node.get_logger().info(f'[BT] Sending: {self._command!r}')

    def update(self):
        """Called every tick while RUNNING — never blocks."""
        # Phase 1: wait for server to accept/reject the goal
        if self._goal_handle is None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            self._goal_handle = self._send_future.result()
            if not self._goal_handle.accepted:
                self._node.get_logger().error(
                    f'Goal {self._command!r} rejected.')
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # Phase 2: wait for the action to complete
        if not self._result_future.done():
            return py_trees.common.Status.RUNNING

        res = self._result_future.result().result
        self._node.get_logger().info(
            f'[BT] {self._command!r} -> '
            f'x:{res.x} y:{res.y} theta:{res.theta} success:{res.success}'
        )
        return (py_trees.common.Status.SUCCESS if res.success
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        """Cancel in-flight goal if the BT halts this node."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


# ---------------------------------------------
#  BT tree definition
# ---------------------------------------------
def build_tree(node, client):
    sequence = py_trees.composites.Sequence(name='MoveSequence', memory=True)
    for name, cmd in [
        ('Straight1',  'straight'),
        ('TurnRight',  'turn_right'),
        ('Straight2',  'straight'),
        ('TurnLeft',   'turn_left'),
        ('Straight3',  'straight'),
    ]:
        sequence.add_child(MoveCommand(name, cmd, node, client))
    return sequence


# ---------------------------------------------
#  Robot ROS2 node
# ---------------------------------------------
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


# ---------------------------------------------
#  main
# ---------------------------------------------
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

    # Spin ROS2 in a background thread so BT ticks are non-blocking
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.wait_for_server()

    sequence = build_tree(node, node.client)
    bt = py_trees.trees.BehaviourTree(root=sequence)

    node.get_logger().info('Behavior tree running...')

    while rclpy.ok():
        bt.tick()
        status = sequence.status

        if status == py_trees.common.Status.FAILURE:
            node.get_logger().warn('Sequence failed — stopping.')
            break

        if status == py_trees.common.Status.SUCCESS:
            node.get_logger().info('Sequence complete, repeating...')
            # Reset all children so initialise() is called again next tick
            sequence.stop(py_trees.common.Status.INVALID)

        time.sleep(0.05)  # 20 Hz tick rate

    rclpy.shutdown()


if __name__ == '__main__':
    main()
