#!/usr/bin/env python3

import json
import math
import os
import sys
import threading
import time

import pygame
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from bt_planner.action import Move

WINDOW_W = 1000
WINDOW_H = 1000


# ---------------------------------------------
#  Environment parsing
# ---------------------------------------------
def parse_environment(path):
    with open(path) as f:
        data = json.load(f)
    return {
        'size_x':    data['size']['x'],
        'size_y':    data['size']['y'],
        'obstacles': {(o['x'], o['y']) for o in data['obstacles']},
    }


# ---------------------------------------------
#  Pygame drawing helpers
# ---------------------------------------------
def _draw_arrow(surface, cx, cy, theta, size):
    if theta == 0:
        angle = -math.pi / 2   # up
    elif theta == 90:
        angle = 0.0             # right
    elif theta == 180:
        angle = math.pi / 2    # down
    else:
        angle = math.pi        # left (270)

    dx, dy = math.cos(angle), math.sin(angle)
    px, py_ = -dy, dx          # perpendicular

    tip   = (cx + dx * size,            cy + dy * size)
    base  = (cx - dx * size * 0.4,      cy - dy * size * 0.4)
    left  = (base[0] + px * size * 0.35, base[1] + py_ * size * 0.35)
    right = (base[0] - px * size * 0.35, base[1] - py_ * size * 0.35)

    pygame.draw.polygon(surface, (255, 255, 255), [tip, left, right])


def draw_environment(surface, env, robot):
    size_x, size_y = env['size_x'], env['size_y']
    cell_w = WINDOW_W // size_x
    cell_h = WINDOW_H // size_y

    surface.fill((180, 180, 180))  # grid-line colour

    for gy in range(size_y):
        for gx in range(size_x):
            px = gx * cell_w
            py = (size_y - 1 - gy) * cell_h   # y=0 at bottom

            rect = pygame.Rect(px + 1, py + 1, cell_w - 2, cell_h - 2)

            if (gx, gy) in env['obstacles']:
                colour = (0, 0, 0)
            elif gx == robot['x'] and gy == robot['y']:
                colour = (220, 0, 0)
            else:
                colour = (255, 255, 255)

            pygame.draw.rect(surface, colour, rect)

            if gx == robot['x'] and gy == robot['y']:
                _draw_arrow(
                    surface,
                    px + cell_w // 2,
                    py + cell_h // 2,
                    robot['theta'],
                    min(cell_w, cell_h) * 0.35,
                )


# ---------------------------------------------
#  Simulator ROS2 node
# ---------------------------------------------
class SimulatorNode(Node):
    def __init__(self, env):
        super().__init__('simulator')
        self._env = env
        self._lock = threading.Lock()
        self._robot = {'x': 0, 'y': 0, 'theta': 0}

        self._action_server = ActionServer(
            self, Move, 'move', self._execute_cb,
        )
        self.get_logger().info(
            f"Ready. Environment {env['size_x']}x{env['size_y']}, "
            f"{len(env['obstacles'])} obstacle(s)."
        )

    @property
    def robot_snapshot(self):
        with self._lock:
            return dict(self._robot)

    # -- movement helpers --------------------------------------------------
    @staticmethod
    def _fwd(theta):
        return {0: (0, 1), 90: (1, 0), 180: (0, -1), 270: (-1, 0)}[theta]

    def _execute_cb(self, goal_handle):
        cmd = goal_handle.request.command
        self.get_logger().info(f'[execute] {cmd!r}')

        time.sleep(1.0)   # simulate action duration

        success = True
        with self._lock:
            r = self._robot
            if cmd == 'turn_left':
                r['theta'] = (r['theta'] - 90) % 360
            elif cmd == 'turn_right':
                r['theta'] = (r['theta'] + 90) % 360
            else:
                dx, dy = self._fwd(r['theta'])
                if cmd == 'back':
                    dx, dy = -dx, -dy
                elif cmd == 'left':       # strafe left  = rotate dir 90° CCW
                    dx, dy = -dy, dx
                elif cmd == 'right':      # strafe right = rotate dir 90° CW
                    dx, dy = dy, -dx
                # 'straight': unchanged

                nx, ny = r['x'] + dx, r['y'] + dy
                in_bounds = 0 <= nx < self._env['size_x'] and \
                            0 <= ny < self._env['size_y']
                free = (nx, ny) not in self._env['obstacles']

                if in_bounds and free:
                    r['x'], r['y'] = nx, ny
                else:
                    self.get_logger().warn(f'Move blocked at ({nx},{ny})')
                    success = False

            snap = dict(r)

        self.get_logger().info(
            f'[execute] {cmd!r} -> x:{snap["x"]} y:{snap["y"]} '
            f'theta:{snap["theta"]} success:{success}'
        )

        result = Move.Result()
        result.x, result.y, result.theta, result.success = (
            snap['x'], snap['y'], snap['theta'], success
        )
        goal_handle.succeed()
        return result


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
    env = parse_environment(env_path)
    node = SimulatorNode(env)

    # Spin in a background thread (MultiThreadedExecutor so time.sleep
    # inside the action callback doesn't block other callbacks)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Pygame render loop on the main thread
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption('Robot Environment')
    clock = pygame.time.Clock()

    running = True
    while running and rclpy.ok():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        draw_environment(screen, env, node.robot_snapshot)
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
