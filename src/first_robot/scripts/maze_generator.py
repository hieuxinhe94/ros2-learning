#!/usr/bin/env python3
import random

# Maze config
WIDTH = 15     # số ô ngang
HEIGHT = 15    # số ô dọc
CELL_SIZE = 1  # mỗi ô 1x1m
WALL_HEIGHT = 1.5
WALL_THICKNESS = 0.1

# Maze generation using DFS
def generate_maze(width, height):
    maze = [[0] * width for _ in range(height)]
    visited = [[False] * width for _ in range(height)]

    def dfs(x, y):
        visited[y][x] = True
        dirs = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        random.shuffle(dirs)
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and not visited[ny][nx]:
                maze[y][x] |= direction_to_bit(dx, dy)
                maze[ny][nx] |= direction_to_bit(-dx, -dy)
                dfs(nx, ny)

    def direction_to_bit(dx, dy):
        if dx == 1: return 1  # East
        if dx == -1: return 2  # West
        if dy == 1: return 4  # South
        if dy == -1: return 8  # North
        return 0

    dfs(0, 0)
    return maze

def wall_block(name, x, y, yaw):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {WALL_HEIGHT/2} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{CELL_SIZE} {WALL_THICKNESS} {WALL_HEIGHT}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{CELL_SIZE} {WALL_THICKNESS} {WALL_HEIGHT}</size></box></geometry>
          <material><diffuse>0.5 0.2 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
    """

def generate_world():
    maze = generate_maze(WIDTH, HEIGHT)
    world = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="maze_world">

    <!-- Light -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Large Ground -->
    <model name="ground">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="ground_link">
        <collision name="collision">
          <geometry><box><size>{WIDTH * CELL_SIZE} {HEIGHT * CELL_SIZE} 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{WIDTH * CELL_SIZE} {HEIGHT * CELL_SIZE} 0.1</size></box></geometry>
          <material><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

    count = 0
    for y in range(HEIGHT):
        for x in range(WIDTH):
            cell = maze[y][x]
            cx = x * CELL_SIZE - WIDTH / 2
            cy = y * CELL_SIZE - HEIGHT / 2

            # North wall
            if not (cell & 8):
                count += 1
                world += wall_block(f"wall_n_{count}", cx, cy - CELL_SIZE/2, 0)
            # West wall
            if not (cell & 2):
                count += 1
                world += wall_block(f"wall_w_{count}", cx - CELL_SIZE/2, cy, 1.5708)

    # Outer borders
    for i in range(WIDTH):
        world += wall_block(f"wall_top_{i}", i * CELL_SIZE - WIDTH/2, HEIGHT/2 * CELL_SIZE, 0)
        world += wall_block(f"wall_bottom_{i}", i * CELL_SIZE - WIDTH/2, -HEIGHT/2 * CELL_SIZE, 0)
    for j in range(HEIGHT):
        world += wall_block(f"wall_left_{j}", -WIDTH/2 * CELL_SIZE, j * CELL_SIZE - HEIGHT/2, 1.5708)
        world += wall_block(f"wall_right_{j}", WIDTH/2 * CELL_SIZE, j * CELL_SIZE - HEIGHT/2, 1.5708)

    world += "\n  </world>\n</sdf>\n"
    return world


with open("maze_world.world", "w") as f:
    f.write(generate_world())

print("✅ Maze world generated: maze_world.world")
