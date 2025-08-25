from math import pi

from common.consts import (
    OBSTACLE_WIDTH,
    ROBOT_MIN_CAMERA_DIST,
    ROBOT_HEIGHT,
    ROBOT_WIDTH,
)
from common.enums import Direction
from common.types import Position


class Obstacle:
    def __init__(self, x: float, y: float, facing: Direction):
        self.x = x
        self.y = y
        self.facing = facing
        self.middle = (x + OBSTACLE_WIDTH / 2, y + OBSTACLE_WIDTH / 2)

    def to_pos(self) -> Position:
        x = self.x
        y = self.y
        theta = None

        pad = (ROBOT_WIDTH - OBSTACLE_WIDTH) / 2

        if self.facing == Direction.NORTH:
            if x == 190:
                x += pad
            elif x == 0:
                x += pad + (2*OBSTACLE_WIDTH)
            else:
                x += pad + OBSTACLE_WIDTH  # pad changed to 4
            y += ROBOT_MIN_CAMERA_DIST + OBSTACLE_WIDTH + ROBOT_HEIGHT
            theta = -pi / 2
        elif self.facing == Direction.SOUTH:
            if x == 190:
                x -= pad + OBSTACLE_WIDTH  # pad changed to 13
            elif x != 0:
                x -= pad
            y -= ROBOT_MIN_CAMERA_DIST + ROBOT_HEIGHT
            theta = pi / 2
        elif self.facing == Direction.EAST:
            if y == 190:
                y -= pad + OBSTACLE_WIDTH
            elif y != 0:
                y -= pad
            x += ROBOT_MIN_CAMERA_DIST + OBSTACLE_WIDTH + ROBOT_WIDTH
            theta = pi
        elif self.facing == Direction.WEST:
            if y == 0:
                y += pad + (2*OBSTACLE_WIDTH)
            elif y == 190:
                y += pad
            else:
                y += pad + OBSTACLE_WIDTH  # pad changed to 4
            x -= ROBOT_MIN_CAMERA_DIST + ROBOT_WIDTH
            theta = 0

        print("original", self.x, self.y)
        print("modified", x, y)
        return Position(x, y, theta)
