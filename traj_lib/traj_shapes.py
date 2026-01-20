from abc import ABC, abstractmethod
from dataclasses import dataclass
import math


@dataclass
class Point:
    x: float
    y: float


class TrajShape(ABC):
    def __init__(self, start: Point, seconds: float) -> None:
        self.start = start
        self.seconds = seconds

    @abstractmethod
    def get_goal_pt(self, t: float) -> Point:
        pass


class LineTraj(TrajShape):
    def __init__(self, start: Point, end: Point, seconds: float):
        super().__init__(start, seconds)

        self.end = end

    def get_goal_pt(self, t: float) -> Point:
        x_goal = self.start.x + (self.end.x - self.start.x) * t / self.seconds
        y_goal = self.start.y + (self.end.y - self.start.y) * t / self.seconds
        goal_pt = Point(x_goal, y_goal)
        return goal_pt


class CircleTraj(TrajShape):
    def __init__(self, start: Point, radius: float, seconds: float):
        super().__init__(start, seconds)
        self.radius = radius
        self.circle_ctr = Point(start.x - self.radius, start.y)

    def get_goal_pt(self, t: float) -> Point:
        cur_angle_around_circle = 2 * math.pi * t / self.seconds
        x_goal = self.circle_ctr.x + self.radius * math.cos(cur_angle_around_circle)
        y_goal = self.circle_ctr.y + self.radius * math.sin(cur_angle_around_circle)
        goal_pt = Point(x_goal, y_goal)
        return goal_pt
