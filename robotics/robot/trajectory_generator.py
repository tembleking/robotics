from robotics.geometry import Point


class TrajectoryGenerator:
    def __init__(self, points_to_visit):
        self.points_to_visit = points_to_visit

    def next_absolute_point_to_visit(self) -> Point:
        try:
            return self.points_to_visit[0]
        except IndexError:
            return None

    def mark_point_as_visited(self):
        self.points_to_visit = self.points_to_visit[1:]
