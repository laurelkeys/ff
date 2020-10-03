from __future__ import annotations
from fly_zone import zigzag_path

import json

from typing import List, Union

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Vector3r


class Rect:
    def __init__(self, center: Vector3r, width: Vector3r, height: Vector3r):
        self.center = center
        self.half_width = width / 2.0
        self.half_height = height / 2.0

    def corners(self, repeat_first: bool = False) -> List[Vector3r]:
        corners = [
            self.center - self.half_width - self.half_height,
            self.center + self.half_width - self.half_height,
            self.center + self.half_width + self.half_height,
            self.center - self.half_width + self.half_height,
        ]
        return corners if not repeat_first else corners + [corners[0]]

    def closest_corner(self, point: Vector3r) -> Vector3r:
        closest_corner, _ = min(
            [(corner, corner.distance_to(point)) for corner in self.corners()],
            key=lambda corner_and_distance: corner_and_distance[1],
        )
        return closest_corner

    def zigzag(
        self, lanes: int, start_corner: Union[int, Vector3r] = 0, clock_wise: bool = False
    ) -> List[Vector3r]:
        corners = self.corners()

        if isinstance(start_corner, int):
            assert start_corner in range(4), f"invalid corner index ({start_corner}) for {self}"
            start_corner = corners[start_corner]
        else:
            assert start_corner in corners, f"invalid corner {start_corner} for {self}"

        # sort `corners` by their distance to `start_corner`, so that we can remove the
        # diagonally opposite corner to it (and also itself) from `corners`, and then use
        # the cross product to find out to each of the remaining 2 we should go first
        _, *corners, _ = sorted(corners, key=lambda corner: corner.distance_to(start_corner))

        #                      *-----------------* :ignored after
        #                     /                 /   sorting
        #                    /                 /
        #               ^   / e2              /
        #               |  /                 /   ( counter clock-wise:      )
        # |e1 x e2| > 0 | /                 /    ( start flying towards e1, )
        #               |/       e1        /     ( then zigzag along e2     )
        # start_corner: *-----------------*
        #               |
        # |e2 x e1| < 0 |
        #               v

        e1, e2 = [corner - start_corner for corner in corners]
        if e1.cross(e2).get_length() < 0:
            e1, e2 = e2, e1

        path, curr = [start_corner], start_corner
        dx, dy = (e2, (e1 / lanes)) if clock_wise else (e1, (e2 / lanes))
        for _ in range(lanes):
            curr += dx; path.append(curr)
            curr += dy; path.append(curr)
            dx *= -1  # zigzag

        return path

    def __str__(self) -> str:
        return f"Rect({', '.join([ff.to_xyz_str(_) for _ in (self.center, self.half_width, self.half_height)])})"

    @staticmethod
    def to_dump(dump_rect: Rect) -> str:
        return json.dumps(
            {
                "center": ff.to_xyz_tuple(dump_rect.center),
                "half_width": ff.to_xyz_tuple(dump_rect.half_width),
                "half_height": ff.to_xyz_tuple(dump_rect.half_height),
            }
        )

    @staticmethod
    def from_dump(dump_str: str) -> Rect:
        json_repr = json.loads(dump_str)
        return Rect(
            Vector3r(*json_repr["center"]),
            Vector3r(*[2 * _ for _ in json_repr["half_width"]]),
            Vector3r(*[2 * _ for _ in json_repr["half_height"]]),
        )
