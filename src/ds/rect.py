from __future__ import annotations

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
        """ Create a new rectangle representation in 3D. """
        self.center = center
        self.half_width = width / 2.0
        self.half_height = height / 2.0

    def corners(self, repeat_first: bool = False) -> List[Vector3r]:
        """ Return a list of the rectangle's corner coordinates. """
        corners = [
            self.center - self.half_width - self.half_height,
            self.center + self.half_width - self.half_height,
            self.center + self.half_width + self.half_height,
            self.center - self.half_width + self.half_height,
        ]
        return corners if not repeat_first else corners + [corners[0]]

    def closest_corner(self, point: Vector3r) -> Vector3r:
        """ Return the rectangle's closest corner to `point`. """
        closest_corner, _ = min(
            [(corner, corner.distance_to(point)) for corner in self.corners()],
            key=lambda corner_and_distance: corner_and_distance[1],
        )
        return closest_corner

    def zigzag(
        self, lanes: int, start_corner: Union[int, Vector3r] = 0, clock_wise: bool = False
    ) -> List[Vector3r]:
        """ Create a path that zigzags through the rectangle, starting at `start_corner`,
            finishing at the opposite corner, and dividing the path in `lanes` sections.
            Returns the list of waypoints that define the zigzagging path.
        """
        corners = self.corners()

        if not isinstance(start_corner, int):
            assert start_corner in corners, f"invalid corner {start_corner} for {self}"
        else:
            assert start_corner in range(4), f"invalid corner index ({start_corner}) for {self}"
            start_corner = corners[start_corner]

        # sort `corners` by their distance to `start_corner`, so that we can remove the
        # diagonally opposite corner to it (and also itself) from `corners`, and then use
        # the cross product to find out to each of the remaining 2 we should go first
        _, *corners, end_corner = sorted(
            corners, key=lambda corner: corner.distance_to(start_corner)
        )

        #                      *-----------------* :end_corner
        #                     /                 /
        #                    /                 /
        #               ^   / e2              /
        #               |  /                 /
        # |e1 x e2| > 0 | /                 /
        #               |/       e1        /
        # start_corner: *-----------------*
        #               |
        # |e2 x e1| < 0 |
        #               v

        e1, e2 = [corner - start_corner for corner in corners]
        if e1.cross(e2).get_length() < 0:
            e1, e2 = e2, e1

        # "counter clock-wise": start flying towards e1, then zigzag along e2
        #
        #          ^  ^---------------->*
        #         /  /
        #        /  <-----------------^
        #   e2  /                    /
        #      /  ^----------------->   ^
        #     /  /                     / "dy" == e2 / lanes
        #    /  <-----------------^   +
        #   /                    /
        #  +   *---------------->
        #          e1 == "dx"

        curr_pos, path = start_corner, [start_corner]
        dx, dy = (e2, (e1 / lanes)) if clock_wise else (e1, (e2 / lanes))
        for _ in range(lanes):
            curr_pos += dx; path.append(curr_pos)
            curr_pos += dy; path.append(curr_pos)
            dx *= -1  # zig zag

        # NOTE if the `lanes` count is odd, the `end_corner` will already have
        #      been added to the `path`, otherwise, we append it to the end
        return path if lanes % 2 == 1 else path + [end_corner]

    def __str__(self) -> str:
        return f"Rect({', '.join([ff.to_xyz_str(_) for _ in (self.center, self.half_width, self.half_height)])})"

    @staticmethod
    def to_dump(dump_rect: Rect) -> str:
        """ Convert `dump_rect` to a JSON-like string representation. """
        import json

        return json.dumps(
            {
                "center": ff.to_xyz_tuple(dump_rect.center),
                "half_width": ff.to_xyz_tuple(dump_rect.half_width),
                "half_height": ff.to_xyz_tuple(dump_rect.half_height),
            }
        )

    @staticmethod
    def from_dump(dump_str: str) -> Rect:
        """ Convert `dump_str`, a string representation of a rectangle, back to a `Rect`.

            Note: `dump_str` is assumed to have been created by calling the `Rect.to_dump` method.
        """
        import json

        json_repr = json.loads(dump_str)
        return Rect(
            Vector3r(*json_repr["center"]),
            Vector3r(*[2 * _ for _ in json_repr["half_width"]]),
            Vector3r(*[2 * _ for _ in json_repr["half_height"]]),
        )
