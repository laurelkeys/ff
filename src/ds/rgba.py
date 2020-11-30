from __future__ import annotations


class Rgba(tuple):
    White = (1.0, 1.0, 1.0, 1.0)
    Black = (0.0, 0.0, 0.0, 1.0)

    Red   = (1.0, 0.0, 0.0, 1.0)
    Green = (0.0, 1.0, 0.0, 1.0)
    Blue  = (0.0, 0.0, 1.0, 1.0)

    Cyan    = (0.0, 1.0, 1.0, 1.0)
    Magenta = (1.0, 0.0, 1.0, 1.0)
    Yellow  = (1.0, 1.0, 0.0, 1.0)

    def __new__(cls, r: float, g: float, b: float, alpha: float = 1.0):
        rgba = (r, g, b, alpha)
        assert all(0.0 <= ch <= 1.0 for ch in rgba), rgba
        return super(Rgba, cls).__new__(cls, rgba)

    def __init__(self, r: float, g: float, b: float, alpha: float = 1.0):
        """ Construct a new color, with values RGBA in `[0.0, 1.0]`. """
        self.r = self[0]
        self.g = self[1]
        self.b = self[2]
        self.a = self[3]

    @staticmethod
    def from255(r: int, g: int, b: int, alpha: float = 1.0) -> Rgba:
        """ Construct a new color by converting RGB values from `[0, 255]` to `[0.0, 1.0]`. """
        return Rgba(r / 255, g / 255, b / 255, alpha)
