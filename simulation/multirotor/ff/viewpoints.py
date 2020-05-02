from typing import List, Tuple


class Viewpoints:

    def positions(self) -> List[Tuple[float]]:
        raise NotImplementedError

    def orientations(self) -> List[Tuple[float]]:
        raise NotImplementedError


class Cidadela(Viewpoints):
    def __init__(self):
        self._positions = [
            (-26.4475, -33.3632, -6.5308),
            (-27.9139, -33.5906, -0.3988),
            (-28.0695, -33.6147,  3.8688),
            (-28.7371, -29.3113,  3.8688),
            (-28.9572, -22.3789,  3.8688),
            (-28.5201, -17.5598,  3.8688),
            (-27.2726, -17.6730,  0.2850),
            (-26.7237, -17.7617, -1.5968),
            (-26.3550, -17.8213, -4.6385),
            (-27.1018, -22.4406, -4.6385),
            (-28.9038, -25.6483, -5.2894)
        ]

        self._orientations = [
            ( 0.012022, -0.155972,  0.075903, 0.984767),
            ( 0.008701, -0.112869,  0.076367, 0.990633),
            ( 0.004693, -0.060868,  0.076729, 0.995181),
            ( 0.000969, -0.061041,  0.015837, 0.998009),
            (-0.002760, -0.060987, -0.045119, 0.997114),
            (-0.002760, -0.060987, -0.045119, 0.997114),
            (-0.004886, -0.060853, -0.079891, 0.994932),
            (-0.004887, -0.060853, -0.079897, 0.994932),
            (-0.008368, -0.104194, -0.079614, 0.991330),
            (-0.004727, -0.104422, -0.044977, 0.993504),
            (-0.000237, -0.147810, -0.001589, 0.989015)
        ]

        assert len(self._positions) == len(self._orientations)

    def positions(self) -> List[Tuple[float]]:
        return self._positions

    def orientations(self) -> List[Tuple[float]]:
        return self._orientations


DEFAULT_ENV_VIEWPOINTS = {
    "Cidadela": Cidadela
}
