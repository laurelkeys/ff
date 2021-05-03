from airsim import Vector3r


class Ned:
    """ Positions in NED coordinate system used in AirSim. """

    Cidadela_Statue = Vector3r(-0.02, -5.485, -3.15)

    Urban_Building = Vector3r(-50.0, 10.0, -10.0)


class Uavmvs:
    """ Translation offset and scaling factor used for uavmvs. """

    Cidadela_Statue_Offset = Vector3r(1.2375, -6.15, 7.75)
    Cidadela_Statue_Scale = 0.168

    Urban_Building_Offset = Vector3r(-55, 11, 1)
    # Urban_Building_Scale = None
