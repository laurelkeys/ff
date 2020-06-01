import numpy as np

import ff
import airsim

###############################################################################
###############################################################################


def array_from_uncompressed(image, height, width):
    return np.flipud(np.fromstring(image, dtype=np.uint8).reshape(height, width, -1))


def get_image_mono(client, camera_name=ff.CameraName.front_center, vehicle_name=None):
    request = {
        "requests": [
            airsim.ImageRequest(
                camera_name, airsim.ImageType.Scene, pixels_as_float=False, compress=False,
            )
        ]
    }

    if vehicle_name is not None:
        request["vehicle_name"] = vehicle_name

    response, *_ = client.simGetImages(**request)

    return array_from_uncompressed(response.image_data_uint8, response.height, response.width)


def get_image_stereo(client, vehicle_name=None):
    request = {
        "requests": [
            airsim.ImageRequest(
                ff.CameraName.front_left,
                airsim.ImageType.Scene,
                pixels_as_float=False,
                compress=False,
            ),
            airsim.ImageRequest(
                ff.CameraName.front_right,
                airsim.ImageType.Scene,
                pixels_as_float=False,
                compress=False,
            ),
        ]
    }

    if vehicle_name is not None:
        request["vehicle_name"] = vehicle_name

    response_left, response_right, *_ = client.simGetImages(**request)

    return (
        array_from_uncompressed(
            response_left.image_data_uint8, response_left.height, response_left.width
        ),
        array_from_uncompressed(
            response_right.image_data_uint8, response_right.height, response_right.width
        ),
    )


###############################################################################
###############################################################################
