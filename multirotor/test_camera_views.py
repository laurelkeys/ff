from sys import path
path.append("..")

import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

from sys import argv

###############################################################################

# FIXME TODO use argparse
request_image_type = {
    "Scene":               True,
    "DepthPlanner":        False,
    "DepthPerspective":    False,
    "DepthVis":            False,
    "DisparityNormalized": False,
    "Segmentation":        False,
    "SurfaceNormals":      True,
    "Infrared":            False
}

def generateImageRequestList(request_image_type_dict=request_image_type, **kwargs):
    # ImageType(camera_name, image_type, pixels_as_float=False, compress=True)
    str2ImageType = {
        "Scene":               airsim.ImageType.Scene,
        "DepthPlanner":        airsim.ImageType.DepthPlanner,
        "DepthPerspective":    airsim.ImageType.DepthPerspective,
        "DepthVis":            airsim.ImageType.DepthVis,
        "DisparityNormalized": airsim.ImageType.DisparityNormalized,
        "Segmentation":        airsim.ImageType.Segmentation,
        "SurfaceNormals":      airsim.ImageType.SurfaceNormals,
        "Infrared":            airsim.ImageType.Infrared
    }
    image_type_list = [str2ImageType[image_type_str] 
                       for image_type_str, request in request_image_type_dict.items() 
                       if request == True]
    return [airsim.ImageRequest(image_type=image_type, **kwargs) for image_type in image_type_list]

###############################################################################

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# state = client.getMultirotorState()
# s = pprint.pformat(state)
# print("state: %s" % s)
airsim.wait_key('Press any key to takeoff')
client.takeoffAsync(timeout_sec=10).join()
# airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
# client.moveToPositionAsync(-10, 10, -10, 5).join()
client.hoverAsync().join()

airsim.wait_key('Press any key to take images')
# get camera images
responses = client.simGetImages(generateImageRequestList(camera_name='0'))
print('Retrieved images: %d' % len(responses))

tmp_dir = os.path.join('.', "airsim_drone")
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses):
    filename = os.path.join(tmp_dir, str(idx))
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

airsim.wait_key('Press any key to reset to original state')
client.armDisarm(False)
client.reset()
client.enableApiControl(False)