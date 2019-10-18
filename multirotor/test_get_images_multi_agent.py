from sys import path
path.append("..")
from multirotor import setup_path
import airsim
print("setup_path:", setup_path.__file__, "\nairsim:", airsim.__file__)

import numpy as np
import os
import tempfile
import pprint
import cv2

from sys import argv

###############################################################################

# FIXME TODO use argparse
drones = ["Drone1", "Drone2"]

request_image_type = {
    "Scene":               True,
    "DepthPlanner":        False,
    "DepthPerspective":    False,
    "DepthVis":            False,
    "DisparityNormalized": False,
    "Segmentation":        False,
    "SurfaceNormals":      False,
    "Infrared":            False
}

cameras = ["front_center", "front_right", "front_left", "bottom_center", "back_center"]

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

def wait_for_tasks(async_task_list):
    for task in async_task_list:
        task.join()

###############################################################################

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
for drone in drones:
    client.enableApiControl(True, drone)
    client.armDisarm(True, drone)

airsim.wait_key('Press any key to takeoff')
async_tasks = []
for drone in drones:
    async_tasks.append(client.takeoffAsync(timeout_sec=5, vehicle_name=drone))
wait_for_tasks(async_tasks)

# airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
# client.moveToPositionAsync(-10, 10, -10, 5).join()
async_tasks = []
for drone in drones:
    async_tasks.append(client.hoverAsync(drone))
wait_for_tasks(async_tasks)

airsim.wait_key('Press any key to take images')
# get camera images
for drone in drones:
    requests = []
    for camera in cameras:
        requests.extend(generateImageRequestList(camera_name=camera))
    responses = client.simGetImages(requests, vehicle_name=drone)
    print(drone + ' retrieved images: %d' % len(responses))
    tmp_dir = os.path.join('.', "airsim_drone")
    try:
        os.makedirs(tmp_dir)
    except OSError:
        if not os.path.isdir(tmp_dir):
            raise
    for idx, response in enumerate(responses):
        filename = os.path.join(tmp_dir, drone + '_' + str(idx))
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