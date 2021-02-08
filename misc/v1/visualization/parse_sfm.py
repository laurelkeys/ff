import os
import argparse
import numpy as np

import json
from collections import namedtuple

#########################################

def json2obj(json_str):
    ''' Parse string representation of JSON file (`json_str`) 
        into a `namedtuple` with attributes corresponding to dict keys '''
    def _json_object_hook(_dict):
        return namedtuple('X', _dict.keys())(*_dict.values())
    try:
        return json.loads(json_str, object_hook=_json_object_hook)
    except json.decoder.JSONDecodeError:
        return json.loads(json_str.replace("'", '"'), object_hook=_json_object_hook)

def dict2obj(d):
    return namedtuple('Y', d)(**d)

#########################################

def del_key_inplace(obj, bad_key):
    ''' Remove entries of `bad_key` from `obj` (inplace)\n
        ref.: https://stackoverflow.com/a/20692955 '''
    if isinstance(obj, dict):
        for key in list(obj.keys()):
            if key == bad_key:
                del obj[key]
            else:
                del_key_inplace(obj[key], bad_key)
    elif isinstance(obj, list):
        for i in reversed(range(len(obj))):
            if obj[i] == bad_key:
                del obj[i]
            else:
                del_key_inplace(obj[i], bad_key)
    else:
        # neither a dict nor a list, do nothing
        pass
    return obj

def del_keys_inplace(obj, bad_keys):
    for bad_key in bad_keys:
        del_key_inplace(obj, bad_key)
    return obj

#########################################

def recursive_keys(obj):
    if not isinstance(obj, dict):
        return
    for key, value in obj.items():
        yield key
        if isinstance(value, dict):
            for key in recursive_keys(value):
                yield key
        elif isinstance(value, list):
            for elem in value:
                for key in recursive_keys(elem):
                    yield key

def recursive_unique_keys(obj):
    for key in set(recursive_keys(obj)):
        yield key

#########################################

# .sfm is actually a JSON
with open("data/cameras.sfm", 'r') as sfm_file:
    cameras = json.loads(sfm_file.read())
with open("data/sfm.sfm", 'r') as sfm_file:
    sfm = json.loads(sfm_file.read())

# TODO find out what's 'resectionId'
unused_keys = ['version', 'featuresFolders', 'matchesFolders', 'intrinsics', 'metadata', 'resectionId']
del_keys_inplace(cameras, unused_keys)
del_keys_inplace(sfm, unused_keys)

# print([k for k in cameras.keys()], list(recursive_unique_keys(cameras)))
# print([k for k in sfm.keys()], list(recursive_unique_keys(sfm)))

#  ________________________________sfm.sfm_________________________________
# /________________cameras.sfm_________________                            \
# /                                            \
# -> views              -> poses                   -> structure
#    |-> viewId            |-> poseId                 |-> landmarkId
#    |-> poseId            |-> pose                   |-> descType
#    |-> intrinsicId           |-> transform          |-> color
#    |-> path                      |-> rotation       |-> X
#    |-> width                     |-> center         |-> observations
#    |-> height                |-> locked                 |-> observationId

def camera2ply(obj, camera_color=(0, 255, 0)):
    header = (
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
    )

    obj_paths = [_views['path'] for _views in obj.views]
    for path in obj_paths:
        header += f"comment {path}\n"

    ply = ""
    vertex_count = 0
    obj_centers = [_poses['pose']['transform']['center'] for _poses in obj.poses]
    for center in obj_centers:
        ply += f"{' '.join(map('{:.8f}'.format, map(float, center)))} {' '.join(map(str, camera_color))}\n"
        vertex_count += 1

    header = f"ply\nformat ascii 1.0\nelement vertex {vertex_count}\n" + header + "end_header\n"
    return header + ply

def sfm2ply(obj, camera_color=(0, 255, 0)):
    header = (
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
    )

    obj_paths = [_views['path'] for _views in obj.views]
    for path in obj_paths:
        header += f"comment {path}\n"

    ply = ""
    vertex_count = 0
    obj_centers = [_poses['pose']['transform']['center'] for _poses in obj.poses]
    for center in obj_centers:
        ply += f"{' '.join(map('{:.8f}'.format, map(float, center)))} {' '.join(map(str, camera_color))}\n"
        vertex_count += 1
    
    obj_points = [(_structure['X'], _structure['color']) for _structure in obj.structure]
    for point, color in obj_points:
        ply += f"{' '.join(map('{:.8f}'.format, map(float, point)))} {' '.join(color)}\n"
        vertex_count += 1

    header = f"ply\nformat ascii 1.0\nelement vertex {vertex_count}\n" + header + "end_header\n"
    return header + ply

_cameras = dict2obj(cameras)
# print(camera2ply(_cameras))

_sfm = dict2obj(sfm)
print(sfm2ply(_sfm))

# -> views: List[dict]
#    |-> viewId: str(int)
#    |-> poseId: str(int)
#    |-> intrinsicId: str(int)
#    |-> path: str
#    |-> width: str(int)
#    |-> height: str(int)
# -> poses: List[dict]
#    |-> poseId: str(int)
#    |-> pose: dict
#        |-> transform: dict
#            |-> rotation: List[str(int)]
#            |-> center: List[str(int)]
#        |-> locked: str(int)
# -> structure: List[dict]
#    |-> landmarkId: str(int)
#    |-> descType: str
#    |-> color: List[str(int)]
#    |-> X: List[str(float)]
#    |-> observations *
#        |-> observationId **
#
# *, ** e.g.: "observations": [{ "observationId": "536766085" }, { "observationId": "1571755330" }]