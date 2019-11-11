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

def obj2ply(obj):
    ply = (
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    vertex_count = 0
    ply = f"ply\nformat ascii 1.0\nelement vertex {vertex_count}\n" + ply
    return ply

# print(obj2ply(obj))

