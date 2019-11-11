import os
import argparse
import numpy as np

import json
from collections import namedtuple

def json2obj(data):
    ''' Parse JSON into an object with attributes corresponding to dict keys '''
    def _json_object_hook(_dict):
        return namedtuple('X', _dict.keys())(*_dict.values())
    try:
        return json.loads(data, object_hook=_json_object_hook)
    except json.decoder.JSONDecodeError:
        return json.loads(data.replace("'", '"'), object_hook=_json_object_hook)

def scrub(obj, bad_key):
    ''' Remove entries of `bad_key` from `obj` (inplace)\n
        ref.: https://stackoverflow.com/a/20692955 '''
    if isinstance(obj, dict):
        for key in list(obj.keys()):
            if key == bad_key:
                del obj[key]
            else:
                scrub(obj[key], bad_key)
    elif isinstance(obj, list):
        for i in reversed(range(len(obj))):
            if obj[i] == bad_key:
                del obj[i]
            else:
                scrub(obj[i], bad_key)
    else:
        # neither a dict nor a list, do nothing
        pass
    return obj

# data = "{'name': 'John Smith', 'hometown': {'name': 'New York', 'id': 123}}"
# x = json2obj(data)
# print("x:", x)

with open("data/cameras.sfm", 'r') as cameras_sfm:
    cameras = cameras_sfm.read()

data = json.loads(cameras)
scrub(data, "metadata") # NOTE this changes the value of data
print(namedtuple('Y', data)(**data))