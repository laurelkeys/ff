import os
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

data = "{'name': 'John Smith', 'hometown': {'name': 'New York', 'id': 123}}"
x = json2obj(data)
print("x:", x)