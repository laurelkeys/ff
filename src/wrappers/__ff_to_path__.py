import os
import sys

__ff_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "ff"
)

sys.path.insert(0,  os.path.dirname(__ff_path))

import ff

del sys.path[0], sys, os

if __name__ == "__main__":
    print(f"ff path = '{__ff_path}'")
