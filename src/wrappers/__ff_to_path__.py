import os
import sys

__abs_dir_path = os.path.abspath(os.path.dirname(__file__))
__ff_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "ff")
sys.path.insert(0, __ff_path)

if __name__ == "__main__":
    print(f"abs_dir_path = '{__abs_dir_path}'")
    print(f"ff_path = '{__ff_path}'")
