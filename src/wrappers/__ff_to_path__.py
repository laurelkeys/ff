import os
import sys

abs_dir_path = os.path.abspath(os.path.dirname(__file__))
ff_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "ff")
sys.path.insert(0, ff_path)

if __name__ == "__main__":
    print(f"abs_dir_path = '{abs_dir_path}'")
    print(f"ff_path = '{ff_path}'")
