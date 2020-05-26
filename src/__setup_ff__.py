import os
import sys

abs_dir_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(abs_dir_path, "ff"))

if __name__ == "__main__":
    print(f"abs_dir_path = '{abs_dir_path}'")