import os
import sys

from wrappers.meshroomy import MeshroomParser

if __name__ == "__main__":
    cameras_file_path = sys.argv[1]
    print(f"{cameras_file_path=}")
    assert os.path.isfile(cameras_file_path), "File not found"
    views, poses = MeshroomParser.parse_cameras(cameras_file_path)
    print(f"views: {views.keys()}")
    print(f"poses: {poses.keys()}")
