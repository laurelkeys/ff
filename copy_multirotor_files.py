import os
from shutil import copy
from sys import argv

# TODO copy more than just scripts (e.g. folders with pictures)
def copy_from(src_path, dst_path):
    for file in os.listdir(src_path):
        if file.endswith(".py"):
            try:
                copy(os.path.join(src_path, file), os.path.join(dst_path, file))
            except IOError as io_error:
                print("Error:", io_error)

# NOTE default values for my machine
src = os.path.join(os.getcwd(), "..", "..", "..", "Program Files", "AirSim", "PythonClient", "multirotor", "my")
dst = os.path.join(os.getcwd(), "multirotor")

if len(argv) > 1:
    dst = argv[1]
    if len(argv) > 2:
        src = argv[2]

print("src: " + src + "\ndst: " + dst + "\n")

copy_from(src, dst)
