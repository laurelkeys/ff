# HACK clean this up later..

import sys
from os.path import join, abspath, dirname


FF_PROJECT_ROOT = abspath(join(abspath(__file__), "..", "..", ".."))


def include(*relative_path):
    file_dir_path = dirname(abspath(__file__))
    absolute_path = abspath(join(file_dir_path, *relative_path))
    sys.path.append(dirname(absolute_path))


if __name__ == "__main__":
    relative_path = join(*sys.argv[1:])
    file_dir_path = dirname(abspath(__file__))
    absolute_path = abspath(join(file_dir_path, relative_path))
    print(f"relative: '{relative_path}'")
    print(f"file dir: '{file_dir_path}'")
    print(f"absolute: '{absolute_path}'")
    print(f"sys.path: '{dirname(absolute_path)}'")
    print(f"ff/ path: '{FF_PROJECT_ROOT}'")
