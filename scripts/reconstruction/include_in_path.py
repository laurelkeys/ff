# HACK clean this up later..


def include(*relative_path):
    import os, sys

    file_dir_path = os.path.dirname(os.path.abspath(__file__))
    absolute_path = os.path.abspath(os.path.join(file_dir_path, *relative_path))
    sys.path.append(os.path.dirname(absolute_path))

    del sys, os


if __name__ == "__main__":
    from os.path import join, abspath, dirname
    from sys import argv

    relative_path = join(*argv[1:])
    absolute_path = abspath(join(dirname(abspath(__file__)), relative_path))
    print(f"relative: '{relative_path}'")
    print(f"absolute: '{absolute_path}'")
    print(f"sys.path: '{dirname(absolute_path)}'")
