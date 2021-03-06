# HACK keep this synced with reconstruction/include_in_path.py

import sys
from os.path import join, abspath, dirname

FF_PROJECT_ROOT = abspath(join(abspath(__file__), "..", "..", ".."))


def include(*relative_path):
    file_dir_path = dirname(abspath(__file__))
    absolute_path = abspath(join(file_dir_path, *relative_path))
    sys.path.append(dirname(absolute_path))
