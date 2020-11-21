# HACK clean this up later..

def include(*relative_path):
    import os, sys

    file_dir_path = os.path.dirname(os.path.abspath(__file__))
    absolute_path = os.path.abspath(os.path.join(file_dir_path, *relative_path))
    sys.path.append(os.path.dirname(absolute_path))

    del sys, os
