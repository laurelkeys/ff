import os
import argparse
from subprocess import run

def get_parser():
    parser = argparse.ArgumentParser(description="Setup a venv for running AirSim (on Windows).")
    parser.add_argument("env_dir", type=str, help="Path to create the venv in (with its name as the last folder)")
    return parser

if __name__ == "__main__":
    args = get_parser().parse_args()

    venv_name = os.path.basename(os.path.normpath(args.env_dir))

    # create venv
    run(["python", "-m", "venv", args.env_dir])

    venv_path = args.env_dir if os.path.isabs(args.env_dir) else os.path.join(os.getcwd(), args.env_dir)
    if not os.path.isdir(venv_path):
        exit(f"Error: '{venv_path}' is not a valid directory")

    # activate venv
    run([f"Scripts\\activate.bat"], cwd=venv_path)

    # upgrade pip
    run(["python", "-m", "pip", "install", "--upgrade", "pip"])

    # install requirements
    run(["pip", "install", "numpy", "matplotlib", "opencv-python", "msgpack-rpc-python"])



