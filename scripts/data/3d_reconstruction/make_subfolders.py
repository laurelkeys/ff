import os

from os.path import join, abspath, dirname

# HACK manually edited values...
HERE = dirname(abspath(__file__))

ROOT_SCENE_NAME = "angel"
RECONSTRUCTIONS = ["v1_initial", "v1_shortened"]

if __name__ == "__main__":
    folder = join(HERE, ROOT_SCENE_NAME)
    os.makedirs(folder, exist_ok=True)
    for reconstruction in RECONSTRUCTIONS:
        subfolder = join(folder, reconstruction)
        os.makedirs(subfolder, exist_ok=True)
        for sim_mode in ["cv", "drone" , "drone2", "cv0"]:
            os.makedirs(join(subfolder, sim_mode), exist_ok=True)
            os.makedirs(join(subfolder, sim_mode, "eval"), exist_ok=True)
            os.makedirs(join(subfolder, sim_mode, "images"), exist_ok=True)
            os.makedirs(join(subfolder, sim_mode, "meshroom"), exist_ok=True)
        # open(join(folder, f"run_{ROOT_SCENE_NAME}_{reconstruction}.json"), "w+")
