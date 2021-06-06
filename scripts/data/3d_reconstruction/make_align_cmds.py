import os

from os.path import join, abspath, dirname

# HACK manually edited values...
HERE = dirname(abspath(__file__))

ROOT_SCENE_NAME = "angel"
RECONSTRUCTIONS = ["v1_initial", "v1_shortened"]

if __name__ == "__main__":
    folder = join(HERE, ROOT_SCENE_NAME)
    for reconstruction in RECONSTRUCTIONS:
        subfolder = join(folder, reconstruction)
        for sim_mode in ["cv", "drone", "drone2"]:
            meshroom_sfm = os.path.abspath(join(subfolder, sim_mode, "meshroom", "cache", "StructureFromMotion", "cameras.sfm"))
            airsim_rec = os.path.abspath(join(subfolder, sim_mode, "rec.txt"))
            output = os.path.abspath(join(subfolder, sim_mode, "align.txt"))

            assert os.path.isfile(meshroom_sfm), meshroom_sfm  # source
            assert os.path.isfile(airsim_rec), airsim_rec  # target

            # $ python align_meshroom_to_airsim.py <meshroom_sfm> <airsim_rec>
            print(f"\npython align_meshroom_to_airsim.py {meshroom_sfm} {airsim_rec} --output {output}")
