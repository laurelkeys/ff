import os
import argparse

import ff
from wrappers.airsimy import AirSimRecord
from wrappers.meshroomy import MeshroomParser, MeshroomTransform

###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.sfm), f"Invalid file path: '{args.sfm}'"
    assert os.path.isfile(args.rec), f"Invalid file path: '{args.rec}'"

    ff.log(f"cameras.sfm path: '{args.sfm}'")
    ff.log(f"airsim_rec.txt path: '{args.rec}'")
    print()

    record_dict = AirSimRecord.dict_from(args.rec)
    view_dict, _pose_dict = MeshroomParser.extract_views_and_poses(
        *MeshroomParser.parse_cameras(args.sfm)
    )

    airsim_image_fnames = [os.path.basename(_) for _ in record_dict.keys()]
    meshroom_image_fnames = [os.path.basename(_.path) for _ in view_dict.values()]

    airsim_image_count = len(airsim_image_fnames)
    meshroom_image_count = len(meshroom_image_fnames)

    # ff.log(*airsim_image_fnames)
    # print()
    # ff.log(*meshroom_image_fnames)

    matching_image_fnames = set(airsim_image_fnames) & set(meshroom_image_fnames)
    match_count = len(matching_image_fnames)

    ff.log(f"cameras.sfm: {match_count} out of {meshroom_image_count} images matched")
    ff.log(f"airsim_rec.txt: {match_count} out of {airsim_image_count} images matched")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Matches Meshroom's reconstructed camera poses (stored in cameras.sfm)"
        " with the AirSim recording log from which the images were taken (airsim_rec.txt),"
        " by comparing file names."
    )

    parser.add_argument("sfm", type=str, help="Path to cameras.sfm")
    parser.add_argument("rec", type=str, help="Path to airsim_rec.txt")

    # ff.add_arguments_to(parser)  # unnecessary (unless we're going to launch AirSim)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)
