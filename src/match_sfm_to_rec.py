import os
import json
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
    view_dict, pose_dict = MeshroomParser.extract_views_and_poses(
        *MeshroomParser.parse_cameras(args.sfm)
    )

    # NOTE Meshroom uses absolute paths, while AirSim uses relative
    airsim_images = [os.path.basename(_) for _ in record_dict.keys()]
    meshroom_images = [os.path.basename(_.path) for _ in view_dict.values()]

    airsim_image_count = len(airsim_images)
    meshroom_image_count = len(meshroom_images)

    matching_images = set(airsim_images).intersection(set(meshroom_images))
    match_count = len(matching_images)

    ff.log(f"cameras.sfm: {match_count} out of {meshroom_image_count} images matched")
    ff.log(f"airsim_rec.txt: {match_count} out of {airsim_image_count} images matched")
    print()

    # Generate a new cameras.sfm file using the poses from airsim.rec to be
    # used with the `FeatureMatching > matchFromKnownCameraPoses` node option
    # https://github.com/alicevision/meshroom/wiki/Using-known-camera-positions
    new_cameras_sfm = json.loads(open(args.sfm, "r").read())

    assert list(new_cameras_sfm.keys()) == [
        "version",          # keep
        "featuresFolders",  # remove
        "matchesFolders",   # remove
        "views",            # keep
        "intrinsics",       # keep (TODO find AirSim's camera intrinsics)
        "poses",            # update 'transform' and set 'locked' from 0 to 1
    ]

    del new_cameras_sfm["featuresFolders"]
    del new_cameras_sfm["matchesFolders"]

    for pose in new_cameras_sfm["poses"]:
        # Replace the 'transform's 'center' with `position`, and 'rotation' with
        # `orientation` (converted from WXYZ quaternion to a 3x3 rotation matrix)
        pose_id, rotation, center = MeshroomParser.Pose.extract_from(pose)
        ff.log_debug(f"old: {center}")
        ff.log_debug(f"     {rotation}")

        view = view_dict[pose_id]  # NOTE 'poseId' is equal to the 'viewId'
        assert view.pose_id == pose_id  # so we can simply use it to index

        image_file = os.path.basename(view.path)  # FIXME
        record = record_dict[image_file]

        new_rotation, new_center = record.orientation, record.position
        # TODO http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q55
        new_center = list(ff.to_xyz_tuple(new_center))
        ff.log_debug(f"new: {new_center}")
        ff.log_debug(f"     {new_rotation}")

        break

    # TODO see https://github.com/alicevision/meshroom/issues/655
    # TODO see https://github.com/alicevision/meshroom/issues/453


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
