import os
import sys
import json
import argparse

try:
    def include(*relative_path):
        file_dir_path = os.path.dirname(os.path.abspath(__file__))
        absolute_path = os.path.abspath(os.path.join(file_dir_path, *relative_path))
        sys.path.append(os.path.dirname(absolute_path))

    include("..", "ff")
    include("..", "wrappers")

except:
    pass

finally:
    import ff

    from wrappers.airsimy import AirSimRecord
    from wrappers.meshroomy import MeshroomParser, MeshroomTransform, MeshroomQuaternion


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.sfm), f"Invalid file path: '{args.sfm}'"
    assert os.path.isfile(args.rec), f"Invalid file path: '{args.rec}'"
    if not args.sfm.endswith(".sfm") and args.rec.endswith(".sfm"):
        ff.log_warning("Swapping argument order based on file extensions\n")
        args.sfm, args.rec = args.rec, args.sfm
    else:
        assert False, f"Expected sfm file as a .sfm: '{args.sfm}'"
    assert args.rec.endswith(".txt"), f"Expected rec file as a .txt: '{args.rec}'"

    if args.verbose:
        ff.log_info(f"cameras.sfm path: '{args.sfm}'")
        ff.log_info(f"airsim_rec.txt path: '{args.rec}'")
        print()

    record_dict = AirSimRecord.dict_from(args.rec)
    view_dict, _pose_dict = MeshroomParser.extract_views_and_poses(
        *MeshroomParser.parse_cameras(args.sfm)
    )

    # NOTE Meshroom uses absolute paths, while AirSim uses relative
    airsim_images = [os.path.basename(_) for _ in record_dict.keys()]
    meshroom_images = [os.path.basename(_.path) for _ in view_dict.values()]

    matching_images = set(airsim_images).intersection(set(meshroom_images))
    match_count = len(matching_images)

    ff.log(f"cameras.sfm: {match_count} out of {len(meshroom_images)} images matched")
    ff.log(f"airsim_rec.txt: {match_count} out of {len(airsim_images)} images matched")
    print()

    # Generate a new cameras.sfm file matching poses from airsim.rec, to be
    # used with the `FeatureMatching > matchFromKnownCameraPoses` node option
    # https://github.com/alicevision/meshroom/wiki/Using-known-camera-positions
    cameras_sfm = json.loads(open(args.sfm, "r").read())
    assert list(cameras_sfm.keys()) == [
        "version",          # keep
        "featuresFolders",  # remove
        "matchesFolders",   # remove
        "views",            # keep
        "intrinsics",       # keep (TODO find AirSim's camera intrinsics)
        "poses",            # update 'transform' and set 'locked' from 0 to 1
    ]

    del cameras_sfm["featuresFolders"]
    del cameras_sfm["matchesFolders"]

    for pose in cameras_sfm["poses"]:
        # Replace the 'transform's 'center' with `position`, and 'rotation' with
        # `orientation` (converted from WXYZ quaternion to a 3x3 rotation matrix)
        pose_id, _rotation, _center = MeshroomParser.Pose.extract_from(pose)

        # NOTE 'poseId' is equal to 'viewId', so we can use it to index into the view dict
        image_file = os.path.basename(view_dict[pose_id].path)  # FIXME this isn't a "general" solution
        record = record_dict[image_file]

        new_center = ff.to_xyz_tuple(record.position)
        new_rotation = MeshroomTransform.unparse_rotation(
            MeshroomQuaternion.XYZW.to_rotation_matrix(
                record.orientation.to_numpy_array()  # returned in XYZW order
            )
        )

        # Update 'transform' values and set 'locked' to 1
        pose["pose"]["transform"]["center"] = list(map(str, new_center))
        pose["pose"]["transform"]["rotation"] = list(map(str, new_rotation))
        pose["pose"]["locked"] = "1"

    # Save the new .sfm file
    new_file_path = "new_cameras.sfm"
    if args.output is not None:
        new_file_path = (
            args.output if args.output.endswith(".sfm")    # file
            else os.path.join(args.output, new_file_path)  # dir
        )

    with open(new_file_path, "w") as f:
        print(json.dumps(cameras_sfm, indent=4), file=f)
    ff.log(f"Saved output to '{new_file_path}'")

    # TODO see https://github.com/alicevision/meshroom/issues/655
    # TODO see https://github.com/alicevision/meshroom/issues/453


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Matches Meshroom's reconstructed camera poses (stored in cameras.sfm)"
        " with the AirSim recording log from which the images were taken (airsim_rec.txt),"
        " by comparing file names, then generates a new_cameras.sfm file."
    )

    parser.add_argument("sfm", type=str, help="Path to cameras.sfm")
    parser.add_argument("rec", type=str, help="Path to airsim_rec.txt")
    parser.add_argument("--output", type=str, help="Path to the generated .sfm file")
    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")

    # ff.add_arguments_to(parser)  # unnecessary (unless we're going to launch AirSim)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)
