import os
import json
import argparse

from collections import namedtuple

try:
    from include_in_path import include
    include("..", "wrappers")
except:
    pass
finally:
    from wrappers.airsimy import AirSimRecord
    from wrappers.meshroomy import MeshroomParser, MeshroomTransform, MeshroomQuaternion

SHOW_CORRESPONDENCES = True  # FIXME


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.sfm), f"Invalid file path: '{args.sfm}'"
    assert os.path.isfile(args.rec), f"Invalid file path: '{args.rec}'"
    if not args.sfm.endswith(".sfm") and args.rec.endswith(".sfm"):
        print("Swapping argument order based on file extensions\n")
        args.sfm, args.rec = args.rec, args.sfm
    else:
        assert False, f"Expected sfm file as a .sfm: '{args.sfm}'"
    assert args.rec.endswith(".txt"), f"Expected rec file as a .txt: '{args.rec}'"

    if args.verbose:
        print(f"cameras.sfm path: '{args.sfm}'")
        print(f"airsim_rec.txt path: '{args.rec}'")
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

    print(f"cameras.sfm: {match_count} out of {len(meshroom_images)} images matched")
    print(f"airsim_rec.txt: {match_count} out of {len(airsim_images)} images matched")
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

        new_center = [record.position.x_val, record.position.y_val, record.position.z_val]
        new_rotation = MeshroomTransform.unparse_rotation(
            MeshroomQuaternion.XYZW.to_rotation_matrix(
                record.orientation.to_numpy_array()  # returned in XYZW order
            )
        )

        # Update 'transform' values and set 'locked' to 1
        pose["pose"]["transform"]["center"] = list(map(str, new_center))
        pose["pose"]["transform"]["rotation"] = list(map(str, new_rotation))
        pose["pose"]["locked"] = "1"

    if SHOW_CORRESPONDENCES:
        # NOTE this is used by align_with_icp.py
        Correspondence = namedtuple("Correspondence", ["source_idx", "target_idx"])
        correspondences = []

        # FIXME this assumes `record_dict.keys()` have the same order as in airsim_rec.txt
        indexed_airsim_paths = list(enumerate(record_dict.keys()))
        for meshroom_idx, pose in enumerate(cameras_sfm["poses"]):
            meshroom_path = view_dict[pose["poseId"]].path
            [(i, airsim_idx)] = [
                (i, airsim_idx)
                for i, (airsim_idx, airsim_path) in enumerate(indexed_airsim_paths)
                if os.path.basename(airsim_path) == os.path.basename(meshroom_path)
            ]
            del indexed_airsim_paths[i]  # remove images that have already been matched
            correspondences.append(Correspondence(source_idx=meshroom_idx, target_idx=airsim_idx))

        print(f"{len(correspondences)} [meshroom (source), airsim (target)] correspondences:")
        print("[")
        for correspondence in correspondences:
            print(f"[{correspondence.source_idx}, {correspondence.target_idx}],")
        print("]")

    # Save the new .sfm file
    new_file_path = "new_cameras.sfm"
    if args.output is not None:
        new_file_path = (
            args.output if args.output.endswith(".sfm")    # file
            else os.path.join(args.output, new_file_path)  # dir
        )

    with open(new_file_path, "w") as f:
        print(json.dumps(cameras_sfm, indent=4), file=f)
    print(f"Saved output to '{new_file_path}'")

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
