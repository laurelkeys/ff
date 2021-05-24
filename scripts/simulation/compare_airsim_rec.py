import os
import argparse

import numpy as np

from ie.airsimy import AirSimRecord

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compute statistic metrics between two camera trajectories"
    )
    # NOTE this is AirSim's `Recording` output
    parser.add_argument("rec1", type=str, help="Path to the 1st airsim_rec.txt")
    parser.add_argument("rec2", type=str, help="Path to the 2nd airsim_rec.txt")
    args = parser.parse_args()

    assert os.path.isfile(args.rec1), f"Invalid file path: '{args.rec1}'"
    assert os.path.isfile(args.rec2), f"Invalid file path: '{args.rec2}'"

    args.recording1 = AirSimRecord.dict_from(rec_file=args.rec1)
    args.recording2 = AirSimRecord.dict_from(rec_file=args.rec2)

    distance_error = np.asarray(
        [
            r1.position.distance_to(r2.position)
            for r1, r2 in zip(args.recording1.values(), args.recording2.values())
        ]
    )

    print(f"     N: {len(distance_error)}")
    print(f"   min: {np.amin(distance_error):.4f}")
    print(f"   max: {np.amax(distance_error):.4f}")
    print(f"   std: {np.std(distance_error):.4f}")
    print(f"  mean: {np.mean(distance_error):.4f}")
    print(f"median: {np.median(distance_error):.4f}")
