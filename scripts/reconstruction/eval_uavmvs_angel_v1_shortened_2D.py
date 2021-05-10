import os
import argparse

try:
    from include_in_path import include, FF_PROJECT_ROOT

    include(FF_PROJECT_ROOT, "vendor", "TanksAndTemples", "tanksandtemples_evaluator")
    from tanksandtemples_evaluator import TanksAndTemplesEvaluator
except:
    raise

SCENE_NAME = "angel"
SCENE_FOLDER = os.path.join(FF_PROJECT_ROOT, "scripts", "data", "uavmvs", "angel_v1")
RECONSTRUCTION_FOLDER = os.path.join(SCENE_FOLDER, "capture-shortened_2D", "Reconstruction")
assert os.path.isdir(RECONSTRUCTION_FOLDER), RECONSTRUCTION_FOLDER

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Runs tanksandtemples_evaluator.py on {SCENE_FOLDER}/"
    )

    parser.add_argument("--dtau", type=float, default=0.51)
    parser.add_argument("--out_dir", type=str, default=None)
    parser.add_argument("--plot_stretch", type=int, default=5)

    args = parser.parse_args()

    dTau = args.dtau
    plot_stretch = args.plot_stretch

    out_dir = os.path.join(RECONSTRUCTION_FOLDER, "Evaluation") if args.out_dir is None else args.out_dir
    os.makedirs(out_dir, exist_ok=True)

    precision, recall, fscore, *histogram_data = TanksAndTemplesEvaluator.evaluate_reconstruction(
        scene_name=SCENE_NAME,
        out_dir=out_dir,
        dTau=dTau,
        # Output from uavmvs:
        gt_ply_path=os.path.join(SCENE_FOLDER, f"{SCENE_NAME}-proxy_cloud.ply"),
        # Generated with evaluate_reconstruction.py from data captured in AirSim:
        gt_log_path=os.path.join(RECONSTRUCTION_FOLDER, "tanksandtemples.rec.log"),
        # Generated from Meshroom output with geometry_crop.py:
        est_ply_path=os.path.join(RECONSTRUCTION_FOLDER, "cropped_1.ply"),
        # Generated from Meshroom output with evaluate_reconstruction.py:
        est_log_path=os.path.join(RECONSTRUCTION_FOLDER, "tanksandtemples.sfm.log"),
        # Eyeballed in Meshlab from uavmvs and Meshroom outputs:
        align_txt_path=os.path.join(RECONSTRUCTION_FOLDER, "alignment2.txt"),
        # Generated from Meshroom output with geometry_crop.py:
        crop_json_path=os.path.join(RECONSTRUCTION_FOLDER, "cropped_1.json"),
        plot_stretch=plot_stretch,
    )

    print("==============================")
    print("evaluation result : %s" % SCENE_NAME)
    print("==============================")
    print("distance tau : %.3f" % dTau)
    print("precision : %.4f" % precision)
    print("recall : %.4f" % recall)
    print("f-score : %.4f" % fscore)
    print("==============================")

    edges_source, cum_source, edges_target, cum_target = histogram_data

    TanksAndTemplesEvaluator.plot_graph(
        SCENE_NAME,
        fscore,
        dTau,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
        plot_stretch,
        out_dir,
    )
