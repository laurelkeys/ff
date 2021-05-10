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
RECONSTRUCTION_FOLDER = os.path.join(SCENE_FOLDER, "capture-initial_2D", "Reconstruction")
assert os.path.isdir(RECONSTRUCTION_FOLDER), RECONSTRUCTION_FOLDER

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Runs tanksandtemples_evaluator.py on {SCENE_FOLDER}/"
    )

    parser.add_argument("--dtau", type=float, default=0.5)
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
        gt_ply_path=os.path.join(RECONSTRUCTION_FOLDER, "cropped_proxy_cloud.ply"),
        gt_log_path=os.path.join(RECONSTRUCTION_FOLDER, "tanksandtemples.rec.log"),
        est_ply_path=os.path.join(RECONSTRUCTION_FOLDER, "cropped_sfm.ply"),
        est_log_path=os.path.join(RECONSTRUCTION_FOLDER, "tanksandtemples.sfm.log"),
        align_txt_path=os.path.join(RECONSTRUCTION_FOLDER, "tanksandtemples.align_rec_to_sfm.txt"),  # gt -> est
        crop_json_path=None,  # HACK (see the changes on TanksAndTemples "debug" branch)
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
