import os
import argparse

try:
    from include_in_path import include, FF_PROJECT_ROOT

    # include(FF_PROJECT_ROOT, "vendor", "TanksAndTemples", "tanksandtemples_evaluator")
    # from tanksandtemples_evaluator import TanksAndTemplesEvaluator

    include(FF_PROJECT_ROOT, "misc", "tools", "tanksandtemples_evaluator")
    from tanksandtemples_evaluator import TanksAndTemplesEvaluator
except:
    raise

SCENE_NAME = "angel"
SCENE_FOLDER = os.path.join(FF_PROJECT_ROOT, "scripts", "data", "uavmvs", "angel_v1", "capture-shortened_2D")

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

    out_dir = os.path.join(SCENE_FOLDER, "drone", "data", "eval") if args.out_dir is None else args.out_dir
    os.makedirs(out_dir, exist_ok=True)

    precision, recall, fscore, *histogram_data = TanksAndTemplesEvaluator.evaluate_reconstruction(
        scene_name=SCENE_NAME,
        output_folder=out_dir,
        dtau_threshold=dTau,
        # NOTE use the same source point cloud as eval_uavmvs_angel_v1_shortened_2D.py
        source_ply_path=os.path.join(SCENE_FOLDER, "drone", "data", "cropped_sfm.ply"),  # reconstruction
        target_ply_path=os.path.join(SCENE_FOLDER, "Reconstruction", "cropped_proxy_cloud.ply"),  # "ground-truth"
        # NOTE this is computed with MeshLab, and roughly aligns source_ply_path to target_ply_path:
        source_ply_to_ply_align_txt_path=os.path.join(SCENE_FOLDER, "drone", "data", "align_source_to_target.txt"),
        skip_refinement=True,
        plot_stretch=plot_stretch,
        verbose=True,
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
