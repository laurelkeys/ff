import os
import argparse

try:
    from include_in_path import include
except:
    pass
finally:
    include("..", "..", "vendor", "TanksAndTemples", "tanksandtemples_evaluator")
    from tanksandtemples_evaluator import TanksAndTemplesEvaluator

SCENE_NAME = "Cidadela_2021-01-09-18-27-07"

# TODO use absolute paths
ASSETS_FOLDER = "assets"
EVAL_FOLDER = os.path.join("..", "..", "data", "recordings", SCENE_NAME)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Runs tanksandtemples_evaluator.py on {SCENE_NAME}/"
    )

    parser.add_argument("--dtau", type=float, default=0.01)
    parser.add_argument("--out_dir", type=str, default=None)
    parser.add_argument("--plot_stretch", type=int, default=5)

    args = parser.parse_args()

    dTau = args.dtau
    out_dir = args.out_dir
    plot_stretch = args.plot_stretch

    if out_dir is None:
        out_dir = os.path.join(EVAL_FOLDER, "evaluation")

    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # FIXME some file seems to be wrong
    [
        precision,
        recall,
        fscore,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
    ] = TanksAndTemplesEvaluator.evaluate_reconstruction(
        scene_name=SCENE_NAME,
        out_dir=out_dir,
        dTau=dTau,
        gt_ply_path=os.path.join(ASSETS_FOLDER, "CidadelaStatue_point_cloud.ply"),
        gt_log_path=os.path.join(EVAL_FOLDER, "tanksandtemples.rec.log"),
        est_ply_path=os.path.join(EVAL_FOLDER, "cropped_1.ply"),
        est_log_path=os.path.join(EVAL_FOLDER, "tanksandtemples.sfm.log"),
        align_txt_path=os.path.join(EVAL_FOLDER, "tanksandtemples.alignment.txt"),
        crop_json_path=os.path.join(EVAL_FOLDER, "cropped_1.json"),
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
