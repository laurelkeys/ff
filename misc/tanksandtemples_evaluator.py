# NOTE this is a modified, single-file implementation of the evaluation scripts

# ----------------------------------------------------------------------------
# -                   TanksAndTemples Website Toolbox                        -
# -                    http://www.tanksandtemples.org                        -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2021
# Tiago Chaves <t187690@dac.unicamp.br>
#
# Copyright (c) 2017
# Arno Knapitsch <arno.knapitsch@gmail.com >
# Jaesik Park <syncle@gmail.com>
# Qian-Yi Zhou <Qianyi.Zhou@gmail.com>
# Vladlen Koltun <vkoltun@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# ----------------------------------------------------------------------------


import os
import copy
import argparse

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


# ----------------------------------------------------------------------------


class Trajectory:
    def __init__(self, camera_poses=None):
        self.camera_poses = [] if camera_poses is None else camera_poses

    class CameraPose:
        def __init__(self, meta, mat):
            self.metadata = meta
            self.pose = mat

        def __str__(self):
            return "Metadata : %s\nPose : \n%s" % (
                " ".join(map(str, self.metadata)),
                np.array_str(self.pose),
            )

    @staticmethod
    def read(log_file_path):
        traj = []
        with open(log_file_path, "r") as f:
            metastr = f.readline()
            while metastr:
                metadata = map(int, metastr.split())
                mat = np.zeros(shape=(4, 4))
                for i in range(4):
                    mat[i, :] = np.fromstring(f.readline(), dtype=float, sep=" \t")
                traj.append(Trajectory.CameraPose(metadata, mat))
                metastr = f.readline()
        return Trajectory(traj)

    def write(self, log_file_path):
        with open(log_file_path, "w") as f:
            for x in self.camera_poses:
                p = x.pose.tolist()
                f.write(" ".join(map(str, x.metadata)) + "\n")
                f.write("\n".join(" ".join(map("{0:.12f}".format, p[i])) for i in range(4)))
                f.write("\n")

    def point_cloud(self):
        pcd = o3d.geometry.PointCloud()
        for x in self.camera_poses:
            pcd.points.append(x.pose[:3, 3])
        return pcd


# ----------------------------------------------------------------------------


def crop_pcd(pcd, crop_volume, trans=None):
    pcd_copy = copy.deepcopy(pcd)
    if trans is not None:
        pcd_copy.transform(trans)
    return crop_volume.crop_point_cloud(pcd_copy)


def uniform_downsample(pcd, max_points):
    if len(pcd.points) > max_points:
        every_k_points = int(round(len(pcd.points) / max_points))
        return pcd.uniform_down_sample(every_k_points)
    return pcd


def voxel_downsample(pcd, voxel_size=0.01):
    return pcd.voxel_down_sample(voxel_size)


def uniform_registration(
    source, target, init_trans, crop_volume, threshold, max_iter, max_size=None, verbose=True
):
    if max_size is not None:
        max_points = max_size // 4
    else:
        max_size, max_points = 16e6, 4e6

    if verbose:
        print("[Registration] threshold: %f, max_size: %d" % (threshold, max_size))
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    s = uniform_downsample(crop_pcd(source, crop_volume, init_trans), max_points)
    t = uniform_downsample(crop_pcd(target, crop_volume), max_points)

    reg = o3d.registration.registration_icp(
        s,
        t,
        threshold,
        np.identity(4),
        o3d.registration.TransformationEstimationPointToPoint(True),  # with_scaling
        o3d.registration.ICPConvergenceCriteria(1e-6, max_iter),
    )

    reg.transformation = np.matmul(reg.transformation, init_trans)
    return reg


def voxel_registration(
    source, target, init_trans, crop_volume, threshold, max_iter, voxel_size, verbose=True
):
    if verbose:
        print("[Registration] threshold: %f, voxel_size: %f" % (threshold, voxel_size))
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    s = voxel_downsample(crop_pcd(source, crop_volume, init_trans), voxel_size)
    t = voxel_downsample(crop_pcd(target, crop_volume), voxel_size)

    reg = o3d.registration.registration_icp(
        s,
        t,
        threshold,
        np.identity(4),
        o3d.registration.TransformationEstimationPointToPoint(True),  # with_scaling
        o3d.registration.ICPConvergenceCriteria(1e-6, max_iter),
    )

    reg.transformation = np.matmul(reg.transformation, init_trans)
    return reg


def trajectory_alignment(map_file, est_traj, gt_traj, gt_to_est_transform, randomvar=0):
    assert len(est_traj.camera_poses) <= 1600 or map_file is None  # TODO use video frames

    est_traj_pcd = est_traj.point_cloud()  # trajectory to register

    gt_traj_pcd = gt_traj.point_cloud()
    gt_traj_pcd.transform(gt_to_est_transform)

    rand_number_added = np.asanyarray(est_traj_pcd.points)
    if randomvar != 0:
        n_of_cam_pos = len(est_traj_pcd.points)
        rand_number_added *= np.random.rand(n_of_cam_pos, 3) * randomvar - randomvar / 2 + 1

    est_traj_pcd_rand = o3d.geometry.PointCloud()
    for elem in list(rand_number_added):
        est_traj_pcd_rand.points.append(elem)

    # Rough registration based on aligned SfM data
    corres = o3d.utility.Vector2iVector(
        np.asarray([[x, x] for x in range(len(gt_traj.camera_poses))])
    )

    rr = o3d.registration.RANSACConvergenceCriteria()
    rr.max_iteration = 100000
    rr.max_validation = 100000

    reg = o3d.registration.registration_ransac_based_on_correspondence(
        est_traj_pcd_rand,
        gt_traj_pcd,
        corres,
        0.2,
        o3d.registration.TransformationEstimationPointToPoint(True),  # with_scaling
        6,
        rr,
    )
    return reg.transformation


# ----------------------------------------------------------------------------


class TanksAndTemplesEvaluator:
    @staticmethod
    def evaluate_histogram(
        source,
        target,
        trans,
        crop_volume,
        voxel_size,
        threshold,
        out_dir_path,
        plot_stretch,
        scene_name,
        verbose,
    ):
        if verbose:
            print("[EvaluateHisto]")
            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
            if_verbose_print = lambda *args, **kwargs: print(*args, **kwargs)
        else:
            if_verbose_print = lambda *args, **kwargs: None

        scene_file_base = os.path.join(out_dir_path, scene_name)
        if_verbose_print(scene_file_base + ".precision.ply")

        s = copy.deepcopy(source)
        s.transform(trans)
        s = crop_volume.crop_point_cloud(s)
        s = s.voxel_down_sample(voxel_size)
        s.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))

        t = copy.deepcopy(target)
        t = crop_volume.crop_point_cloud(t)
        t = t.voxel_down_sample(voxel_size)
        t.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))

        if_verbose_print("[compute_point_cloud_to_point_cloud_distance]")
        distance1 = s.compute_point_cloud_distance(t)
        assert len(distance1)

        if_verbose_print("[compute_point_cloud_to_point_cloud_distance]")
        distance2 = t.compute_point_cloud_distance(s)
        assert len(distance2)

        # Write the distances to bin files
        # np.array(distance1).astype("float64").tofile(scene_file_base + ".precision.bin")
        # np.array(distance2).astype("float64").tofile(scene_file_base + ".recall.bin")

        # Colorize the point cloud files with the precision and recall values
        # o3d.io.write_point_cloud(scene_file_base + ".precision.ply", s)
        # o3d.io.write_point_cloud(scene_file_base + ".recall.ply", t)

        source_n_fn = scene_file_base + ".precision.ply"
        target_n_fn = scene_file_base + ".recall.ply"

        # def viewDT(path):
        #     eval_str_viewDT = (
        #         OPEN3D_EXPERIMENTAL_BIN_PATH
        #         + "ViewDistances " + path
        #         + " --max_distance " + str(threshold * 3)
        #         + " --write_color_back --without_gui"
        #     )
        #     os.system(eval_str_viewDT)

        def write_color_distances(path, pcd, distances, max_distance):
            cmap = plt.get_cmap("hot_r")  # "afmhot")
            colors = cmap(np.minimum(np.array(distances), max_distance) / max_distance)[:, :3]
            pcd.colors = o3d.utility.Vector3dVector(colors)
            o3d.io.write_point_cloud(path, pcd)

        if_verbose_print("[ViewDistances] Add color coding to visualize error")
        # viewDT(source_n_fn)
        write_color_distances(source_n_fn, s, distance1, 3 * threshold)

        if_verbose_print("[ViewDistances] Add color coding to visualize error")
        # viewDT(target_n_fn)
        write_color_distances(target_n_fn, t, distance2, 3 * threshold)

        # Get F-score and histogram
        if_verbose_print("[get_f1_score_histo2]")

        precision = sum(d < threshold for d in distance1) / len(distance1)
        recall = sum(d < threshold for d in distance2) / len(distance2)
        fscore = 2 * recall * precision / (recall + precision)

        bins = np.arange(0, threshold * plot_stretch, threshold / 100)
        hist_source, edges_source = np.histogram(distance1, bins)
        cum_source = np.cumsum(hist_source).astype(float) / len(distance1)
        hist_target, edges_target = np.histogram(distance2, bins)
        cum_target = np.cumsum(hist_target).astype(float) / len(distance2)

        np.savetxt(scene_file_base + ".recall.txt", cum_target)
        np.savetxt(scene_file_base + ".precision.txt", cum_source)
        np.savetxt(
            scene_file_base + ".prf_tau_plotstr.txt",
            np.array([precision, recall, fscore, threshold, plot_stretch]),
        )

        return [
            precision,
            recall,
            fscore,
            edges_source,
            cum_source,
            edges_target,
            cum_target,
        ]

    @staticmethod
    def evaluate_reconstruction(
        scene_name,
        out_dir,
        dTau,
        gt_ply_path,  # Ground-truth, i.e. reference / target
        gt_log_path,
        est_ply_path,  # Estimate reconstruction, i.e. source
        est_log_path,
        align_txt_path,  # Alignment transformation matrix
        crop_json_path,  # Area cropping for the PLY files
        plot_stretch,
        map_file=None,
        verbose=True,
    ):
        # Load reconstruction and according ground-truth
        est_pcd = o3d.io.read_point_cloud(est_ply_path)  # "source"
        gt_pcd = o3d.io.read_point_cloud(gt_ply_path)  # "target"

        transform = np.loadtxt(align_txt_path)
        est_traj = Trajectory.read(est_log_path)  # trajectory to register
        gt_traj = Trajectory.read(gt_log_path)

        traj_transformation = trajectory_alignment(map_file, est_traj, gt_traj, transform)

        # Refine alignment by using the actual 'gt' and 'est' point clouds
        # Big point clouds will be downsampled to 'dTau' to speed up alignment
        vol = o3d.visualization.read_selection_polygon_volume(crop_json_path)

        # Registration refinement in 3 iterations
        r2 = voxel_registration(
            est_pcd,
            gt_pcd,
            init_trans=traj_transformation,
            crop_volume=vol,
            threshold=dTau * 80,
            max_iter=20,
            voxel_size=dTau,
            verbose=verbose,
        )
        r3 = voxel_registration(
            est_pcd,
            gt_pcd,
            init_trans=r2.transformation,
            crop_volume=vol,
            threshold=dTau * 20,
            max_iter=20,
            voxel_size=dTau / 2,
            verbose=verbose,
        )
        r = uniform_registration(
            est_pcd,
            gt_pcd,
            init_trans=r3.transformation,
            crop_volume=vol,
            threshold=2 * dTau,
            max_iter=20,
            verbose=verbose,
        )

        # Generate histograms and compute P/R/F1
        # Returns: [precision, recall, fscore, edges_source, cum_source, edges_target, cum_target]
        return TanksAndTemplesEvaluator.evaluate_histogram(
            est_pcd,
            gt_pcd,
            trans=r.transformation,
            crop_volume=vol,
            voxel_size=dTau / 2,
            threshold=dTau,
            out_dir_path=out_dir,
            plot_stretch=plot_stretch,
            scene_name=scene_name,
            verbose=verbose,
        )

    @staticmethod
    def plot_graph(
        scene_name,
        fscore,
        threshold,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
        plot_stretch,
        out_dir_path,
        show_figure=False,
    ):
        f = plt.figure()
        plt_size = [14, 7]
        pfontsize = "medium"

        ax = plt.subplot(111)
        label_str = "precision"
        ax.plot(edges_source[1::], cum_source * 100, c="red", label=label_str, linewidth=2)

        label_str = "recall"
        ax.plot(edges_target[1::], cum_target * 100, c="blue", label=label_str, linewidth=2)

        from cycler import cycler

        ax.grid(True)
        plt.rcParams["figure.figsize"] = plt_size
        plt.rc("axes", prop_cycle=cycler("color", ["r", "g", "b", "y"]))
        plt.title("Precision and Recall: " + scene_name + ", " + "%02.2f f-score" % (fscore * 100))
        plt.axvline(x=threshold, c="black", ls="dashed", linewidth=2)

        plt.ylabel("# of points (%)", fontsize=15)
        plt.xlabel("Meters", fontsize=15)
        plt.axis([0, threshold * plot_stretch, 0, 100])
        ax.legend(shadow=True, fancybox=True, fontsize=pfontsize)

        plt.setp(ax.get_legend().get_texts(), fontsize=pfontsize)

        plt.legend(loc=2, borderaxespad=0, fontsize=pfontsize)
        plt.legend(loc=4)
        plt.legend(loc="lower right")

        box = ax.get_position()
        ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

        # Put a legend to the right of the current axis
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        plt.setp(ax.get_legend().get_texts(), fontsize=pfontsize)

        name_no_ext = "PR_{0}_@d_th_0_{1}".format(scene_name, "%04d" % (threshold * 10000))
        png_name = os.path.join(out_dir_path, name_no_ext + ".png")
        pdf_name = os.path.join(out_dir_path, name_no_ext + ".pdf")

        # Save figure and display
        f.savefig(png_name, format="png", bbox_inches="tight")
        f.savefig(pdf_name, format="pdf", bbox_inches="tight")
        if show_figure:
            plt.show()


# ----------------------------------------------------------------------------


def run_evaluation(dataset_dir, traj_path, ply_path, out_dir, dTau=None):
    scene = os.path.basename(os.path.normpath(dataset_dir))

    if dTau is None:
        try:
            dTau = {
                "Barn": 0.01,
                "Caterpillar": 0.005,
                "Church": 0.025,
                "Courthouse": 0.025,
                "Ignatius": 0.003,
                "Meetingroom": 0.01,
                "Truck": 0.005,
            }[scene]
        except KeyError:
            print(dataset_dir, scene)
            raise Exception("invalid dataset-dir, not in scenes_tau_dict")

    print("")
    print("===========================")
    print("Evaluating %s" % scene)
    print("===========================")

    # put the crop-file, the GT file, the COLMAP SfM log file and
    # the alignment of the according scene in a folder of
    # the same scene name in the dataset_dir
    colmap_ref_logfile = os.path.join(dataset_dir, scene + "_COLMAP_SfM.log")
    alignment = os.path.join(dataset_dir, scene + "_trans.txt")
    gt_filen = os.path.join(dataset_dir, scene + ".ply")
    cropfile = os.path.join(dataset_dir, scene + ".json")
    map_file = os.path.join(dataset_dir, scene + "_mapping_reference.txt")

    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    print(ply_path)
    print(gt_filen)
    plot_stretch = 5

    [
        precision,
        recall,
        fscore,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
    ] = TanksAndTemplesEvaluator.evaluate_reconstruction(
        scene,
        out_dir,
        dTau,
        gt_ply_path=gt_filen,
        gt_log_path=colmap_ref_logfile,
        est_ply_path=ply_path,
        est_log_path=traj_path,
        align_txt_path=alignment,
        crop_json_path=cropfile,
        plot_stretch=plot_stretch,
        map_file=map_file,
        verbose=os.getenv("VERBOSE", None) is not None,
    )

    print("==============================")
    print("evaluation result : %s" % scene)
    print("==============================")
    print("distance tau : %.3f" % dTau)
    print("precision : %.4f" % precision)
    print("recall : %.4f" % recall)
    print("f-score : %.4f" % fscore)
    print("==============================")

    TanksAndTemplesEvaluator.plot_graph(
        scene,
        fscore,
        dTau,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
        plot_stretch,
        out_dir,
    )


# ----------------------------------------------------------------------------


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-dir",
        type=str,
        required=True,
        help="path to a dataset/scene directory containing X.json, X.ply, ...",
    )
    parser.add_argument(
        "--traj-path",
        type=str,
        required=True,
        help="path to trajectory file (see `convert_to_logfile.py` to create this file)",
    )
    parser.add_argument(
        "--ply-path",
        type=str,
        required=True,
        help="path to reconstruction ply file",
    )
    parser.add_argument(
        "--out-dir",
        type=str,
        default="",
        help="output directory (default: an evaluation directory is created in the directory of the ply file)",
    )
    args = parser.parse_args()

    if args.out_dir.strip() == "":
        args.out_dir = os.path.join(os.path.dirname(args.ply_path), "evaluation")

    run_evaluation(
        dataset_dir=args.dataset_dir,
        traj_path=args.traj_path,
        ply_path=args.ply_path,
        out_dir=args.out_dir,
    )
