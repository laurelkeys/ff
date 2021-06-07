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


try:
    o3d_registration = o3d.registration
    assert int(o3d.__version__.split(".")[1]) <= 10
except AttributeError:
    o3d_registration = o3d.pipelines.registration
    assert int(o3d.__version__.split(".")[1]) > 10 or int(o3d.__version__.split(".")[0]) > 0


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

    # Trajectory file (.log) format parse reference:
    # http://redwood-data.org/indoor/fileformat.html

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


def cropped_pcd(pcd, crop_volume, transformation=None):
    pcd_copy = copy.deepcopy(pcd)
    if transformation is not None:
        pcd_copy.transform(transformation)

    if crop_volume is None:
        return pcd_copy
    if isinstance(crop_volume, o3d.geometry.AxisAlignedBoundingBox):
        return pcd_copy.crop(crop_volume)
    else:
        assert isinstance(crop_volume, o3d.visualization.SelectionPolygonVolume)
        return crop_volume.crop_point_cloud(pcd_copy)


def uniform_downsample(pcd, max_points):
    if len(pcd.points) > max_points:
        every_k_points = int(round(len(pcd.points) / max_points))
        return pcd.uniform_down_sample(every_k_points)
    return pcd


def voxel_downsample(pcd, voxel_size=0.01):
    return pcd.voxel_down_sample(voxel_size)


def uniform_registration(
    source_pcd,
    target_pcd,
    source_align,
    target_crop_volume,
    threshold,
    max_iteration,
    max_size=None,
    verbose=True,
):
    if max_size is not None:
        max_points = max_size // 4
    else:
        max_size, max_points = int(16e6), int(4e6)

    if verbose:
        print("[Registration] max_size: %d, threshold: %f" % (max_size, threshold))
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    # NOTE source_align takes source_pcd to the same coordinate system as target_pcd,
    # hence why we use target_crop_volume for both (since it is in the target frame).
    source_pcd = cropped_pcd(source_pcd, target_crop_volume, source_align)
    target_pcd = cropped_pcd(target_pcd, target_crop_volume)

    registration = o3d_registration.registration_icp(
        source=uniform_downsample(source_pcd, max_points),
        target=uniform_downsample(target_pcd, max_points),
        max_correspondence_distance=threshold,
        init=np.identity(4),
        estimation_method=o3d_registration.TransformationEstimationPointToPoint(with_scaling=True),
        criteria=o3d_registration.ICPConvergenceCriteria(
            relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=max_iteration
        ),
    )

    # NOTE compose the source_align with the registration alignment matrix.
    return np.matmul(registration.transformation, source_align)


def voxel_registration(
    source_pcd,
    target_pcd,
    source_align,
    target_crop_volume,
    threshold,
    max_iteration,
    voxel_size,
    verbose=True,
):
    if verbose:
        print("[Registration] voxel_size: %d, threshold: %f" % (voxel_size, threshold))
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    # NOTE source_align takes source_pcd to the same coordinate system as target_pcd,
    # hence why we use target_crop_volume for both (since it is in the target frame).
    source_pcd = cropped_pcd(source_pcd, target_crop_volume, source_align)
    target_pcd = cropped_pcd(target_pcd, target_crop_volume)

    registration = o3d_registration.registration_icp(
        source=voxel_downsample(source_pcd, voxel_size),
        target=voxel_downsample(target_pcd, voxel_size),
        max_correspondence_distance=threshold,
        init=np.identity(4),
        estimation_method=o3d_registration.TransformationEstimationPointToPoint(with_scaling=True),
        criteria=o3d_registration.ICPConvergenceCriteria(
            relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=max_iteration
        ),
    )

    # NOTE compose the source_align with the registration alignment matrix.
    return np.matmul(registration.transformation, source_align)


# ----------------------------------------------------------------------------


def trajectory_alignment(
    source_traj,
    target_traj,
    target_align,
    randomvar=0,
    max_correspondence_distance=0.2,
    ransac_n=6,
):
    assert len(source_traj.camera_poses) <= 1600
    assert len(source_traj.camera_poses) == len(target_traj.camera_poses)
    camera_poses_len = len(source_traj.camera_poses)

    source_traj_pcd = source_traj.point_cloud()  # NOTE trajectory to register
    target_traj_pcd = target_traj.point_cloud().transform(target_align)  # XXX
    assert camera_poses_len == len(source_traj_pcd.points) == len(target_traj.points)

    rand_number_added = np.asanyarray(source_traj_pcd.points)
    rand_number_added *= np.random.rand(camera_poses_len, 3) * randomvar - randomvar / 2 + 1
    source_traj_pcd_rand = o3d.geometry.PointCloud()
    source_traj_pcd_rand.points.extend(list(rand_number_added))

    # Rough registration based on aligned structure from motion data
    corresponding_points = o3d.utility.Vector2iVector(
        np.asarray([[index, index] for index in range(camera_poses_len)])
    )

    # NOTE there's no random seed we can set for Open3D, so this might no be reproducible
    rr = o3d_registration.RANSACConvergenceCriteria()
    rr.max_iteration = 100000
    try:
        rr.max_validation = 100000
    except AttributeError:
        rr.confidence = 0.999

    registration = o3d_registration.registration_ransac_based_on_correspondence(
        source=source_traj_pcd_rand,
        target=target_traj_pcd,
        corres=corresponding_points,
        max_correspondence_distance=max_correspondence_distance,
        estimation_method=o3d_registration.TransformationEstimationPointToPoint(with_scaling=True),
        ransac_n=ransac_n,
        criteria=rr,
    )

    # NOTE this transformation takes source_traj to the same reference
    # frame as target_traj after being transformed by target_align XXX.
    return registration.transformation


# ----------------------------------------------------------------------------


class TanksAndTemplesEvaluator:
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
        output_folder,
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
        png_name = os.path.join(output_folder, name_no_ext + ".png")
        pdf_name = os.path.join(output_folder, name_no_ext + ".pdf")

        # Save figure and display
        f.savefig(png_name, format="png", bbox_inches="tight")
        f.savefig(pdf_name, format="pdf", bbox_inches="tight")
        if show_figure:
            plt.show()

    @staticmethod
    def evaluate_histogram(
        scene_name,
        output_folder,
        source_pcd,
        target_pcd,
        source_align,
        target_crop_volume,
        voxel_size,
        threshold,
        plot_stretch,
        verbose,
    ):
        if verbose:
            print("[EvaluateHisto]")
            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
            if_verbose_print = lambda *args, **kwargs: print(*args, **kwargs)
        else:
            if_verbose_print = lambda *args, **kwargs: None

        scene_file_base = os.path.join(output_folder, scene_name)
        if_verbose_print(scene_file_base + ".precision.ply")

        s = copy.deepcopy(source_pcd).transform(source_align)
        t = copy.deepcopy(target_pcd)

        if verbose:
            if target_crop_volume is not None:
                if isinstance(target_crop_volume, o3d.visualization.SelectionPolygonVolume):
                    orthogonal_axis_index = {"X": 0, "Y": 1, "Z": 2}[target_crop_volume.orthogonal_axis]
                    bounding_polygon = np.asarray(target_crop_volume.bounding_polygon)
                    min_bound, max_bound = bounding_polygon.min(axis=0), bounding_polygon.max(axis=0)
                    assert min_bound[orthogonal_axis_index] == max_bound[orthogonal_axis_index] == 0
                    min_bound[orthogonal_axis_index] = target_crop_volume.axis_min
                    max_bound[orthogonal_axis_index] = target_crop_volume.axis_max
                    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
                else:
                    assert isinstance(target_crop_volume, o3d.geometry.AxisAlignedBoundingBox)
                    aabb = target_crop_volume
                o3d.visualization.draw_geometries([s, t, aabb])
                o3d.visualization.draw_geometries([cropped_pcd(s, aabb), cropped_pcd(t, aabb), aabb])
            else:
                o3d.visualization.draw_geometries([s, t])

        if target_crop_volume is not None:
            s = cropped_pcd(s, target_crop_volume)
            t = cropped_pcd(t, target_crop_volume)
        s = s.voxel_down_sample(voxel_size)
        t = t.voxel_down_sample(voxel_size)
        s.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))
        t.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))

        if_verbose_print("[compute_point_cloud_to_point_cloud_distance]")
        distance_from_s_to_t = s.compute_point_cloud_distance(t)
        assert len(distance_from_s_to_t), distance_from_s_to_t

        if_verbose_print("[compute_point_cloud_to_point_cloud_distance]")
        distance_from_t_to_s = t.compute_point_cloud_distance(s)
        assert len(distance_from_t_to_s), distance_from_t_to_s

        # Write the distances to bin files
        # np.array(distance_from_s_to_t).astype("float64").tofile(scene_file_base + ".precision.bin")
        # np.array(distance_from_t_to_s).astype("float64").tofile(scene_file_base + ".recall.bin")

        source_n_fn = scene_file_base + ".precision.ply"
        target_n_fn = scene_file_base + ".recall.ply"

        # Colorize the point cloud files with the precision and recall values
        # o3d.io.write_point_cloud(source_n_fn, s)
        # o3d.io.write_point_cloud(target_n_fn, t)

        def write_color_distances(path, pcd, distances, max_distance):
            cmap = plt.get_cmap("hot_r")
            colors = cmap(np.minimum(np.array(distances), max_distance) / max_distance)[:, :3]
            pcd.colors = o3d.utility.Vector3dVector(colors)
            o3d.io.write_point_cloud(path, pcd)

        if_verbose_print("[ViewDistances] Add color coding to visualize error")
        write_color_distances(source_n_fn, s, distance_from_s_to_t, 3 * threshold)

        if_verbose_print("[ViewDistances] Add color coding to visualize error")
        write_color_distances(target_n_fn, t, distance_from_t_to_s, 3 * threshold)

        # Get F-score and histogram
        if_verbose_print("[get_f1_score_histo2]")

        # The precision quantifies the accuracy of the reconstruction: how closely the reconstructed points lie to the ground truth.
        # Precision alone can be maximized by producing a very sparse set of precisely localized landmarks.
        precision = sum(d < threshold for d in distance_from_s_to_t) / len(distance_from_s_to_t)

        # The recall quantifies the reconstruction's completeness: to what extent all the ground-truth points are covered.
        # Recall alone can be maximized by densely covering the space with points.
        recall = sum(d < threshold for d in distance_from_t_to_s) / len(distance_from_t_to_s)

        # Either of these schemes will drive the other measure and the F-score to 0.
        # A high F-score for a stringent distance threshold can only be achieved by a reconstruction that is both accurate and complete.
        fscore = 2 * recall * precision / (recall + precision)

        bins = np.arange(0, threshold * plot_stretch, threshold / 100)
        hist_source, edges_source = np.histogram(distance_from_s_to_t, bins)
        cum_source = np.cumsum(hist_source).astype(float) / len(distance_from_s_to_t)
        hist_target, edges_target = np.histogram(distance_from_t_to_s, bins)
        cum_target = np.cumsum(hist_target).astype(float) / len(distance_from_t_to_s)

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
        output_folder,
        dtau_threshold,
        source_ply_path,  # Estimate reconstruction
        target_ply_path,  # Ground-truth reconstruction
        # Axis-aligned bounding box in the ground-truth reference frame, used
        # to select the region of interest from the dense point clouds (.ply).
        target_crop_json_path=None,
        # If target_crop_json_path is None, the AABB of the target point cloud can
        # be computed and used to crop the source point cloud (ignored if not None).
        use_target_aabb_to_crop=False,
        # Transformation matrix that takes the (possibly arbitrary) reference frame
        # of target_log_path to the same coordinate system as target_ply_path.
        target_log_to_ply_align_txt_path=None,
        source_log_path=None,  # Camera positions in the same reference frame as source_ply_path
        target_log_path=None,  # Camera positions with (possibly) an arbitrary coordinate system
        # Transformation matrix used as a pre-alignment of source_ply_path to target_ply_path if
        # source_log_path or target_log_path are None (otherwise, this is ignored).
        source_ply_to_ply_align_txt_path=None,
        # If True, then the three steps of registration refinement are skipped. The pre-alignment
        # matrix is still applied if it is not None, though.
        skip_refinement=False,
        max_iteration=20,
        plot_stretch=5,
        verbose=True,
        # Function that can be applied to the source and target point clouds after they're parsed
        # with Open3D. Must have parameters ply_pcd: o3d.geometry.PointCloud, is_source_ply: bool
        ply_transform_fn=None,
    ):
        source_pcd = o3d.io.read_point_cloud(source_ply_path)
        target_pcd = o3d.io.read_point_cloud(target_ply_path)

        if ply_transform_fn is not None:
            ply_transform_fn(ply_pcd=source_pcd, is_source_ply=True)
            ply_transform_fn(ply_pcd=target_pcd, is_source_ply=False)

        if target_log_to_ply_align_txt_path is not None:
            assert source_log_path is not None  # "source" (the same as in `source_ply_path`)
            assert target_log_path is not None  # "target" (different from `target_ply_path`)
            source_traj = Trajectory.read(source_log_path)
            target_traj = Trajectory.read(target_log_path)
            target_traj_to_target_pcd_align = np.loadtxt(target_log_to_ply_align_txt_path)

            # Compute a rough pre-alignment using the camera positions from .log trajectories,
            # since source_traj and source_pcd are assumed to have the same coordinate system.
            source_pcd_to_target_pcd_align = trajectory_alignment(
                source_traj, target_traj, target_traj_to_target_pcd_align
            )

            source_align_0 = source_pcd_to_target_pcd_align
        else:
            assert source_log_path is None
            assert target_log_path is None

            if source_ply_to_ply_align_txt_path is not None:
                source_pcd_to_target_pcd_align = np.loadtxt(source_ply_to_ply_align_txt_path)
                source_align_0 = source_pcd_to_target_pcd_align
            else:
                source_align_0 = np.identity(4)

        if target_crop_json_path is not None:
            target_crop_volume = o3d.visualization.read_selection_polygon_volume(target_crop_json_path)
            assert np.asarray(target_crop_volume.bounding_polygon).shape == (4, 3)
        elif use_target_aabb_to_crop:
            target_crop_volume = target_pcd.get_axis_aligned_bounding_box()
        else:
            target_crop_volume = None

        if not skip_refinement:
            # Refine registration in three iterations.
            source_align_1 = voxel_registration(
                source_pcd,
                target_pcd,
                source_align_0,
                target_crop_volume,
                max_iteration=max_iteration,
                threshold=(dtau_threshold * 80),
                voxel_size=dtau_threshold,
                verbose=verbose,
            )
            source_align_2 = voxel_registration(
                source_pcd,
                target_pcd,
                source_align_1,
                target_crop_volume,
                max_iteration=max_iteration,
                threshold=(dtau_threshold * 20),
                voxel_size=(dtau_threshold / 2),
                verbose=verbose,
            )
            source_align_3 = uniform_registration(
                source_pcd,
                target_pcd,
                source_align_2,
                target_crop_volume,
                max_iteration=max_iteration,
                threshold=(2 * dtau_threshold),
                verbose=verbose,
            )
            source_align = source_align_3
        else:
            source_align = source_align_0

        # Generate histograms and compute P/R/F1
        # Returns: [precision, recall, fscore, edges_source, cum_source, edges_target, cum_target]
        return TanksAndTemplesEvaluator.evaluate_histogram(
            scene_name,
            output_folder,
            source_pcd,
            target_pcd,
            source_align,
            target_crop_volume,
            voxel_size=(dtau_threshold / 2),
            threshold=dtau_threshold,
            plot_stretch=plot_stretch,
            verbose=verbose,
        )


# ----------------------------------------------------------------------------


def print_evaluation_result(scene, dTau, precision, recall, fscore):
    print("==============================")
    print("evaluation result : %s" % scene)
    print("==============================")
    print("distance tau : %.3f" % dTau)
    print("precision : %.4f" % precision)
    print("recall : %.4f" % recall)
    print("f-score : %.4f" % fscore)
    print("==============================")


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
    assert not os.path.exists(map_file), "unimplemented!"

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
        scene_name=scene,
        output_folder=out_dir,
        dtau_threshold=dTau,
        source_ply_path=ply_path,
        target_ply_path=gt_filen,
        target_crop_json_path=cropfile,
        target_log_to_ply_align_txt_path=alignment,
        source_log_path=traj_path,
        target_log_path=colmap_ref_logfile,
        plot_stretch=plot_stretch,
        verbose=os.getenv("VERBOSE", None) is not None,
    )

    print_evaluation_result(scene, dTau, precision, recall, fscore)

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
