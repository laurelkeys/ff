# ----------------------------------------------------------------------------
# -                   TanksAndTemples Website Toolbox                        -
# -                    http://www.tanksandtemples.org                        -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
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
#
# This python script is for downloading dataset from www.tanksandtemples.org
# The dataset has a different license, please refer to
# https://tanksandtemples.org/license/

import os
import copy
import json

import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d


def read_alignment_transformation(filename):
    with open(filename) as data_file:
        data = json.load(data_file)
    return np.asarray(data["transformation"]).reshape((4, 4)).transpose()


def write_color_distances(path, pcd, distances, max_distance):
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    # cmap = plt.get_cmap("afmhot")
    cmap = plt.get_cmap("hot_r")
    # cmap = plt.get_cmap("cividis") # TODO make this a parameter
    distances = np.array(distances)
    colors = cmap(np.minimum(distances, max_distance) / max_distance)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(path, pcd)


def EvaluateHisto(
    scene, filename_mvs,
    source, target, trans,
    crop_volume, voxel_size, threshold,
    plot_stretch,
    verbose=True,
):
    print("[EvaluateHisto]")
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    s = copy.deepcopy(source)
    s.transform(trans)
    s = crop_volume.crop_point_cloud(s)
    s = s.voxel_down_sample(voxel_size)
    s.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))

    t = copy.deepcopy(target)
    t = crop_volume.crop_point_cloud(t)
    t = t.voxel_down_sample(voxel_size)
    t.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))

    source_n_fn = os.path.join(filename_mvs, f"{scene}.precision.ply")
    target_n_fn = os.path.join(filename_mvs, f"{scene}.recall.ply")
    # Colorize the poincloud files prith the precision and recall values
    # o3d.io.write_point_cloud(source_n_fn, s)
    # o3d.io.write_point_cloud(target_n_fn, t)

    print("[compute_point_cloud_to_point_cloud_distance]")
    distance1 = s.compute_point_cloud_distance(t)
    print("[compute_point_cloud_to_point_cloud_distance]")
    distance2 = t.compute_point_cloud_distance(s)

    print("[ViewDistances] Add color coding to visualize error")
    write_color_distances(source_n_fn, s, distance1, 3 * threshold)

    print("[ViewDistances] Add color coding to visualize error")
    write_color_distances(target_n_fn, t, distance2, 3 * threshold)

    # Get histogram and f-score
    [
        precision,
        recall,
        fscore,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
    ] = get_f1_score_histo2(
        threshold, filename_mvs, plot_stretch, distance1, distance2
    )
    np.savetxt(os.path.join(filename_mvs, f"{scene}.recall.txt"), cum_target)
    np.savetxt(os.path.join(filename_mvs, f"{scene}.precision.txt"), cum_source)
    np.savetxt(
        os.path.join(filename_mvs, f"{scene}.prf_tau_plotstr.txt"),
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


def get_f1_score_histo2(
    threshold, filename_mvs, plot_stretch, distance1, distance2, verbose=True
):
    print("[get_f1_score_histo2]")

    dist_threshold = threshold
    len_d1, len_d2 = len(distance1), len(distance2)

    if len_d1 and len_d2:
        recall = sum(d < threshold for d in distance2) / len_d2
        precision = sum(d < threshold for d in distance1) / len_d1
        fscore = 2 * recall * precision / (recall + precision)

        bins = np.arange(0, dist_threshold * plot_stretch, dist_threshold / 100)
        hist, edges_source = np.histogram(distance1, bins)
        cum_source = np.cumsum(hist).astype(float) / len_d1

        bins = np.arange(0, dist_threshold * plot_stretch, dist_threshold / 100)
        hist, edges_target = np.histogram(distance2, bins)
        cum_target = np.cumsum(hist).astype(float) / len_d2

    else:
        precision, recall, fscore = 0, 0, 0
        edges_source, cum_source = np.array([0]), np.array([0])
        edges_target, cum_target = np.array([0]), np.array([0])

    return [
        precision,
        recall,
        fscore,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
    ]
