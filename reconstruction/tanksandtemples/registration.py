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

import copy
import numpy as np
import open3d as o3d

from .trajectory_io import convert_trajectory_to_pointcloud


MAX_POINT_NUMBER = 4e6


def trajectory_alignment(scene, traj, gt_traj, gt_trans):
    traj_pcd = convert_trajectory_to_pointcloud(traj)
    
    gt_traj_pcd = convert_trajectory_to_pointcloud(gt_traj)
    gt_traj_pcd.transform(gt_trans)
    
    correspondence = o3d.utility.Vector2iVector(np.asarray(list(
        map(lambda x: [x, x], range(len(gt_traj)))
    )))
    
    rr = o3d.registration.RANSACConvergenceCriteria()
    rr.max_iteration  = 100000
    rr.max_validation = 100000
    
    randomvar = 0.0
    n_of_cam_pos = len(traj_pcd.points)
    rand_number_added = np.asanyarray(traj_pcd.points) * (
        np.random.rand(n_of_cam_pos, 3) * randomvar - randomvar / 2 + 1
    )
    
    traj_pcd_rand = o3d.geometry.PointCloud()
    for elem in list(rand_number_added):
        traj_pcd_rand.points.append(elem)

    # Rough registration based on aligned data
    reg = o3d.registration.registration_ransac_based_on_correspondence(
        source=traj_pcd_rand,
        target=gt_traj_pcd,
        corres=correspondence,
        max_correspondence_distance=0.2,
        estimation_method=o3d.registration.TransformationEstimationPointToPoint(with_scaling=True),
        ransac_n=6,
        criteria=rr,
    )
    return reg.transformation


def __crop_and_downsample(
    pcd,
    crop_volume,
    down_sample_method="voxel",
    voxel_size=0.01,
    trans=np.identity(4),
):
    pcd_copy = copy.deepcopy(pcd)
    pcd_copy.transform(trans)
    pcd_crop = crop_volume.crop_point_cloud(pcd_copy)
    if down_sample_method == "voxel":
        # return voxel_down_sample(pcd_crop, voxel_size)
        return pcd_crop.voxel_down_sample(voxel_size)
    elif down_sample_method == "uniform":
        n_points = len(pcd_crop.points)
        if n_points > MAX_POINT_NUMBER:
            ds_rate = int(round(n_points / float(MAX_POINT_NUMBER)))
            return pcd_crop.uniform_down_sample(ds_rate)
    return pcd_crop


def registration_vol_ds(
    source, gt_target, init_trans,
    crop_volume, voxel_size,
    threshold, max_itr,
    verbose=True,
):
    if verbose:
        print(f"[Registration] voxel_size: {voxel_size}, threshold: {threshold}")
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    
    s = __crop_and_downsample(source, crop_volume, "voxel", voxel_size, init_trans)
    t = __crop_and_downsample(gt_target, crop_volume, "voxel", voxel_size) # trans=np.identity(4)
    
    reg = o3d.registration.registration_icp(
        s, t,
        max_correspondence_distance=threshold, init=np.identity(4),
        estimation_method=o3d.registration.TransformationEstimationPointToPoint(with_scaling=True),
        criteria=o3d.registration.ICPConvergenceCriteria(1e-6, max_itr),
    )
    reg.transformation = np.matmul(reg.transformation, init_trans)
    return reg


def registration_unif(
    source, gt_target, init_trans,
    crop_volume,
    threshold, max_itr, max_size=4 * MAX_POINT_NUMBER,
    verbose=True,
):
    if verbose:
        print(f"[Registration] threshold: {threshold}")
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    
    s = __crop_and_downsample(source, crop_volume, "uniform", init_trans)
    t = __crop_and_downsample(gt_target, crop_volume, "uniform") # trans=np.identity(4)
    
    reg = o3d.registration.registration_icp(
        s, t,
        max_correspondence_distance=threshold, init=np.identity(4),
        estimation_method=o3d.registration.TransformationEstimationPointToPoint(with_scaling=True),
        criteria=o3d.registration.ICPConvergenceCriteria(1e-6, max_itr),
    )
    reg.transformation = np.matmul(reg.transformation, init_trans)
    return reg