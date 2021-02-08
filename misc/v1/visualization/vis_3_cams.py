import os
import json
import numpy as np

def del_key_inplace(obj, bad_key):
    ''' Remove entries of `bad_key` from `obj` (inplace)\n
        ref.: https://stackoverflow.com/a/20692955 '''
    if isinstance(obj, dict):
        for key in list(obj.keys()):
            if key == bad_key:
                del obj[key]
            else:
                del_key_inplace(obj[key], bad_key)
    elif isinstance(obj, list):
        for i in reversed(range(len(obj))):
            if obj[i] == bad_key:
                del obj[i]
            else:
                del_key_inplace(obj[i], bad_key)
    else:
        # neither a dict nor a list, do nothing
        pass
    return obj

def del_keys_inplace(obj, bad_keys):
    for bad_key in bad_keys:
        del_key_inplace(obj, bad_key)
    return obj

# .sfm is actually a JSON
with open("data/sfm.sfm", 'r') as sfm_file:
    sfm = json.loads(sfm_file.read())

unused_keys = ['version', 'featuresFolders', 'matchesFolders', 'intrinsics', 'metadata', 'resectionId']
del_keys_inplace(sfm, unused_keys)

# TODO find the difference between viewId and poseId (which one is used as the observationId?)

def vis3(camIdx):
    header = (
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
    )
    ply = ""
    vertex_count = 0

    camera_colors = ["0 255 255", "255 255 0", "255 0 255"] # [Cyan, Yellow, Magenta]
    camera_paths = [views['path'] for views in sfm['views']][:3]
    camera_ids = [views['viewId'] for views in sfm['views']][:3]
    for viewId, path, color in zip(camera_ids, camera_paths, camera_colors):
        header += f"comment id {viewId}\ncomment path {path}\ncomment color {color}\n"

    camera_centers = [poses['pose']['transform']['center'] for poses in  sfm['poses']][:3]
    for center, color in zip(camera_centers, camera_colors):
        ply += f"{' '.join(map('{:.8f}'.format, map(float, center)))} {color}\n"
        vertex_count += 1

    point_structures = [(structure['color'], structure['X'], 
                        [observation['observationId'] for observation in structure['observations']]) 
                        for structure in sfm['structure']]
    for color, center, observationIds in point_structures:
        if camIdx is not None:
            point_color = ' '.join(color) if camera_ids[camIdx] not in observationIds else camera_colors[camIdx]
        else:
            point_color = ' '.join(color)
        ply += f"{' '.join(map('{:.8f}'.format, map(float, center)))} {point_color}\n"
        vertex_count += 1

    header = f"ply\nformat ascii 1.0\nelement vertex {vertex_count}\n" + header + "end_header\n"
    return header + ply

if __name__ == "__main__":
    with open("data/vis3_cams.ply", 'w') as ply_file:
        ply_file.write(vis3(None))
    # with open("data/vis3_cam0_cyan.ply", 'w') as ply_file:
    #     ply_file.write(vis3(0))
    # with open("data/vis3_cam1_yellow.ply", 'w') as ply_file:
    #     ply_file.write(vis3(1))
    # with open("data/vis3_cam2_magenta.ply", 'w') as ply_file:
    #     ply_file.write(vis3(2))