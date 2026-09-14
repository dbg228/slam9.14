import os
import numpy as np
import open3d as o3d

########################################
# User Config
########################################

# 获取当前工作目录（终端执行脚本时的目录）
current_dir = os.getcwd()
# 拼接当前目录下的 data 文件夹路径
data_dir = os.path.join(current_dir, "robot_data/maps/data/")
print(data_dir)
# 可选：确保 data 文件夹存在（避免路径不存在报错）
if not os.path.exists(data_dir):
    os.makedirs(data_dir)  # 不存在则创建

scan_dir = os.path.join(data_dir, "Scans")

#scan_idx_range = [0, 4]     # [start, end)
node_skip = 1
near_remove = True
near_thres = 2.0

use_voxel = False
voxel_size = 0.1

########################################


def load_poses(txt_path):
    poses = []
    with open(txt_path, "r") as f:
        for line in f:
            T = np.array(list(map(float, line.split()))).reshape(3, 4)
            T = np.vstack((T, [0, 0, 0, 1]))
            poses.append(T)
    return poses


def load_pcd_with_intensity(path):
    pcd = o3d.io.read_point_cloud(path)
    xyz = np.asarray(pcd.points)

    # 尝试从 colors 读 intensity（如果存在）
    if len(pcd.colors) == len(xyz):
        intensity = np.asarray(pcd.colors)[:, 0:1]
    else:
        intensity = np.ones((xyz.shape[0], 1)) * 0.5

    return xyz, intensity


def main():
    poses = load_poses(os.path.join(data_dir, "optimized_poses.txt"))
    scan_files = sorted(os.listdir(scan_dir))

    xyz_all = []
    intensity_all = []
    scan_idx_range = [0, len(poses)]
    for idx in range(scan_idx_range[0], scan_idx_range[1]):
        print(f"Processing scan {idx}")

        scan_path = os.path.join(scan_dir, scan_files[idx])
        xyz, intensity = load_pcd_with_intensity(scan_path)

        # 去近点
        if near_remove:
            d = np.linalg.norm(xyz, axis=1)
            mask = d > near_thres
            xyz = xyz[mask]
            intensity = intensity[mask]

        # 变换到世界坐标
        T = poses[idx]
        xyz_h = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
        xyz_world = (T @ xyz_h.T).T[:, :3]

        xyz_all.append(xyz_world)
        intensity_all.append(intensity)

    xyz_all = np.vstack(xyz_all)
    intensity_all = np.vstack(intensity_all)

    print("Total points:", xyz_all.shape[0])

    # 构建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_all)

    # intensity 映射到灰度颜色
    intensity_norm = (intensity_all - intensity_all.min()) / \
        (intensity_all.ptp() + 1e-6)
    colors = np.repeat(intensity_norm, 3, axis=1)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    if use_voxel:
        pcd = pcd.voxel_down_sample(voxel_size)

    out_path = os.path.join(data_dir, "scans.pcd")

    o3d.io.write_point_cloud(out_path, pcd)
    print("Saved:", out_path)


if __name__ == "__main__":
    main()
