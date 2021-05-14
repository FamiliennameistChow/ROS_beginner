import open3d as o3d
import os
import sys
import numpy as np
import math
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path: sys.path.remove(ros_cv2_path)
import cv2
import copy


data_path = os.path.join(sys.path[0], 'pcds')
pcd_ts_file = "pcd_ts.txt"
# ts_pose_file = "pose_ts_loam.txt"
ts_pose_file = "ground_truth.txt"
result_path = os.path.join(sys.path[0], 'result')
pcd_ts_path = os.path.join(sys.path[0], pcd_ts_file)
ts_pose_path = os.path.join(sys.path[0], ts_pose_file)
pcd_ts = dict()
ts = list()
poses = list()
step = 20


def get_transformation(data):
    """
    从数据中获得位姿矩阵
    """

    quaternion = [data[6], data[3], data[4], data[5]]
    # epsilon for testing whether a number is close to zero
    _EPS = np.finfo(float).eps * 4.0

    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], data[0]],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], data[1]],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], data[2]],
        [                0.0,                 0.0,                 0.0,    1.0]])


def read_pcd(index):
    pcd = o3d.io.read_point_cloud(os.path.join(data_path, "%06d.ply" % index))
    return pcd


# 从pose_ts_loam.txt读取位姿拼接激光点云
def get_poses():
    """
    获得点云的时间戳, 获得位姿的时间戳以及对应的位姿
    """
    with open(pcd_ts_path, 'r') as fp:
        lines = fp.readlines()
        cnt = 0
        for line in lines:
            pcd_ts[float(line)] = cnt
            cnt += 1

    with open(ts_pose_path, 'r') as fp:
        lines = fp.readlines()
        for line in lines:
            data = line.replace(':', '').split(' ')
            ts_q = [float(data[1]), float(data[2]), float(data[3]), float(data[4]), 
                    float(data[5]), float(data[6]), float(data[7])]
            tf = get_transformation(ts_q)
            ts.append(float(data[0]))
            poses.append(tf)


def get_pcd_result():
    pcd_result = o3d.geometry.PointCloud()
    cnt = 0
    for index in range(0, len(ts), step):
        if ts[index] in pcd_ts:
            print("pcd = ", pcd_ts[ts[index]])
            pcd = read_pcd(pcd_ts[ts[index]])
            pcd.transform(poses[index])
            pcd_result += pcd
            cnt += 1
    print("length of pcds: ", cnt)
    o3d.visualization.draw_geometries([pcd_result])
    # o3d.io.write_point_cloud(os.path.join(result_path, "pose_ts_result.ply"), pcd_result, write_ascii=False)
    o3d.io.write_point_cloud(os.path.join(result_path, "ground_truth_result.ply"), pcd_result, write_ascii=False)


def main():
    get_poses()
    get_pcd_result()


if __name__ == "__main__":
    main()