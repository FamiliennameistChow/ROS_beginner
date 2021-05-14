import os
import sys
import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

bag_file = '/home/ysc/Disk1/data/big_paper/virtual/014.big_ring.bag'
bag = rosbag.Bag(bag_file, "r")
bag_data = bag.read_messages('/velodyne_points')
pcd_path = os.path.join(sys.path[0], "pcds")
ts_file = os.path.join(sys.path[0], "pcd_ts.txt")
if not os.path.exists(pcd_path):
    os.makedirs(pcd_path)


def get_pcds():
    cnt = 0
    with open(ts_file, 'w') as fp:
        for topic, msg, t in bag_data:
            print("cnt = {}".format(cnt))
            lidar = pc2.read_points(msg)
            timestr = "%.6f" %  msg.header.stamp.to_sec()
            points = np.array(list(lidar))
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points[:,0:3].reshape(-1,3))
            o3d.io.write_point_cloud(os.path.join(pcd_path, "%06d.ply" % cnt), pcd, write_ascii=False)
            fp.write("{}\n".format(timestr))
            cnt += 1


def main():
    get_pcds()


if __name__ == "__main__":
    main()