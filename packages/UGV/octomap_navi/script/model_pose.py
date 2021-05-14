# #!/usr/bin/env python
# # encoding: utf-8

# from gazebo_msgs.msg import ModelState
# import rospy
# import math

# def pose_publish():
#     # 改变模型pose
#     pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
#     pose_msg = ModelState()
#     pose_msg.model_name = 'scout/'
#     rate = rospy.Rate(30)
#     angle = 0
#     radius = 2
#     vel = 0.3
#     while not rospy.is_shutdown():
#         # 直线轨迹
#         # pose_msg.pose.position.x += 0.3 / 30     
#         # pose_msg.pose.position.y += 0.3 / 30
#         # pose_msg.reference_frame = 'camera_init'
#         pose_msg.reference_frame = 'map'

#         print(pose_msg.reference_frame)
#         rate.sleep()

# def main():
#     rospy.init_node('model_change', anonymous=True)
#     try:
#         pose_publish()
#     except rospy.ROSInterruptException:
#         pass

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelStateRequest, GetModelState
from geometry_msgs.msg import Pose
from  nav_msgs.msg  import Odometry
import os, sys

root_path = sys.path[0]
f = open(os.path.join(root_path, 'ground_truth.txt'), 'w')
get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name = 'scout/'

def callback(odometry):
    time_stamp = odometry.header.stamp.to_sec()
    objstate = get_state_service(model)
    data = (time_stamp, 
            objstate.pose.position.x, 
            objstate.pose.position.y,
            objstate.pose.position.z,
            objstate.pose.orientation.x,
            objstate.pose.orientation.y,
            objstate.pose.orientation.z,
            objstate.pose.orientation.w)
    print("{}: ".format(data[0]))
    print("{} {} {} {} {} {} {}".format(data[1], data[2], data[3], data[4], data[5], data[6], data[7]))
    f.write("{} {} {} {} {} {} {} {}\n".format(data[0], data[1], data[2], 
            data[3], data[4], data[5], data[6], data[7]))  # pcd_id, tx, ty, tz, qx, qy, qz, qw


def main():
    rospy.init_node('model_pose', anonymous=True)
    rate = rospy.Rate(5)
    rospy.Subscriber("/integrated_to_init", Odometry, callback)
    rospy.spin()
    f.close()
    print("\nfile is closed..")


if __name__ == "__main__":
    main()
