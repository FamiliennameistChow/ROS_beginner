#!/usr/bin/env python
# encoding: utf-8

from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import GetModelState
import rospy
import math
import os
from random import sample

def pose_publish(model_name, model_pose):
    # 改变模型pose
    pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    # print("model status ", pose_msg)
    pose_msg.model_name = model_name

    cout = 0
    while cout < 10:
        pose_msg.pose.position.x = model_pose[0]
        pose_msg.pose.position.y = model_pose[1]
        pose_msg.pose.position.z = model_pose[2]
        cout += 1

        # rate = rospy.Rate(30)
        pose_pub.publish(pose_msg)
        # rate.sleep()

def delete_model():
    # 删除模型
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        remove_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        remove_model_proxy("mark_label_h")
    except rospy.ServiceException, ex:
        print "Service call delete_model failed: %e" % ex

def add_model(model_name, model_pose, model_num):
    # 加载模型
    GAZEBO_MODEL_PATH = "~/.gazebo/models/"
    model_to_add = model_name# string
    x = str(model_pose[0])
    y = str(model_pose[1])
    # os.system("rosrun gazebo_ros spawn_model -file $GAZEBO_MODEL_PATH/" + model_to_add +"/model.sdf -sdf -model " + model_to_add + " -x 0 -y 0")
    os.system("rosrun gazebo_ros spawn_model -file " + GAZEBO_MODEL_PATH + model_to_add +"/model.sdf -sdf -model " + model_to_add + "_" + str(model_num) + " -x " + x + " -y " + y)
    print("add model:  " + model_name + "_"+ str(model_num))

def get_world_state():
    # 获取仿真环境状态
    rospy.wait_for_service('gazebo/get_world_properties')
    try:
        get_world_proxy = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
        resp = get_world_proxy()
        return resp
    except rospy.ServiceException, ex:
        print "Service call get_world_properties failed: %e" % ex

# rosservice call gazebo/get_model_state '{model_name: mark_label_1}'
# rosservice call gazebo/get_model_properties '{model_name: mark_label_1}'
def get_model_state():
    rospy.wait_for_service('gazebo/get_model_state')
    try:
        get_world_proxy = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        resp = get_world_proxy('mark_label_1', 'link')
        print resp
    except rospy.ServiceException, ex:
        print "Service call get_model_state failed: %e" % ex

def delete_mark_model():
    exist_model_list = get_world_state().model_names
    i = 0
    for exist_model in list(exist_model_list): #参考问题　https://www.cnblogs.com/leohahah/p/10981068.html 和　https://blog.csdn.net/yan_joy/article/details/79392950
        if 'mark_label' not in exist_model:
            exist_model_list.remove(exist_model)
    
    print("the follow mark model will be detele:\n", exist_model_list)

    rospy.wait_for_service('/gazebo/delete_model')
    try:
        remove_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for model in exist_model_list:
            remove_model_proxy(model)
    except rospy.ServiceException, ex:
        print "Service call delete_model failed: %e" % ex



def main():
    rospy.init_node('model_change', anonymous=True)
    # delete_model()

    ## load model 
    model_name_list = ['mark_label_square', 'mark_label_circle']
    # model_pose_list = [[2.5,2.5,0.15,0, 0, 0.785], 
    #                     [-2.5,2.5,0.15,0, 0, 0.785], 
    #                     [2.5,-2.5,0.15,0, 0, 0.785],
    #                     [-2.5,-2.5,0.15,0, 0, 0.785],
    #                     [2.5,2.5,0.15,0, 0, 0], 
    #                     [-2.5,2.5,0.15,0, 0, 0], 
    #                     [2.5,-2.5,0.15,0, 0, 0],
    #                     [-2.5,-2.5,0.15,0, 0, 0]]

    model_pose_list = [
                    [2.5,2.5,0.15,0, 0, 0], 
                    [-2.5,2.5,0.15,0, 0, 0], 
                    [2.5,-2.5,0.15,0, 0, 0],
                    [-2.5,-2.5,0.15,0, 0, 0]]

    dis = 5
    drone_pose_list = [[dis, 0, 0.115, 0, 0 , 0],
                        [-dis, 0, 0.115, 0, 0 , 0],
                        [0, dis, 0.115, 0, 0 , 0],
                        [0, -dis, 0.115, 0, 0 , 0]]

    drone_status = 0
    drone_pose_choiced_last = []
    while not rospy.is_shutdown():

        if drone_status == 0:
            print("===========================\n")
            try:
                input_status = input("input actions: 0 for init, 1 for load mark model, 2 for change drone pose, 3 for change mark model:\n")
            except:
                print("!!!!!! Please input num 0-3 !!!!!!!!!")
                continue
            # print("type", type(input_status))
            drone_status = input_status

        elif drone_status == 1:
            print("load mark model ....\n")
            model_pose_choiced = sample(model_pose_list, 4)
            model_choiced_dict = {}
            for i in range(4):
                model_name = sample(model_name_list, 1)[0]
                if model_name in model_choiced_dict:
                    model_choiced_dict[str(model_name)] += 1
                else:
                    model_choiced_dict[str(model_name)] = 0
                
                add_model(model_name, model_pose_choiced[i], model_choiced_dict[model_name])
                # print("choiced pose x type", str(model_pose_choiced[i][0]))
            
            print("model load done!!")
            drone_status = 0

        elif drone_status == 2:
            ## change drone pose
            print("change drone pose ....\n")
            drone_pose_choiced = sample(drone_pose_list, 1)[0]
            if drone_pose_choiced == drone_pose_choiced_last:
                drone_status = 2
                continue
            print("drone_pose_choiced",  drone_pose_choiced)
            try:
                pose_publish('iris', drone_pose_choiced)
            except rospy.ROSInterruptException:
                pass
            drone_pose_choiced_last = drone_pose_choiced
            drone_status = 0

        elif drone_status == 3:
            print("change mark pose ....\n")
            delete_mark_model()
            drone_status = 1


if __name__ == '__main__':
    main()
