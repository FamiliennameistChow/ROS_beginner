/*
* @file  : pose_yaw_control.h
* @brief : class for yaw control
* @date  : 2019.01.09
* @author: kumaron
* 
*/

#ifndef _POSE_YAW_CONTROL_H
#define _POSE_YAW_CONTROL_H

#include <iostream>
using namespace std;

#define M_ANTICLOCKWISE 0
#define M_CLOCKWISE     1

typedef struct Quaternion{
    float x;
    float y;
    float z;
    float w;
    int theta;
    Quaternion() {}
    Quaternion(float set_x, float set_y, float set_z, float set_w, int set_theta){
        x = set_x;
        y = set_y;
        z = set_z;
        w = set_w;
        theta = set_theta;
    }
    void showData(){
        cout << x << endl;
        cout << y << endl;
        cout << z << endl;
        cout << w << endl;
        cout << theta << endl;
        cout << endl;
    }
}Quaternion;

typedef struct YawQuaters{
    Quaternion *yaw_left;
    Quaternion *yaw_right;
    YawQuaters(){
        yaw_left = new Quaternion();
        yaw_right = new Quaternion();
    }
    YawQuaters(Quaternion *set_yaw_left, Quaternion *set_yaw_right){
        yaw_left = new Quaternion();
        yaw_right = new Quaternion();
        yaw_left->x = set_yaw_left->x;
        yaw_left->y = set_yaw_left->y;
        yaw_left->z = set_yaw_left->z;
        yaw_left->w = set_yaw_left->w;
        yaw_left->theta = set_yaw_left->theta;
        yaw_right->x = set_yaw_right->x;
        yaw_right->y = set_yaw_right->y;
        yaw_right->z = set_yaw_right->z;
        yaw_right->w = set_yaw_right->w;
        yaw_right->theta = set_yaw_right->theta;  
    }
}YawQuaters;

typedef struct ListNode{
    // Members
    YawQuaters *yawquaters;
    ListNode *prev;
    ListNode *next;

    //Constructor
    ListNode() {
        prev = this;
        next = this;
    }
    ListNode(Quaternion *set_yaw_left, Quaternion *set_yaw_right) {
        yawquaters = new YawQuaters(set_yaw_left, set_yaw_right);
        prev = this;
        next = this;
    }
}ListNode;

class List{
private:
    ListNode *head;
    ListNode *handle;
    int index;
public:
    //
    // Constructor
    //
    List(){
        head = new ListNode();
        handle = head;
        index = 0;
    }
    List(Quaternion *set_yaw_left, Quaternion *set_yaw_right){
        head = new ListNode(set_yaw_left, set_yaw_right);
        handle = head;
        index = 0;
    }

    //
    // Interfaces
    //
    // insert node as last to form a double circular linked list
    // param   : value to control yaw to left
    //           value to control yaw to right
    //
    void insertAsLast(Quaternion *set_yaw_left, Quaternion *set_yaw_right){
        index++;
        ListNode *value = new ListNode(set_yaw_left, set_yaw_right);
        head->prev->next = value;
        value->prev = head->prev;
        value->next = head;
        head->prev = value;        
    }

    // get theta from handle for position control
    // param   : 
    //           
    int getHandleTheta(){
        return handle->yawquaters->yaw_left->theta;
    }

    // yaw control according to desired yaw direction
    // param   : value indicating the yaw control direction
    //              
    Quaternion* yawControl(int yaw_direction_type){
        switch(yaw_direction_type)
        {
            case M_ANTICLOCKWISE:
                handle = handle->next;
                // cout << "yaw_left" << endl;
                // handle->yawquaters->yaw_left->showData();
                // cout << "yaw_right" << endl;
                // handle->yawquaters->yaw_right->showData();
                return handle->yawquaters->yaw_left;
                break;    
            case M_CLOCKWISE:
                handle = handle->prev;
                // cout << "yaw_left" << endl;
                // handle->yawquaters->yaw_left->showData();
                // cout << "yaw_right" << endl;
                // handle->yawquaters->yaw_right->showData();
                return handle->yawquaters->yaw_right;                   
                break;
            default:
                break;
        }
    }
};

typedef List YawControl;

extern void quaterDataAssign(Quaternion* set_yaw_data, float set_yaw_x, float set_yaw_y, float set_yaw_z, float set_yaw_w, int set_yaw_theta);

#endif