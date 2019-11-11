/*
* @file  : pose_pos_control.h
* @brief : class for position control
* @date  : 2019.01.09
* @author: kumaron
* 
*/
#ifndef _POSE_POSITION_CONTROL_H
#define _POSE_POSITION_CONTROL_H

//
// include files
//
#include <iostream>
#include <cmath>

//
// marco definitions
//
#define M_X_AXIS 1
#define M_Y_AXIS 2
#define M_Z_AXIS 3

#define M_LEFT      true
#define M_RIGHT     false

#define M_FORWARD   true
#define M_BACKWARD  false

#define M_UP        true
#define M_DOWN      false

//
// ADT
//
typedef struct PositionData{
    float x;
    float y;
    float z;
}PositionData;

//
// PositionControl class
//
class PositionControl{
private:
    PositionData *position_data;

public:
    //
    // Constructor and Destructor
    //
    PositionControl() {
        position_data = new PositionData();
        position_data->x = 0;
        position_data->y = 0;
        position_data->z = 0;
    }
    PositionControl(PositionData *set_pos_data) {
        position_data = new PositionData();
        position_data->x = set_pos_data->x;
        position_data->y = set_pos_data->y;
        position_data->z = set_pos_data->z;
    }

    //
    // Interfaces
    //
    // to control position
    PositionData* positionControl(int position_axis_type, bool position_direction_type, float control_theta);
};

extern void positionValueAssign(PositionData *set_pos_data, float set_pos_x, float set_pos_y, float set_pos_z);

#endif // _POSE_POSITION_CONTROL_H
