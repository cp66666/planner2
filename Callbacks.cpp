#include "Callbacks.h"

//copy value from some config here
double carto_origin_x, carto_origin_y;
double map_height_meter;

//global here
double slam_x, slam_y, slam_theta;

//
float ex_laser1[3];
float ex_laser2[3];

// =====================================================================
// ### Pose Callback ###
void poseCallback(commander_robot_msg::PoseStamped* msg)
{
}
// =====================================================================

void laser1_callback(commander_robot_msg::LaserScan* message)
{

}

void laser2_callback(commander_robot_msg::LaserScan* message)
{

}

void realsense1_callback(collisionavoidance_msg* recerived_data)
{

}

void robotcomm_callback(commander_msg::StdMsg* robotcomm_msg)
{

}
