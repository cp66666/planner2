#ifndef __CALLBACKS__
#define __CALLBACKS__

// ### Map
#include "LaserScan.h"
#include "LaserScanPubSubTypes.h"
#include "collisionavoidance_msg.h"
#include "collisionavoidance_msgPubSubTypes.h"

// ### Task
#include "downgoing_msg.h"
#include "downgoing_msgPubSubTypes.h"

// ### Planner
#include "collisionavoidance_msg.h"
#include "collisionavoidance_msgPubSubTypes.h"
#include "landmark_msg.h"
#include "landmark_msgPubSubTypes.h"
#include "std_msg.h"
#include "std_msgPubSubTypes.h"

// ### Main
#include "PoseStamped.h"
#include "PoseStampedPubSubTypes.h"
#include "config_msg.h"
#include "config_msgPubSubTypes.h"

/**
 * @brief
 */
void poseCallback(commander_robot_msg::PoseStamped* msg);

/**
 * @brief
 */
void laser1_callback(commander_robot_msg::LaserScan *message);

/**
 * @brief
 */
void laser2_callback(commander_robot_msg::LaserScan *message);

/**
 * @brief
 */
void realsense1_callback(collisionavoidance_msg *recerived_data);

#endif//__CALLBACKS__