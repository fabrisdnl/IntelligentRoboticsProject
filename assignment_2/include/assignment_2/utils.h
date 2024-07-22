#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

struct RPY
{
	double roll;
	double pitch;
	double yaw;
};

geometry_msgs::Point createPoint(const float& x, const float& y, const float& z);

bool lookAt(const float& head_joint1, const float& head_joint2);
bool moveTiagoTorso(float joint);

RPY convertQuaternionToRPY(geometry_msgs::Quaternion& quaternion);
geometry_msgs::Quaternion convertRPYToQuaternion(RPY& rpy);

bool performGroupPlanMotion(moveit::planning_interface::MoveGroupInterface& group_plan, geometry_msgs::Pose& pose);
bool performGroupPlanMotion(moveit::planning_interface::MoveGroupInterface& group_plan, std::vector<double>& joints_position);
bool performGroupPlanLinearMotion(moveit::planning_interface::MoveGroupInterface& group_plan, geometry_msgs::Pose& init_pose, geometry_msgs::Pose& final_pose);

geometry_msgs::Pose poseToFootprint(const geometry_msgs::Pose& pose);
geometry_msgs::Pose poseToMap(const geometry_msgs::Pose& pose);

#endif
