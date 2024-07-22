#include "assignment_2/utils.h"
       
geometry_msgs::Point createPoint(const float& x, const float& y, const float& z)
{
	geometry_msgs::Point point;
	point.x = x; 
	point.y = y; 
	point.z = z;
	return point;
}

/* Function that receives the two tiago head joints, and move its head consequently. */        
bool lookAt(const float& head_joint1, const float& head_joint2)
{
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> point_head_client("/head_controller/follow_joint_trajectory", true);
	
	int iterations = 0, max_iterations = 3;
	while(!point_head_client.waitForServer(ros::Duration(5.0)) && iterations < max_iterations){
		ROS_INFO("Waiting for the point_head_action server to come up");
	}
	if ( iterations == max_iterations)
	{
		throw std::runtime_error("Error: head traj controller action server not available");
		return false;
	}
	
	control_msgs::FollowJointTrajectoryGoal head_goal;
	head_goal.trajectory.joint_names.push_back("head_1_joint");
	head_goal.trajectory.joint_names.push_back("head_2_joint");
	
	head_goal.trajectory.points.resize(1);
	int index = 0;
	head_goal.trajectory.points.at(index).positions.resize(2);
	head_goal.trajectory.points.at(index).positions.at(0) = head_joint1;
	head_goal.trajectory.points.at(index).positions.at(1) = head_joint2;
	
	head_goal.trajectory.points.at(index).velocities.resize(2);
	for (int i = 0; i < 2; ++i) head_goal.trajectory.points.at(index).velocities.at(i) = 0.0;
	
	head_goal.trajectory.points.at(index).time_from_start = ros::Duration(3.0);

	//send the goal
	ROS_INFO("Sending head goal");
	point_head_client.sendGoal(head_goal);
	point_head_client.sendGoal(head_goal);
	bool finished_before_timeout = point_head_client.waitForResult(ros::Duration(10.0));
	if (finished_before_timeout)
	{
		ROS_INFO("Head completed moving.");
		return true;
	}
	else
	{
		ROS_INFO("Head didn't finish before timeout.");
		return false;
	}
}

/* Function that moves tiago torso accordingly to the value of the joint of Tiago' torso specified. */        
bool moveTiagoTorso(float joint)
{
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client("/torso_controller/follow_joint_trajectory", true);
	// Wait for torso controller action server to come up
	int iterations = 0, max_iterations = 3;
	while(!torso_client.waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
	{
		ROS_DEBUG("Waiting for the torso controller action server to come up");
	}
	if (iterations == max_iterations)
	{
		throw std::runtime_error("Error: torso controller action server not available");
		return false;
	}

	control_msgs::FollowJointTrajectoryGoal torso_goal;
	torso_goal.trajectory.joint_names.push_back("torso_lift_joint");
	torso_goal.trajectory.points.resize(1);
	int index = 0;
	torso_goal.trajectory.points.at(index).positions.resize(1);
	torso_goal.trajectory.points.at(index).positions.at(0) = joint;
	torso_goal.trajectory.points.at(index).velocities.resize(1);
	torso_goal.trajectory.points.at(index).velocities.at(0) = 0.0;
	torso_goal.trajectory.points.at(index).time_from_start = ros::Duration(3.0);

	ROS_INFO("Sending torso goal");
	torso_client.sendGoal(torso_goal);
	bool finished_before_timeout = torso_client.waitForResult(ros::Duration(10.0));
	if (finished_before_timeout)
	{
		ROS_INFO("Torso completed moving.");
		return true;
	}
	else
	{
		ROS_INFO("Torso didn't finish before timeout.");
		return false;
	}
}

RPY convertQuaternionToRPY(geometry_msgs::Quaternion& quaternion)
{
	tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
	tf::Matrix3x3 mat(q);
	double r, p, y;
	mat.getRPY(r, p, y);
	return RPY{r, p, y};
}

geometry_msgs::Quaternion convertRPYToQuaternion(RPY& rpy)
{
	return tf::createQuaternionMsgFromRollPitchYaw(rpy.roll, rpy.pitch, rpy.yaw);
}

/* Function to perform the arm and torso (depending on group_plan) motion given the pose to reach. */
bool performGroupPlanMotion(moveit::planning_interface::MoveGroupInterface& group_plan, geometry_msgs::Pose& pose)
{
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	/* getting the current state of the planning group */
	group_plan.setStartState(*group_plan.getCurrentState());
	
	group_plan.setPoseTarget(pose);
	
	moveit::core::MoveItErrorCode error = group_plan.plan(plan);
	if (error = moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_INFO("Plan found in time %f seconds.", plan.planning_time_);
		error = group_plan.move();
		if (error = moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_INFO("Motion executed.");
			return true;
		}
		else
		{
			ROS_INFO("Motion failed with error: %i.", error.val);
			return false;
		}
	}
	else
	{
		ROS_INFO("Planning failed with error: %i.", error.val);
		return false;
	}
}

/* Function to perform the arm and torso (depending on group_plan) motion given the group of joints' values to reach. */
bool performGroupPlanMotion(moveit::planning_interface::MoveGroupInterface& group_plan, std::vector<double>& joints_position)
{
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	/* getting the current state of the planning group */
	group_plan.setStartState(*group_plan.getCurrentState());
	
	group_plan.setJointValueTarget(joints_position);
	
	moveit::core::MoveItErrorCode error = group_plan.plan(plan);
	if (error = moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_INFO("Plan found in time %f seconds.", plan.planning_time_);
		error = group_plan.move();
		if (error = moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_INFO("Motion executed.");
			return true;
		}
		else
		{
			ROS_INFO("Motion failed with error: %i.", error.val);
			return false;
		}
	}
	else
	{
		ROS_INFO("Planning failed with error: %i.", error.val);
		return false;
	}
}

/* Function to perform the arm and torso (depending on group_plan) "linear" motion given the group of joints' values to reach. */
bool performGroupPlanLinearMotion(moveit::planning_interface::MoveGroupInterface& group_plan, geometry_msgs::Pose& init_pose, geometry_msgs::Pose& final_pose)
{
	std::vector<geometry_msgs::Pose> path;
	path.push_back(init_pose);
	path.push_back(final_pose);
	
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	/* getting the current state of the planning group */
	group_plan.setStartState(*group_plan.getCurrentState());
	
	group_plan.setPoseTarget(final_pose);
	group_plan.setMaxVelocityScalingFactor(0.5);
	group_plan.setMaxAccelerationScalingFactor(0.5);
	
	moveit_msgs::RobotTrajectory traj;
	double eef_step = 0.01;
	double jump_threshold = 0.0;
	double path_fraction = group_plan.computeCartesianPath(path, eef_step, jump_threshold, traj, true);
	if (path_fraction == -1.0)
	{
		ROS_INFO("No linear plan found.");
		return false;
	}
	else
	{
		ROS_INFO("Linear plan found.");
		plan.trajectory_ = traj;
		moveit::core::MoveItErrorCode error = group_plan.move();
		// resetting factors
		group_plan.setMaxVelocityScalingFactor(1.0);
		group_plan.setMaxAccelerationScalingFactor(1.0);
		// check
		if (error = moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_INFO("Motion executed.");
			return true;
		}
		else
		{
			ROS_INFO("Motion failed with error: %i.", error.val);
			return false;
		}
	}
	
	
}

/* Function to perform the transform operation to the given pose. */
geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, const tf::StampedTransform& transform) 
        {
            tf::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
            v = transform * v;
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            q = transform * q;
            geometry_msgs::Pose result;
            result.position.x = v.x();
            result.position.y = v.y();
            result.position.z = v.z();
            result.orientation.x = q.x();
            result.orientation.y = q.y();
            result.orientation.z = q.z();
            result.orientation.w = q.w();
            return result;
        }

/* Function to perform the transform from base_footprint frame to map frame. */
geometry_msgs::Pose poseToMap(const geometry_msgs::Pose& pose)
		{
			tf::TransformListener listener;
            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(5.0));
                listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("Exception during transformation: %s", ex.what());
            }
            return transformPose(pose, transform);
		}

/* Function to perform the transform from map frame to base_footprint frame. */
geometry_msgs::Pose poseToFootprint(const geometry_msgs::Pose& pose)
		{
			tf::TransformListener listener;
            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("base_footprint", "map", ros::Time(0), ros::Duration(5.0));
                listener.lookupTransform("base_footprint", "map", ros::Time(0), transform);
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("Exception during transformation: %s", ex.what());
            }
            return transformPose(pose, transform);
		}
