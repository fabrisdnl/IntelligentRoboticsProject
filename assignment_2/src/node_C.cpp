#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment_2/ArmAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <tf/transform_listener.h>
#include <math.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

#include "assignment_2/utils.h"

#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include "LinearMath/btMatrix3x3.h"

//command line arg
std::string arg;

/* Safe Pose after picking/placing */
geometry_msgs::PoseStamped safe_pose;


class ObjectManipulation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<assignment_2::ArmAction> as_;

        std::string name_;

        assignment_2::ArmResult result_;
        assignment_2::ArmFeedback feedback_;
        
        //table height
		float table_height = 0.78; // to be tuned
		float cylinder_height = 0.69; // to be tuned
		float cylinder_radius = 0.12; // to be tuned

		float joint_pos_1[8] = {0.34, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0};
		float joint_pos_2[8] = {0.34, 0.12, 0.55, -0.20, 1.56, -1.58, 0.25, 0.0};
		float joint_pos_3[8] = {0.34, 0.12, 1.5, -1.20, 1.56, 1.60, 0.25, 0.0};

		const float rad = M_PI/180;
		float safe_joints[8] = {0.34, 10*rad, -rad, -190*rad, 90*rad, 0.0, 0.0, 0.0};
		
		std::vector<std::string> collision_objects_ids;

        std::string status_[16] = 
        {
            "Robot is ready to start arm routine"
            //picking
            "Start the picking action: robot is moving the arm",
            "Arm is in position to pick up the target object with the end effector",
            "Arm picked the target object",
            "Arm failed to pick up the object"
            "Arm returned in its initial position",
            "The robot completed the picking routine",
            "The robot failed the picking routine",
            //placing
            "Start the placing action: robot is moving the arm",
            "Arm is in position to place the object",
            "Arm placed the object",
            "Arm failed to place the object"
            "Arm returned in its initial position",
            "The robot completed the placing routine",
            "The robot failed the placing routine",
            //obstacles
            "Arm collided with an obstacle",
        };

        /*
        **  Function that starts the picking/placing routine
        */
        bool armAction(const assignment_2::ArmGoalConstPtr &goal, bool action)
        {
        
            ROS_INFO("Goal Pose from goal (base_footprint): %f, %f, %f", goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);	
            
            moveit::planning_interface::PlanningSceneInterface scene;
            moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
            group_arm_torso.setPoseReferenceFrame("base_footprint");   
            
            /* Get Target Pose (Pick/Place) */
			geometry_msgs::PoseStamped goal_pose;
            
            //actions status
            bool completed_arm_movement = false;
            bool completed_gripper_movement = false;

            //TO DO: gripper position offset w.r.t target object/placing pose, based on the target ID and type of action?
            float x_offset = 0.0;
            float y_offset = 0.0;
            float z_offset = 0.3;

            //Picking
            if (!action) 
            {
                goal_pose.header.frame_id = "base_footprint";
                goal_pose.pose.position.x = goal->pose.position.x + x_offset;
                goal_pose.pose.position.y = goal->pose.position.y + y_offset;
                goal_pose.pose.position.z = goal->pose.position.z + z_offset;
                goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, 0.037);
                ROS_INFO("Orientation: %f, %f, %f, %f", goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);
                double roll, pitch, yaw;
                tf::Quaternion q(goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
		        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		        goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, yaw);
                
                /* set if you want fixed pose
                switch(goal->ID)
				{
					// blue
					case 1:
						goal_pose.pose.position.x = 0.54 + x_offset;
								goal_pose.pose.position.y = 0.11 + y_offset;
								goal_pose.pose.position.z = 0.87 + z_offset;
						break;
					// green
					case 2:
						goal_pose.pose.position.x = 0.56 + x_offset;
								goal_pose.pose.position.y = -0.075 + y_offset;
								goal_pose.pose.position.z = 0.8 + z_offset;
						break;
					// red
					case 3:
						goal_pose.pose.position.x = 0.55 + x_offset;
								goal_pose.pose.position.y = 0.027 + y_offset;
								goal_pose.pose.position.z = 0.82 + z_offset;
						break;
					
				}
				*/

                ROS_INFO("----- ADD COLLISION OBJECTS -----");
                 // add collision object to the scene
                addCollisionObjects(goal, scene, group_arm_torso);

                ROS_INFO("----- PRE-MOVEMENT POSITION -----");
                /* pre-movement pose */ 
				geometry_msgs::PoseStamped a;
				a.header.frame_id = "base_footprint";
                a.pose.position.x = 0.4;
                a.pose.position.y = 0.02;
                a.pose.position.z = 1.1;
                a.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, 0.037);
                ROS_INFO("Orientation: %f, %f, %f, %f", a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w);
                double rolla, pitcha, yawa;
                tf::Quaternion qa(goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
		        tf::Matrix3x3(qa).getRPY(rolla, pitcha, yawa);
		        a.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, yawa);
		        
		        if (!moveArm(a, group_arm_torso))
				{
					ROS_INFO("Failed to move arm");
					return false;
				}
				
                /* approach pose */
                ROS_INFO("----- APPROACH POSE -----");
				if (!moveArm(goal_pose, group_arm_torso))
				{
					ROS_INFO("Failed to move arm");
					return false;
				}
				
                /* Compute grasp pose */
				geometry_msgs::Pose grasp_pose = goal_pose.pose;
				grasp_pose.position.z = goal->pose.position.z + 0.22;
				
                /* compute height pick */
				result_.height_pick = grasp_pose.position.z - table_height;
				
				
				//remove collsion object for the target
                std::vector<std::string> object_ids; 
                object_ids.push_back(std::to_string(goal->ID));
                scene.removeCollisionObjects(object_ids);
				
				// grasp pose 
                ROS_INFO("----- GRASP POSE -----");
				if (!performGroupPlanLinearMotion(group_arm_torso, goal_pose.pose, grasp_pose)) // approach_pose, grasp_pose
				{
					ROS_INFO("Failed to reach grasping pose");
					return false;
				}
				
                /* Close Gripper */
				ROS_INFO("----- CLOSING GRIPPER -----");
				moveGripper(0.0);
				
				/* attach virtually the object */
                ROS_INFO("----- ATTACH -----");
				if (!attachObject(0, goal->ID))
				{
					ROS_INFO("Failed to virtually attach object");
					return false;
				}
				
				/* go back to approach pose	*/
                ROS_INFO("----- BACK TO APPROACH POS -----");
				if (!performGroupPlanLinearMotion(group_arm_torso, grasp_pose, goal_pose.pose)) // grasp_pose, approach_pose
				{
					ROS_INFO("Failed to reach approach pose");
					return false;
				}
				
                /* Go back to pre-movement pose */
                ROS_INFO("----- BACK TO PRE-MOVEMENT POSITION -----");
				if (!moveArm(a, group_arm_torso))
				{
					ROS_INFO("Failed to move arm");
					return false;
				}
				
				/* head back up ready to detect cylinder tables */
                if (lookAt(0.15, -0.3)) return true;
                else return false;
               
            }
            else //Placing
            {
                // clear collision objects and add cylindrical table one
                goal_pose.header.frame_id = "base_footprint";
                goal_pose.pose.position.x = goal->pose.position.x + x_offset;
                goal_pose.pose.position.y = goal->pose.position.y + y_offset;
                goal_pose.pose.position.z = cylinder_height + goal->height_place + 0.05;
                goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, 0.037);
                ROS_INFO("Place Pose from goal (base_footprint): %f, %f, %f", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);	
                
                ROS_INFO("----- ADD CYLINDETR TABLE COLLISION OBJECT -----");
                addPlaceCollisionObjects(goal, scene, group_arm_torso, goal->cylinder, goal->pose); // coordinates must be in map frame
                				
				/* grasp pose */
				geometry_msgs::Pose grasp_pose = goal_pose.pose;
				grasp_pose.position.z = goal_pose.pose.position.z -0.03; 

				ROS_INFO("Cylinder Height: %f", cylinder_height);

                /* approach pose */		
                ROS_INFO("----- APPROACH POSE -----");	
				if (!moveArm(goal_pose, group_arm_torso))
				{
					ROS_INFO("Failed to move arm");
					return false;
				}
				
				 /* grasp pose */
                 ROS_INFO("----- GRASP POSE -----");
				if (!performGroupPlanLinearMotion(group_arm_torso, goal_pose.pose, grasp_pose)) // approach_pose, grasp_pose
				{
					ROS_INFO("Failed to reach grasping pose");
					return false;
				}
				
				/* open gripper */
                ROS_INFO("----- OPEN GRIPPER -----");
				moveGripper(0.05);
				
				
				/* detach virtually the object */
                ROS_INFO("----- DETACH -----");
				if (!attachObject(1, goal->ID))
				{
					ROS_INFO("Failed to virtually detach object");
					return false;
				}
				
				// go back to approach pose
				ROS_INFO("----- BACK TO APPROACH POS -----"); 
				if (!performGroupPlanLinearMotion(group_arm_torso, grasp_pose, goal_pose.pose)) // grasp_pose, approach_pose
				{
					ROS_INFO("Failed to reach approach pose");
					return false;
				}
				
				/* back to pre-movement pose */
				//a little shifted on down-right so the arm is not in front of the camera for the next detection
				geometry_msgs::PoseStamped a;
				a.header.frame_id = "base_footprint";
                a.pose.position.x = 0.38;
                a.pose.position.y = -0.12;
                a.pose.position.z = 1;
                a.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, 0.037);
                ROS_INFO("Orientation: %f, %f, %f, %f", a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w);
                double rolla, pitcha, yawa;
                tf::Quaternion qa(goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
		        tf::Matrix3x3(qa).getRPY(rolla, pitcha, yawa);
		        a.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, yawa);
				
                ROS_INFO("----- BACK TO PRE-MOVEMENT POSITION -----");
                if (!moveArm(a, group_arm_torso))
				{
					ROS_INFO("Failed to move arm");
					return false;
				}
				else return true;
    
            }            

        }
        
        bool moveJoints(moveit::planning_interface::MoveGroupInterface& group_arm_torso, float joints_poses[8])
        {
        	std::map<std::string, double> target_position;
   
	     target_position["torso_lift_joint"] = joints_poses[0];
	     target_position["arm_1_joint"] = joints_poses[1];
	     target_position["arm_2_joint"] = joints_poses[2];
	     target_position["arm_3_joint"] = joints_poses[3];
	     target_position["arm_4_joint"] = joints_poses[4];
	     target_position["arm_5_joint"] = joints_poses[5];
	     target_position["arm_6_joint"] = joints_poses[6];
	     target_position["arm_7_joint"] = joints_poses[7];
	     
	     std::vector<std::string> torso_arm_joint_names;
	     group_arm_torso.setPlannerId("SBLkConfigDefault");
	     torso_arm_joint_names = group_arm_torso.getJoints();
	     
	     group_arm_torso.setStartStateToCurrentState();
             group_arm_torso.setMaxVelocityScalingFactor(1.0);
             
             for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
             if ( target_position.count(torso_arm_joint_names[i]) > 0 )
  	     {
                //ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
                group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
             }
	     
	     moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_torso;

            //set maximum time to find a plan
            group_arm_torso.setPlanningTime(5.0);
            bool success = (group_arm_torso.plan(my_plan_arm_torso)== moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if ( !success ) 
            {
                throw std::runtime_error("No plan found");
                return false;
            }

            ROS_INFO("Plan found in %f seconds", my_plan_arm_torso.planning_time_);

            // Execute the plan
            ros::Time start = ros::Time::now();

            group_arm_torso.move();

            ROS_INFO("Motion duration: %f", (ros::Time::now() - start).toSec());
 
            return true;
        }

        /* 
        ** Function that move the arm in the target position
        */
        bool moveArm(geometry_msgs::PoseStamped& goal_pose, moveit::planning_interface::MoveGroupInterface& group_arm_torso)
        {
                ROS_INFO("Arm movement");
        	ROS_INFO("Goal Pose: %f, %f, %f", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
        	
            std::vector<std::string> torso_arm_joint_names;

            //choose planner
            group_arm_torso.setPlannerId("SBLkConfigDefault");
            //group_arm_torso.setPoseReferenceFrame("/base_footprint"); //pose is expressed in the robot frame (base_link)
            group_arm_torso.setPoseTarget(goal_pose);

            group_arm_torso.setStartStateToCurrentState();
            group_arm_torso.setMaxVelocityScalingFactor(1.0);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_torso;

            //set maximum time to find a plan
            group_arm_torso.setPlanningTime(30.0);
            bool success = (group_arm_torso.plan(my_plan_arm_torso)== moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if ( !success ) 
            {
                throw std::runtime_error("No plan found");
                return false;
            }

            ROS_INFO("Plan found in %f seconds", my_plan_arm_torso.planning_time_);

            // Execute the plan
            ros::Time start = ros::Time::now();

            group_arm_torso.move();

            ROS_INFO("Motion duration: %f", (ros::Time::now() - start).toSec());
 
            return true;
        }

        /* 
        ** Function that activate the end effector to grasp/release the target object 
        */
         
        void moveGripper(float value)
        {
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("/gripper_controller/follow_joint_trajectory", true);

             client.waitForServer();

            control_msgs::FollowJointTrajectoryGoal goal;
	        std::vector<float> values = {value, value};

	        goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
            goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
	        goal.trajectory.points.resize(1);  
            goal.trajectory.points.at(0).positions.resize(2);
            goal.trajectory.points.at(0).velocities.resize(2);
            for (int i = 0; i < values.size(); ++i)
            {
                goal.trajectory.points.at(0).positions.at(i) = values.at(i);
                goal.trajectory.points.at(0).velocities.at(i) = 0.0;
            }
            goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

            client.sendGoal(goal);

            bool finished_before_timeout = client.waitForResult(ros::Duration(20.0));
            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = client.getState();
                ROS_INFO("Gripper controller completed action: %s", state.toString().c_str());
            }
            else
            {
                ROS_INFO("Gripper controller did not finish before the time out");
            }
        }
        
        
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
        
        bool attachObject(int action, const int& id)
        {
			ros::ServiceClient client;
			switch(action)
			{
				case 0:
					client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
					break;
				case 1:
					client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
					break;
			}
			gazebo_ros_link_attacher::Attach srv;
			
			switch(id)
			{
				case 1:
					srv.request.model_name_1 = "Hexagon";
					srv.request.link_name_1 = "Hexagon_link";
					break;
				case 2:
					srv.request.model_name_1 = "Triangle";
					srv.request.link_name_1 = "Triangle_link";
					break;
				case 3:
					srv.request.model_name_1 = "cube";
					srv.request.link_name_1 = "cube_link";
					break;
			}
			srv.request.model_name_2 = "tiago";
			srv.request.link_name_2 = "arm_7_link";
			if (client.call(srv))
			{
				ROS_INFO("Object successfully attached/detached to/from the gripper.");
				ros::Duration(3).sleep();
				return true;
			}
			else
			{
				ROS_INFO("Object failed to be attached/detached to/from the gripper.");
				return false;
			}
		}
		
        
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
		
		/* 
        ** Function that adds collision objects to the scene (picking)
        */
        void addPlaceCollisionObjects(const assignment_2::ArmGoalConstPtr &goal, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& group_arm_torso, 
			const geometry_msgs::Point& cylinder_point, const geometry_msgs::Pose& place_pose)
        {			
			// clear collision objects
			planning_scene_interface.removeCollisionObjects(collision_objects_ids);
			collision_objects_ids.clear();
			
			std::vector<moveit_msgs::CollisionObject> collision_objects;
			collision_objects.resize(1);
			
			// define cylindrical table
			collision_objects[0].id = "cylinder";
			collision_objects[0].header.frame_id = "base_footprint";
			
			collision_objects_ids.push_back(collision_objects[0].id);
			
			float radius_offset = 0.1;
			float height_offset = 0.01;
			
			geometry_msgs::Pose pose;
			pose.position = cylinder_point;
			pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
			
			// cylinder pose from map frame to base_footprint frame
			pose = poseToFootprint(pose);
			pose.position = place_pose.position;
			pose.position.z = (cylinder_height + height_offset)/2;
			
			// primitive
			shape_msgs::SolidPrimitive prim;
			prim.type = prim.CYLINDER;
			prim.dimensions.resize(2);
			prim.dimensions[0] = cylinder_height + height_offset;
			prim.dimensions[1] = cylinder_radius; 
			
			ROS_INFO("OBSTACLE %s Coords: (%f, %f, %f)", "Cylinder table", pose.position.x, pose.position.y, pose.position.z);
            ROS_INFO("OBSTACLE %s Size: (H: %f, R: %f)", "Cylinder table", prim.dimensions[0], prim.dimensions[1]);
			
			collision_objects[0].primitives.push_back(prim);
			collision_objects[0].primitive_poses.push_back(pose);
			collision_objects[0].operation = collision_objects[0].ADD;
			
			planning_scene_interface.addCollisionObjects(collision_objects);
		}

        /* 
        ** Function that adds collision objects to the scene (picking)
        */
        void addCollisionObjects(const assignment_2::ArmGoalConstPtr &goal, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& group_arm_torso)
        {
			// clearing
			// clear collision objects
			planning_scene_interface.removeCollisionObjects(collision_objects_ids);
			collision_objects_ids.clear();
			
            ROS_INFO("Adding Obstacles");
            
            //a little offset to add to the tag size in order to get the obstacle dimensions a little bigger 
            float dim_offset_table = 0.15; //to be tuned
            float dim_offset = 0.03;

            //collision object vector containing obstacles
            std::vector<moveit_msgs::CollisionObject> collision_objects;
            
            //Table 
            //create collision object
            moveit_msgs::CollisionObject collision_object;

            collision_object.header.frame_id = "base_footprint";
            //collision_object.header.frame_id = "/map";
            ROS_INFO("Group_arm_torso frame: %s", group_arm_torso.getPlanningFrame().c_str());
            collision_object.id = "table";
            collision_objects_ids.push_back(collision_object.id);

            //shape of the obstacle
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
                    primitive.dimensions.resize(3);
                    /* Setting dim of box, all the same --> CUBE */
                    primitive.dimensions[primitive.BOX_X] = 0.92 + dim_offset_table;
                    primitive.dimensions[primitive.BOX_Y] = 0.92 + dim_offset_table;
                    primitive.dimensions[primitive.BOX_Z] = 0.78;
                    
                    geometry_msgs::Pose obstacle_pose; 
                obstacle_pose.position.x = 7.8; 
                obstacle_pose.position.y = -2.975; 
                obstacle_pose.position.z = 0.39;
                obstacle_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
                
                obstacle_pose = poseToFootprint(obstacle_pose);
                
                ROS_INFO("OBSTACLE %s Coords: (%f, %f, %f)", "table", obstacle_pose.position.x, obstacle_pose.position.y, obstacle_pose.position.z);
                ROS_INFO("OBSTACLE %s Size: (X: %f, Y: %f, Z: %f)", "table", primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);

                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(obstacle_pose);
                collision_object.operation = collision_object.ADD;

                collision_objects.push_back(collision_object);

            //create a collision object for each obstacles detected, except for the target
            for (size_t i = 0; i < goal->objects_ID.size(); i++)
            {

                //create collision object
                moveit_msgs::CollisionObject collision_object;

                collision_object.header.frame_id = "base_footprint";
                collision_object.id = std::to_string(goal->objects_ID[i]);

                //shape of the obstacle
                shape_msgs::SolidPrimitive primitive;

                if (goal->objects_ID[i] == 2 || goal->objects_ID[i] == 3) //BOX SHAPE obstacles
                {
                    ROS_INFO("Creating BOX collision object for Tag %i", goal->objects_ID[i]);  
                    primitive.type = primitive.BOX;
                    primitive.dimensions.resize(3);
                    /* Setting dim of box, all the same --> CUBE */
                    primitive.dimensions[primitive.BOX_X] = goal->objects_size[i] + dim_offset;
                    primitive.dimensions[primitive.BOX_Y] = goal->objects_size[i] + dim_offset;
                    primitive.dimensions[primitive.BOX_Z] = goal->objects_size[i] + dim_offset;
                }
                else //CYLINDER SHAPE obstacles
                {
                    ROS_INFO("Creating CYLINDER collision object for Tag %i", goal->objects_ID[i]);
                    primitive.type = primitive.CYLINDER;
                    primitive.dimensions.resize(2);
                    /* Setting height of cylinder. */
                    primitive.dimensions[0] = goal->objects_pose[i].position.z - table_height + dim_offset; 
                    /* Setting radius of cylinder. */
                    primitive.dimensions[1] = goal->objects_size[i]/2 + dim_offset + 0.02;
                }

                geometry_msgs::Pose obstacle_pose; 
                obstacle_pose.position.x = goal->objects_pose[i].position.x; 
                obstacle_pose.position.y = goal->objects_pose[i].position.y; 
                if (goal->objects_ID[i] == 2 || goal->objects_ID[i] == 3) //BOX SHAPE obstacles
                {
                    obstacle_pose.position.z = goal->objects_pose[i].position.z - primitive.dimensions[primitive.BOX_Z]/2;
                }
                else //CYLINDER SHAPE obstacles
                {
                    obstacle_pose.position.z = goal->objects_pose[i].position.z - primitive.dimensions[0]/2;
                }
                obstacle_pose.position.z = goal->objects_pose[i].position.z;
                obstacle_pose.orientation = goal->objects_pose[i].orientation;
                
                ROS_INFO("OBSTACLE %i Coords: (%f, %f, %f)", goal->objects_ID[i], obstacle_pose.position.x, obstacle_pose.position.y, obstacle_pose.position.z);
                if (goal->objects_ID[i] == 2 || goal->objects_ID[i] == 3) //BOX SHAPE obstacles
                {
                	ROS_INFO("OBSTACLE %i Size: (X: %f, Y: %f, Z: %f)", goal->objects_ID[i], primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);
		        }
                else //CYLINDER SHAPE obstacles
                {
                	ROS_INFO("OBSTACLE %i Size: (H: %f, R: %f)", goal->objects_ID[i], primitive.dimensions[0], primitive.dimensions[1]);
                }
                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(obstacle_pose);
                collision_object.operation = collision_object.ADD;
				
				collision_objects_ids.push_back(collision_object.id);
                collision_objects.push_back(collision_object);
            }
            
            
            //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            // add obstacles to the scene
            while (!planning_scene_interface.applyCollisionObjects(collision_objects))
            {
    		ROS_WARN("Planning scene not ready, sleeping");
    		ros::Duration(1.0).sleep();
	    }
	    ROS_INFO("Collision Objects added to scene");  
        }
        
        
	
    public:
        ObjectManipulation(std::string name) : as_(nh_, name, boost::bind(&ObjectManipulation::executeCB, this, _1), false), name_(name)
        {
            as_.start();
            ROS_INFO("Node_C READY!");
        }

        ~ObjectManipulation(void){}

        void executeCB(const assignment_2::ArmGoalConstPtr &goal)
        {
            ROS_INFO("Executing ArmAction: %s.", name_.c_str());

            ros::Rate r(1);
            bool success = true;

            /* Robot is ready to start arm routine */
            feedback_.status = status_[0];
            as_.publishFeedback(feedback_);
            r.sleep();

            /* Checking if client has requested a preempt. */
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted.", name_.c_str());
                as_.setPreempted();
                success = false;
            }
            else
            {
                bool action; //picking = 0, placing = 1

                /* ------------------------- Picking action ------------------------- */
                if (goal->picking == true && goal->placing == false)
                {
                
               	    ROS_INFO("STart Picking");
                    action = 0;

                    feedback_.status = status_[1];
                    as_.publishFeedback(feedback_);

                    if (armAction(goal, action))
                    {
                        /* Robot has correctly picked up the object */
                        feedback_.status = status_[6];
                        as_.publishFeedback(feedback_);
                        r.sleep();

                        success = true;
                    } 
                    else 
                    {
                        /* Robot has failed to pick up the object */
                        feedback_.status = status_[7];
                        as_.publishFeedback(feedback_);
                        r.sleep();
                        as_.setAborted(result_);
                        return;
                    }
                }

                /* ------------------------- Placing action ------------------------- */
                else if (goal->picking == false && goal->placing == true)
                {
                    action = 1;

                    feedback_.status = status_[8];
                    as_.publishFeedback(feedback_);

                    if (armAction(goal, action))
                    {
                        /* Robot has correctly placed the object */
                        feedback_.status = status_[13];
                        as_.publishFeedback(feedback_);
                        r.sleep();

                        success = true;
                    } 
                    else 
                    {
                        /* Robot has failed to place the object */
                        feedback_.status = status_[14];
                        as_.publishFeedback(feedback_);
                        r.sleep();
                        as_.setAborted(result_);
                        return;
                    }
                }
                
                /* Error, can't perform both actions */
                else 
                {
                    ROS_INFO("Something wrong... what action?");
                    feedback_.status = "Something wrong... what action?";
                    as_.publishFeedback(feedback_);
                    success = false;
                    as_.setAborted(result_);
                }

                /* picking object completed, send results */
                if (success)
                {
                    ROS_INFO("%s: Succeeded.", name_.c_str());
                    feedback_.status = status_[6];
                    as_.setSucceeded(result_);
                }
            }
        }
};
        


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_C");

    //action object
    ObjectManipulation objManipulation("node_C");

    ros::spin();
    return 0;
}
