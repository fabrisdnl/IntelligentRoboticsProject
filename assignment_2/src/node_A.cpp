#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tiago_iaslab_simulation/objs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <assignment_1/TiagoAction.h>
#include <assignment_2/ArmAction.h>
#include <assignment_2/DetectAction.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <algorithm>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "assignment_2/utils.h"

class TiagoHandler
{
		protected:
			actionlib::SimpleActionClient<assignment_1::TiagoAction> ac_tiago_;
			actionlib::SimpleActionClient<assignment_2::DetectAction> ac_detect_;
			actionlib::SimpleActionClient<assignment_2::ArmAction> ac_arm_;
			
			ros::NodeHandle nh_;
			image_transport::ImageTransport image_transport_;
			cv::Mat img_;
			
			/* Simple function to create a position (geometry_msgs::Point) and then set its values. */
			geometry_msgs::Point setPosition(const float& x, const float& y, const float& z)
			{
				geometry_msgs::Point position;
				position.x = x; 
				position.y = y; 
				position.z = z;
				return position;
			}

			/* Function to set all the poses, and head joints for picking poses. */
			void setGlobalParameters(geometry_msgs::Pose& waypoint, geometry_msgs::Pose& waypoint_postpick, geometry_msgs::Pose& left_passage, geometry_msgs::Pose& room, geometry_msgs::Pose& waypoint_postplace, geometry_msgs::Pose& blue_pose, geometry_msgs::Pose& green_pose, geometry_msgs::Pose& red_pose,
				float& red_hj1, float& red_hj2, float& blue_hj1, float& blue_hj2, float& green_hj1, float& green_hj2)
			{
				waypoint.position = setPosition(8.5, 0, 0);
				waypoint.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);

				waypoint_postpick.position = setPosition(8.9, -2.0, 0);
				waypoint_postpick.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
				
				left_passage.position = setPosition(8.9, -4.0, 0);
				left_passage.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI);
				
				room.position = setPosition(11.5, -2.0, 0);
				room.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI/2.0);

				waypoint_postplace.position = setPosition(11.0, -4.0, 0);
				waypoint_postplace.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI);
				
				red_pose.position = setPosition(7.5, -2.1, 0);
				red_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
				red_hj1 = 0.0;
				red_hj2 = -0.75; // head down
				
				blue_pose.position = setPosition(8.0, -2.1, 0);
				blue_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0-0.1);
				blue_hj1 = 0.0;
				blue_hj2 = -0.70;
				
				green_pose.position = setPosition(7.65, -3.9, 0);
				green_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI/2.0-0.1);
				green_hj1 = 0.0;
				green_hj2 = -0.70;
			}

			/* Function to call the action server and navigate to a given position. */
			bool movingRoutine(const assignment_1::TiagoGoal& goal, assignment_1::TiagoResultConstPtr& result_ptr)
			{
				ROS_INFO("Sending goal to the server_tiago.");
				ac_tiago_.sendGoal(goal);
				
				bool completed = ac_tiago_.waitForResult(ros::Duration(240));
				if (completed)
				{
					result_ptr = ac_tiago_.getResult();
					return true;
				}
				else
				{
					ROS_INFO("server_tiago action is not completed before timeout.");
					return false;
				}
			}

			void detectMarkerFeedbackCb(const assignment_2::DetectFeedbackConstPtr&){}
			void detectMarkerActiveCb(){}
			void detectMarkerDoneCb(const actionlib::SimpleClientGoalState& state, const assignment_2::DetectResultConstPtr& detect_result)
			{
				if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_INFO("Detected %i markers.", (int) detect_result->objects_ID.size() + 1);
				}
			}

			assignment_2::DetectResultConstPtr detectMarker(const assignment_2::DetectGoal& detect_goal)
			{
				ROS_INFO("Sending goal to the node_B.");
				ac_detect_.sendGoal(detect_goal, boost::bind(&TiagoHandler::detectMarkerDoneCb, this, _1, _2),
					boost::bind(&TiagoHandler::detectMarkerActiveCb, this), 
					boost::bind(&TiagoHandler::detectMarkerFeedbackCb, this, _1));
				
				bool completed = ac_detect_.waitForResult(ros::Duration(240));
				if (completed)
				{
					return ac_detect_.getResult();
				}
				else
				{
					ROS_INFO("node_B action is not completed before timeout.");
					return NULL;
				}
			}
			
			/* Function to set the pick goal action parameters. */
			void setPickingParameters(assignment_2::ArmGoal& pick_goal, assignment_2::DetectResultConstPtr& detect_ptr)
			{
				pick_goal.picking = true;
				pick_goal.placing = false;
				pick_goal.ID = detect_ptr->ID;
				pick_goal.size = detect_ptr->size;
				pick_goal.pose = detect_ptr->pose;
				pick_goal.objects_ID = detect_ptr->objects_ID;
				pick_goal.objects_size = detect_ptr->objects_size;
				pick_goal.objects_pose = detect_ptr->objects_pose;
			}

			/* Function to set the place goal action parameters. */
			void setPlacingParameters(assignment_2::ArmGoal& place_goal, assignment_2::DetectResultConstPtr& detect_ptr)
			{
				place_goal.picking = false;
				place_goal.placing = true;
				place_goal.ID = detect_ptr->ID;
				place_goal.size = detect_ptr->size;
				place_goal.pose = detect_ptr->pose;
			}

			/* Function to call the node C in order to pick the object. */
			bool pickSequence(assignment_2::ArmGoal& pick_goal, assignment_2::ArmGoal& place_goal)
			{
				ROS_INFO("Sending goal to the node_C.");
				ac_arm_.sendGoal(pick_goal);
				ROS_INFO("Waiting for the result.");
				ac_arm_.waitForResult();
				
				if (ac_arm_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					if (pick_goal.picking == true && pick_goal.placing == false)
					{
						assignment_2::ArmResultConstPtr pick_result = ac_arm_.getResult();
						/* Saving the height of the grip w.r.t. the table's surface. */
						place_goal.height_place = pick_result->height_pick;
					}
					return true;
				}
				else
				{
					ROS_INFO("Error in node_C while picking up the object.");
					return false;
				}
			}

			/* Function to call the node C in order to place the object. */
			bool placeSequence(assignment_2::ArmGoal& place_goal)
			{
				ROS_INFO("Sending goal to the node_C.");
				ac_arm_.sendGoal(place_goal);
				ROS_INFO("Waiting for the result.");
				ac_arm_.waitForResult();
				
				if (ac_arm_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					return true;
				}
				else
				{
					ROS_INFO("Error in node_C while picking up the object.");
					return false;
				}
			}

			void imageCb(const sensor_msgs::ImageConstPtr& msg)
			{
				cv_bridge::CvImagePtr cv_ptr;
				try
				{
					cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				}
				catch (cv_bridge::Exception& e)
				{
					ROS_ERROR("cv_bridge exception: %s", e.what());
					return;
				}
				cv_ptr->image.copyTo(img_);
			}

			/* Function to analyze the image and compute the position from right to left (0, 1 or 2) of
			*  cylindrical table of color indicated by the id. */
			void getTablesOrder(cv::Mat& img, int& id, int& pos)
			{
				cv::Mat hsv_img;
				cv::cvtColor(img, hsv_img, CV_BGR2HSV);
				
				cv::Mat red_mask, green_mask, blue_mask;
				cv::Mat mask1, mask2, locations;
				/* For each color, it is applied a mask to the image, in order to find
				*  the rightmost pixel (column value) of the relative color of the mask in the image.  */
				cv::inRange(hsv_img, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
				cv::inRange(hsv_img, cv::Scalar(160, 70, 50), cv::Scalar(180, 255, 255), mask2);
				red_mask = mask1 | mask2;
				cv::findNonZero(red_mask, locations);
				int total = locations.total();
				int red_col = locations.at<cv::Point>(total-1).x;	// rightmost point
				
				cv::inRange(hsv_img, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), blue_mask);
				cv::findNonZero(blue_mask, locations);
				total = locations.total();
				int blue_col = locations.at<cv::Point>(total-1).x;	// rightmost point
				
				cv::inRange(hsv_img, cv::Scalar(45, 20, 20), cv::Scalar(75, 255, 255), green_mask);
				cv::findNonZero(green_mask, locations);
				total = locations.total();
				int green_col = locations.at<cv::Point>(total-1).x;	// rightmost point
				/* Then we sort the positions, to keep the pixel order by righmost appearance to leftmost. */
				int order[3] = {blue_col, green_col, red_col};
				std::sort(std::begin(order), std::end(order), std::greater<int>());
				/* For each target id we get the position of the column value corresponding to the color, which
				   correspond to the position of the cylindrical table coordinates ordered from right to left too.  */
				switch(id)
				{
					// blue
					case 1:
						pos = std::distance(order, std::find(order, order + 3, blue_col));
						break;
					// green
					case 2:
						pos = std::distance(order, std::find(order, order + 3, green_col));
						break;
					// red
					case 3:
						pos = std::distance(order, std::find(order, order + 3, red_col));
						break;
				}
			}
			
			/* Function that return the coordinates of the right cylindrical table associated with the color of target id. */
			geometry_msgs::Point detectCylindricalTable(assignment_1::TiagoResultConstPtr tiago_result_ptr, int id)
			{
				geometry_msgs::Point pos;
				if (tiago_result_ptr->obstacles.size() != 3)
				{
					ROS_INFO("Missing cylindrical tables. wrong detection pose!");
					pos.z = -1.0;
					return pos;
				}

				//get image
				cv::Mat img;
				sensor_msgs::ImageConstPtr msg_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/rgb/image_raw", nh_, ros::Duration(5.0));
				imageCb(msg_ptr);

				int position;
				getTablesOrder(img_, id, position);
				return tiago_result_ptr->obstacles.at(position);
			}
			
		public:
			TiagoHandler(ros::NodeHandle nh) : nh_(nh), image_transport_(nh), ac_tiago_("server_tiago", true), ac_detect_("node_B", true), ac_arm_("node_C", true)
			{
				ROS_INFO("Waiting for server_tiago, node_B and node_C to start.");
				ac_tiago_.waitForServer();
				ROS_INFO("tiago_server READY!");
				ac_detect_.waitForServer();
				ROS_INFO("node_B READY!");
				ac_arm_.waitForServer();
				ROS_INFO("node_C READY!");
			}
			
			void executeProgram()
			{
				actionlib_msgs::GoalStatusArrayConstPtr status_ptr;
				bool ready = false;
				do
				{
					ros::Duration(1).sleep();
					status_ptr = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/play_motion/status", nh_);
					for (uint i = 0; i < status_ptr->status_list.size(); ++i)
					{
						if (status_ptr->status_list[i].status == status_ptr->status_list[i].SUCCEEDED) {
							ready = true;
						}
					}
				} while(!ready);
				/* ServiceClient for interacting with the human_node service. */
				ros::ServiceClient client = nh_.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
				/* human_objects_srv */
				tiago_iaslab_simulation::Objs srv;
				srv.request.ready = true;		// human_node sends the object's ID
				srv.request.all_objs = true;	// extra point parts
				/* vector to store the order of object's indexes */
				std::vector<int> ids;
				
				/* Call the human_node service and wait for the response. */
				if (client.call(srv))
				{
					int ids_number = (int)srv.response.ids.size();
					ROS_INFO("ids number: %i", ids_number);
					for (int i = 0; i < ids_number; ++i)
					{
						int id = srv.response.ids.at(i);
						ids.push_back(id);
						ROS_INFO("object's id in index %i is: %i", i+1, (int) id);
					}
				}
				else
				{
					ROS_ERROR("Failed to call service human_objects_srv.");
					ros::shutdown();
				}
				
				assignment_1::TiagoResultConstPtr tiago_result_ptr;
				geometry_msgs::Pose waypoint, left_passage, room, waypoint_postpick, waypoint_postplace;
				
				/* Configure global poses in front of the table. */
				geometry_msgs::Pose blue_pose, green_pose, red_pose;
				float red_hj1, red_hj2, blue_hj1, blue_hj2, green_hj1, green_hj2;
				
				setGlobalParameters(waypoint, waypoint_postpick, left_passage, room, waypoint_postplace, blue_pose, green_pose, red_pose,
					red_hj1, red_hj2, blue_hj1, blue_hj2, green_hj1, green_hj2);
					
				/* Goals setting. */
				assignment_1::TiagoGoal obj_goal;
				assignment_1::TiagoGoal waypoint_goal;
				assignment_1::TiagoGoal waypoint_postpick_goal;
				assignment_1::TiagoGoal left_passage_goal;
				assignment_1::TiagoGoal table_goal;
				assignment_1::TiagoGoal room_goal;
				assignment_1::TiagoGoal waypoint_postplace_goal;
				assignment_1::TiagoGoal back_goal;
				assignment_2::DetectGoal detect_goal;
				
				/* Moving to waypoint in first room to avoid losing time 
				* (because of ROS Stack Navigation) in the first obstacle. */
				ROS_INFO("Moving to intermediate point in order to avoid obstacle at the end of corridor.");
				waypoint.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
				waypoint_goal.final_pose = waypoint;
				if (!movingRoutine(waypoint_goal, tiago_result_ptr))
				{
					ROS_INFO("Failed to reach waypoint pose.");
					ros::shutdown();
				}
				ROS_INFO("Reached waypoint.");

				/* for each object to pick */
				for (int i = 0; i < ids.size(); ++i)
				{
					switch(ids.at(i))
					{
						// blue hexagon
						case 1:
							ROS_INFO("Object of ID %i is the blue hexagon.", ids.at(i));
							obj_goal.final_pose = blue_pose;
							detect_goal.hj1 = blue_hj1;
							detect_goal.hj2 = blue_hj2;
							break;
						// green triangle
						case 2:
							ROS_INFO("Object of ID %i is the green triangle.", ids.at(i));
							obj_goal.final_pose = green_pose;
							detect_goal.hj1 = green_hj1;
							detect_goal.hj2 = green_hj2;
							/* In case of the green object we create another waypoint in order to let Tiago
							 * go through the passage in the left, because ROS Navigation Stack has some problem
							 * with the table surface dimensions. */
							ROS_INFO("Moving on the left of the table.");
							left_passage.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI);
							left_passage_goal.final_pose = left_passage;
							if (!movingRoutine(left_passage_goal, tiago_result_ptr))
							{
								ROS_INFO("Failed to reach left of table pose.");
								ros::shutdown();
							}
							ROS_INFO("Reached left of table.");
							break;
						// red cube
						case 3:
							ROS_INFO("Object of ID %i is the red cube.", ids.at(i));
							obj_goal.final_pose = red_pose;
							detect_goal.hj1 = red_hj1;
							detect_goal.hj2 = red_hj2;
							break;
					}
										
					/* Moving to position near the table to pick the object */
					if (!movingRoutine(obj_goal, tiago_result_ptr))
					{
						ROS_INFO("Failed to reach global pose in front of object %i.", ids.at(i));
						ros::shutdown();
					}
					ROS_INFO("Reached object of ID %i.", ids.at(i));
					
					/* Detection (node_B). */
					detect_goal.ID = ids.at(i);
					assignment_2::DetectResultConstPtr detect_ptr = detectMarker(detect_goal);
					if (detect_ptr == NULL)
					{
						ROS_INFO("Detection of object of ID %i failed.", ids.at(i));
						ros::shutdown();
					}
					ROS_INFO("Detection of object of ID %i completed.", ids.at(i));
					
					assignment_2::ArmGoal pick_goal;
					assignment_2::ArmGoal place_goal;
					setPickingParameters(pick_goal, detect_ptr);
					setPlacingParameters(place_goal, detect_ptr);
					/* Pick-up sequence. */
					ROS_INFO("Start sequence to pick up object of ID %i.", ids.at(i));
					if (!pickSequence(pick_goal, place_goal))
					{
						ROS_INFO("Picking sequence of object of ID %i failed!", ids.at(i));
						ros::shutdown();
					}
					ROS_INFO("Object of ID %i picked up successfully!", ids.at(i));

					switch(ids.at(i))
					{
						// blue hexagon
						case 1:
							back_goal.final_pose = blue_pose;
							back_goal.final_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI/2.0);
							break;
						// green triangle
						case 2:
							back_goal.final_pose = green_pose;
							back_goal.final_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
							break;
						// red cube
						case 3:
							back_goal.final_pose = red_pose;
							back_goal.final_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI/2.0);
							break;
					}
					if (!movingRoutine(back_goal, tiago_result_ptr))
					{
						ROS_INFO("Failed to rotate back.");
						ros::shutdown();
					}
					ROS_INFO("Rotated back.");

					/* Reaching an other intermediate point */
					if (ids.at(i) == 2)
					{
						left_passage.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
						left_passage_goal.final_pose = left_passage;
						if (!movingRoutine(left_passage_goal, tiago_result_ptr))
						{
							ROS_INFO("Failed to reach left of table pose.");
							ros::shutdown();
						}
						ROS_INFO("Reached left of table.");
					}
					else
					{
						/* First waypoint */
						waypoint_postpick.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
						waypoint_postpick_goal.final_pose = waypoint_postpick;
						if (!movingRoutine(waypoint_postpick_goal, tiago_result_ptr))
						{
							ROS_INFO("Failed to reach waypoint pose.");
							ros::shutdown();
						}
						ROS_INFO("Reached waypoint.");
						/* Second waypoint */
						left_passage.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
						left_passage_goal.final_pose = left_passage;
						if (!movingRoutine(left_passage_goal, tiago_result_ptr))
						{
							ROS_INFO("Failed to reach left of table pose.");
							ros::shutdown();
						}
						ROS_INFO("Reached left of table.");
					}
					
					/* Moving to position in order to detect cylidrical tables. */
					ROS_INFO("Moving to intermediate point in order to detect cylindrical tables.");
					room_goal.final_pose = room;
					if (!movingRoutine(room_goal, tiago_result_ptr))
					{
						ROS_INFO("Failed to reach room pose in order to detect cylindrical tables.");
						ros::shutdown();
					}
					ROS_INFO("Reached position for detection.");
					
					/* Detecting right cylindrical table. */
					geometry_msgs::Point cylindrical_coords = detectCylindricalTable(tiago_result_ptr, ids.at(i));
					/* Moving Tiago in front of the cylindrical table. */
					float threshold = 0.7;
					table_goal.final_pose.position.x = cylindrical_coords.x; 
					table_goal.final_pose.position.y = cylindrical_coords.y;
					table_goal.final_pose.position.z = 0.0;
					table_goal.final_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
					table_goal.final_pose = poseToMap(table_goal.final_pose);
					table_goal.final_pose.position.y -= threshold;
					if (!movingRoutine(table_goal, tiago_result_ptr))
					{
						ROS_INFO("Failed to reach position in front of cylindrical table relative to object of ID %i.", ids.at(i));
						ros::shutdown();
					}
					ROS_INFO("Reached position in front of cylindrical table relative to object of ID %i.", ids.at(i));
					
					/* Placing the object to the relative cylindrical table. */
					ROS_INFO("Start sequence to place the object of ID %i on the respective cylindrical table.", ids.at(i));
					place_goal.cylinder = cylindrical_coords;
					place_goal.pose.position.x = table_goal.final_pose.position.x;
					place_goal.pose.position.y = table_goal.final_pose.position.y + threshold; // threshold subtracted before
					place_goal.pose.position.z = 0;
					place_goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0); // default, not relevant
					place_goal.pose = poseToFootprint(place_goal.pose);
					//place_goal.pose.position.y += threshold;
					if (!placeSequence(place_goal))
					{
						ROS_INFO("Placing sequence of object of ID %i failed!", ids.at(i));
						ros::shutdown();
					}
					ROS_INFO("Object of ID %i placed successfully!", ids.at(i));

					if (i < 2)
					{
						table_goal.final_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
						if (!movingRoutine(table_goal, tiago_result_ptr))
						{
							ROS_INFO("Failed to rotate back.");
							ros::shutdown();
						}
						ROS_INFO("Rotated back.");
						waypoint_postplace.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI);
						waypoint_postplace_goal.final_pose = waypoint_postplace;
						if (!movingRoutine(waypoint_postplace_goal, tiago_result_ptr))
						{
							ROS_INFO("Failed to reach waypoint.");
							ros::shutdown();
						}
						ROS_INFO("Reached waypoint.");
						/* If the id in the next iteration is not of green target. */
						if (ids.at(i+1) != 2)
						{
							/* First waypoint */
							waypoint_postpick.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI);
							waypoint_postpick_goal.final_pose = waypoint_postpick;
							if (!movingRoutine(waypoint_postpick_goal, tiago_result_ptr))
							{
								ROS_INFO("Failed to reach waypoint pose.");
								ros::shutdown();
							}
							ROS_INFO("Reached waypoint.");
						}
					}
				}
				ROS_INFO("All objects have been placed successfully!");
			}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_A");
	ros::NodeHandle nh;
	TiagoHandler handler(nh);
	handler.executeProgram();
	ros::spin();
	return 0;
}
