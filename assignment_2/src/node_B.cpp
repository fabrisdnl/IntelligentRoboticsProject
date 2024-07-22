#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2/DetectAction.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

#include "assignment_2/utils.h"

#include <math.h>
//#include "assignment_1/detectCV.h"

//AprilTag headers
#include "apriltag_ros/continuous_detector.h"
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

//tf headers
#include <tf/transform_listener.h>

class ObjectTagDetection
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<assignment_2::DetectAction> as_;

        std::string name_;

        assignment_2::DetectResult result_;
        assignment_2::DetectFeedback feedback_;

        std::string status_[12] = 
        {
            "The robot is ready to start object detection phase", 
            "The robot is in position -> moving head to look at the table"
            "The robot has started the object detection", 
            "The robot has failed to start the object detection",
            "The robot found the target object",
            "The robot didn't found the target object, but found other objects",
            "The robot did not find any object",
            "The robot has failed the object detection process", 
            "Started Camera to Robot frame tranformation",
            "Camera to Robot frame transformation completed",
            "Camera to Robot frame transformation failed",
            "The object detection is completed, sending object positions!"
        };
		
		bool updateTagsDetection(apriltag_ros::AprilTagDetectionArrayConstPtr& tag_array)
		{
			tag_array = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh_, ros::Duration(5.0));
			if (tag_array != NULL) return true;
			else return false;
		}
		
        /*
        **  Function that detects objects with a specific tag
        */
        apriltag_ros::AprilTagDetection findTargetObj(apriltag_ros::AprilTagDetectionArray::ConstPtr &tagsArray, const int &target_id)
        {
            /* find the tag with target ID*/
            apriltag_ros::AprilTagDetection detection;
			
            if (tagsArray->detections.size() == 0)
            {
                // No tags detected
                ROS_INFO("No Tags detected!");
                feedback_.status = status_[6];
            } 
            else 
            {
                for (const auto& detection : tagsArray->detections)
                {
                    if (detection.id[0] == target_id)
                    {
                        // target object found
                        ROS_INFO("Target Tag found!");
                        feedback_.status = status_[4];
                        as_.publishFeedback(feedback_);
                        return detection;
                    }

                    ROS_INFO("Target Tag not detected!");
                    feedback_.status = status_[5];
                }
            }

            as_.publishFeedback(feedback_);
            return detection;
        }

        /*
        ** Function that convert tag pose from camera to robot frame
        */
        geometry_msgs::Pose tfConverter(geometry_msgs::PoseWithCovarianceStamped& tagCameraPose)
        {
        	
            geometry_msgs::Pose tagRobotPose;
            feedback_.status = status_[8];
            as_.publishFeedback(feedback_);

            //Listener and Transform -> tranformation: camera to robot frame
            tf::TransformListener listener;
            tf::StampedTransform transform;
            listener.waitForTransform("base_footprint", "xtion_rgb_optical_frame", ros::Time(0), ros::Duration(5.0));

            try
            {
                listener.lookupTransform("base_footprint", "xtion_rgb_optical_frame", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("Exception trying to transform: %s",ex.what());

                feedback_.status = status_[10];
                as_.publishFeedback(feedback_);
                return tagRobotPose;
            }

            tf::Vector3 v(tagCameraPose.pose.pose.position.x, tagCameraPose.pose.pose.position.y, tagCameraPose.pose.pose.position.z);
            v = transform * v;
            tf::Quaternion q(tagCameraPose.pose.pose.orientation.x, tagCameraPose.pose.pose.orientation.y, tagCameraPose.pose.pose.orientation.z, tagCameraPose.pose.pose.orientation.w);
            q = transform * q;
            tagRobotPose.position.x = v.x();
            tagRobotPose.position.y = v.y();
            tagRobotPose.position.z = v.z();
            tagRobotPose.orientation.w = q.w();
            tagRobotPose.orientation.x = q.x();
            tagRobotPose.orientation.y = q.y();
            tagRobotPose.orientation.z = q.z();
            

            feedback_.status = status_[9];
            as_.publishFeedback(feedback_);
            
            ROS_INFO("Tag original pose: %f, %f, %f", tagCameraPose.pose.pose.position.x, tagCameraPose.pose.pose.position.y, tagCameraPose.pose.pose.position.z);
            ROS_INFO("Target Pose inside function post conversion: %f, %f, %f", tagRobotPose.position.x, tagRobotPose.position.y, tagRobotPose.position.z);

            return tagRobotPose;
        }

    public:
        ObjectTagDetection(std::string name) : as_(nh_, name, boost::bind(&ObjectTagDetection::executeCB, this, _1), false), name_(name)
        {
            as_.start();
        }

        ~ObjectTagDetection(void){}

        void executeCB(const assignment_2::DetectGoalConstPtr &goal)
        {
            ROS_INFO("Executing DetectAction: %s.", name_.c_str());

            ros::Rate r(1);
            bool success = true;

            /* Clearing list of previously detected objects. */
            result_.objects_ID.clear();
            result_.objects_size.clear();
            result_.objects_pose.clear();

            /* Robot is ready to start object tag detection phase */
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
                /* check if robot is in position and head is looking at table*/
                feedback_.status = status_[1];
                as_.publishFeedback(feedback_);

                /* Starting Object Tag Detection */
                feedback_.status = status_[2];
                as_.publishFeedback(feedback_);
				
				/* moving torso and head */
				moveTiagoTorso(0.35); // moving torso up
				lookAt(goal->hj1, goal->hj2); // look down
				
                /* get message from tag_detections topic */
                apriltag_ros::AprilTagDetectionArrayConstPtr tagsArray;
                while(!updateTagsDetection(tagsArray));

                //target object
                apriltag_ros::AprilTagDetection targetObject = findTargetObj(tagsArray, goal->ID);

                /* Converting from camera to robot frame */
                geometry_msgs::Pose targetObjectPose = tfConverter(targetObject.pose);
                
                ROS_INFO("Target Pose: %f, %f, %f", targetObjectPose.position.x, targetObjectPose.position.y, targetObjectPose.position.z);

                /* Target Object Information and pose */
                result_.ID =  targetObject.id[0];
                result_.size =  targetObject.size[0];
                result_.pose =  targetObjectPose;

                /* All the objects Information and poses */
                for (const auto& obj : tagsArray->detections)
                {
                    result_.objects_ID.push_back(obj.id[0]);
                    result_.objects_size.push_back(obj.size[0]);
                    geometry_msgs::PoseWithCovarianceStamped temp = obj.pose;
                    result_.objects_pose.push_back(tfConverter(temp));
                }

                if (targetObjectPose.position.x == NULL)
                {
                    success = false;
                    feedback_.status = status_[7];
                    as_.publishFeedback(feedback_);
                    as_.setAborted(result_);
                } 

                /* Detection completed, send results */
                if (success)
                {
                    ROS_INFO("%s: Succeeded.", name_.c_str());
                    feedback_.status = status_[11];
                    as_.setSucceeded(result_);
                }
            }
        }
};
        
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_B");
    ObjectTagDetection objTagDetect("node_B");
    ros::spin();
    return 0;
}
