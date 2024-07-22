#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_1/TiagoAction.h>
#include <assignment_1/detect.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <math.h>
#include "assignment_1/detectCV.h"

// OpenCV headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

std::string arg;

class TiagoAction
{
    protected:
        ros::NodeHandle nh_;
        ros::Publisher pub_cmdvel_;

        actionlib::SimpleActionServer<assignment_1::TiagoAction> as_;
        std::string name_;

        assignment_1::TiagoResult result_;
        assignment_1::TiagoFeedback feedback_;

        std::string status_[8] = {"The robot received final pose", 
            "The robot is stopped", 
            "The robot is moving",
            "The robot is in the final pose",
            "The robot has failed to reach final pose", 
            "The robot has started the detection of the obstacles", 
            "The detection is completed"};

        /*
        **  Function to check if the robot is in the initial position, 
        **  considering a certain threshold (initial position [0,0]).
        */
        bool checkInitialPosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr& current_pose, 
            float threshold)
        {
            if (abs(current_pose->pose.pose.position.x) < threshold &&
                abs(current_pose->pose.pose.position.y) < threshold)
                return true;
            else return false;
        }

        /*
        **  Simple function to check if the two distances are equal
        **  given a threshold value.
        */
        bool checkLateralDistances(float& dist1, float& dist2, float threshold)
        {
            if (abs(dist1) <= abs(dist2) + threshold && abs(dist1) >= abs(dist2) - threshold)
            {
                return true;
            }
            return false;
        }
		
		/*
		** Callback function to update laser scan.
		*/
		bool scanCb(sensor_msgs::LaserScanConstPtr& scan)
		{
			scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh_, ros::Duration(1.0));
			if (scan != NULL) return true;
			else return false;
		}
		
        /* 
        **  Function in order to get distances from the walls
        **  (frontal, left and right) in the narrow corridor moving routine.
        */
        void updateParameters(sensor_msgs::LaserScanConstPtr& scan, 
            float& right_dist, float& left_dist, float& frontal_dist)
        {
            int frontal_index = scan->ranges.size() / 2;
            int right_index = 60; // specific value for our case, computed offline (see PDF)
            int left_index = scan->ranges.size() - 1 - right_index;

            float left_angle = scan->angle_min + (scan->angle_increment * left_index);
            float right_angle = scan->angle_min + (scan->angle_increment * right_index);
            
            frontal_dist = scan->ranges.at(frontal_index);
            left_dist = scan->ranges.at(left_index) * sin(left_angle);
            right_dist = scan->ranges.at(right_index) * sin(right_angle);
        }

        /*
        **  Function implementing a control law to navigate through the narrow
        **  corridor using the laser data. The routine sends the velocity commands
        **  to the topic /cmd_vel to move the robot.
        */
        void controlledMoving()
        {
            /* Variables necessary to maintain a costant linear velocity. */
            ros::Rate loop_rate(5);
            const float velocity = 0.4;
            const float rotation = 0.05;

            sensor_msgs::LaserScanConstPtr scan;
            /* Velocity in free space broken into its linear and angular parts. */
            geometry_msgs::Twist tiago_twist;
            tiago_twist.linear.x = 0.0;
            tiago_twist.linear.y = 0.0;
            tiago_twist.angular.x = 0.0;
            tiago_twist.angular.y = 0.0;
            

            float frontal_dist, left_dist, right_dist;
            bool end = false;

            while(!end)
            {
				scan = NULL;
                while(!scanCb(scan));
				updateParameters(scan, right_dist, left_dist, frontal_dist);
                /* Waiting until the \scan topic publishes. */
                if (scan == NULL) continue;
                /* If there is an obstacle within the range specified in front of the robot. */
                if (frontal_dist < 2.0)
                {
                    tiago_twist.linear.x = 0.0;
                    tiago_twist.angular.z = 0.0;
                    feedback_.status = status_[1];
                    as_.publishFeedback(feedback_);
                    end = true;
                }
                /* If the robot has reached the end of the corridor. */
                else if (abs(left_dist) > 2.0 || abs(right_dist) > 2.0)
                {
                    tiago_twist.linear.x = 0.0;
                    tiago_twist.angular.z = 0.0;
                    feedback_.status = status_[1];
                    as_.publishFeedback(feedback_);
                    end = true;
                }
                /* The robot is still going through the narrow corridor. */
                else
                {
                    /* Checking if the robot is going straight. */
                    float threshold = 0.1;
                    if (checkLateralDistances(left_dist, right_dist, 0.1))
                    {
                        /* Continue going straight. */
                        tiago_twist.linear.x = velocity;
                        tiago_twist.angular.z = 0.0;
                        feedback_.status = status_[2];
                        as_.publishFeedback(feedback_);
                    }
                    else
                    {
                        /* Robot corrects its orientation. */
                        tiago_twist.linear.x = 0.0;
                        pub_cmdvel_.publish(tiago_twist);
                        feedback_.status = status_[1];
                        as_.publishFeedback(feedback_);
                        loop_rate.sleep();
                        
                        while (!checkLateralDistances(left_dist, right_dist, 0.05) 
                            && frontal_dist > 0.5)
                        {
                            /* If robot reaches end of corridor, stop the correction process. */
                            if (abs(left_dist) > 2.0 || abs(right_dist) > 2.0)
                            {
                                end = true;
                                break;
                            }

                            tiago_twist.linear.x = velocity;
                            /* Set rotation to left or right depending on which wall is closer. */
                            if (abs(left_dist) > abs(right_dist))
                            {
                                tiago_twist.angular.z = rotation;
                            }
                            else tiago_twist.angular.z = rotation * (-1);
                            pub_cmdvel_.publish(tiago_twist);
                            feedback_.status = status_[2];
                            as_.publishFeedback(feedback_);
                            loop_rate.sleep();

                            /* Updating params. */
							scan = NULL;
                            while(!scanCb(scan));
							updateParameters(scan, right_dist, left_dist, frontal_dist);
                        }
                        tiago_twist.linear.x = 0.0;
                        pub_cmdvel_.publish(tiago_twist);
                        feedback_.status = status_[1];
                        as_.publishFeedback(feedback_);
                        loop_rate.sleep();

                        tiago_twist.angular.z *= -1;
                        pub_cmdvel_.publish(tiago_twist);
                        loop_rate.sleep();

                        tiago_twist.angular.z = 0.0;
                    }
                }
                pub_cmdvel_.publish(tiago_twist);
                loop_rate.sleep();
            }
            /* Stopping the robot. */
            tiago_twist.linear.x = 0.0;
            tiago_twist.angular.z = 0.0;
            pub_cmdvel_.publish(tiago_twist);
            loop_rate.sleep();
            feedback_.status = status_[1];
            as_.publishFeedback(feedback_);
        }


        /*
        **  Autonomous moving function to reach final pose, calling the
        **  move_base stack.
        */
        bool autonomousMoving(const move_base_msgs::MoveBaseGoal& final_pose)
        {
			ros::Rate rate(1);
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_ac("move_base", true);
            /* Waiting action server to be ready. */
            while (!mb_ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for move_base action server to be ready.");
            }
			feedback_.status = status_[2];
            as_.publishFeedback(feedback_);
			rate.sleep();
            mb_ac.sendGoal(final_pose);
            /* Waiting until robot reaches final pose. */
            mb_ac.waitForResult();
            if (mb_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("The robot has arrived to the pose specified.");
                return true;
            }
            else 
            {
                ROS_INFO("The robot has failed to arrive to the pose specified.");
                return false;
            }
        }
        
    public:
        TiagoAction(std::string name) : 
            as_(nh_, name, boost::bind(&TiagoAction::executeCb, this, _1), false), 
            name_(name)
        {
            as_.start();
            pub_cmdvel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);
        }

        ~TiagoAction(void){}

        void executeCb(const assignment_1::TiagoGoalConstPtr &goal)
        {
            ROS_INFO("Executing TiagoAction: %s.", name_.c_str());

            ros::Rate r(1);
            bool success = true;
            /* Clearing list of previously detected obstacles. */
            result_.obstacles.clear();
            
            /* Server has received final pose. */
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
                /* If robot is in the initial position, start controlled navigation. */
                geometry_msgs::PoseWithCovarianceStampedConstPtr current_pose;
                current_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", nh_);
                if (checkInitialPosition(current_pose, 0.1))
                {
                    ROS_INFO("The robot is in the initial position.");
                    ROS_INFO("Controlled navigation in order to reach end of narrow corridor.");
                    controlledMoving();
                    ROS_INFO("Stopped controlled navigation.");
                }
                else {
                    ROS_INFO("The robot is not in the initial position.");
                }

                /* Starting autonomous navigation using move_base stack. */
                move_base_msgs::MoveBaseGoal gpose;
                gpose.target_pose.header.frame_id = "map";
                gpose.target_pose.header.stamp = ros::Time::now();
                gpose.target_pose.pose.position = goal->final_pose.position;
                gpose.target_pose.pose.orientation = goal->final_pose.orientation;

                ROS_INFO("Autonomous navigation in order to reach the final pose.");
                if (autonomousMoving(gpose))
                {
                    /* Robot has reached final pose. */
                    feedback_.status = status_[3];
                    as_.publishFeedback(feedback_);
                    r.sleep();
                }
                else
                {
                    /* Robot has failed to reach final pose. */
                    feedback_.status = status_[4];
                    as_.publishFeedback(feedback_);
                    r.sleep();
                    as_.setAborted(result_);
                    return;
                }

            }
            /* Waiting for a bit of seconds to be sure the laser scan is not done while the robot moving. */
			ros::Duration(3).sleep();
            /* Detection */
			ROS_INFO("The robot has started the detection of the obstacles.");
			feedback_.status = status_[5];
			as_.publishFeedback(feedback_);
			r.sleep();
			
			sensor_msgs::LaserScanConstPtr scan = NULL;
			while(!scanCb(scan));
			//detection(scan);

            std::vector<geometry_msgs::Point> positions;

            //select obstacle detection algorithm chosen in command line
            if (arg == "detection")
            {
                Detect detect;
                positions = detect.detection(scan, success);
            } 
            else 
            {
                positions = detectionCV(scan, success);
            }
                
            if (!success)
            {
                ROS_ERROR("Error performing detection.");
            }
            else
            {
                for (geometry_msgs::Point p : positions)
                {
                    result_.obstacles.push_back(p);
                }
                feedback_.status = status_[6];
                as_.publishFeedback(feedback_);
                ROS_INFO("Detected %d obstacles.", (int) result_.obstacles.size());
                success = true;
            }
			
			
            if (success)
            {
                ROS_INFO("%s: Succeeded.", name_.c_str());
                as_.setSucceeded(result_);
            }
        }
        
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server_tiago");

    /* Check command line arguments. */
    if (argc != 2)
    {
        ROS_INFO("Select an algorithm for the obstacle detection: detection or detectionCV");
        ROS_INFO("usage: server_tiago detection");
        ROS_INFO("or");
        ROS_INFO("usage: server_tiago detectionCV");
        return -1;
    }

    // Check if command line arguments are valid
    else
    {
        if(std::string(argv[1]) != "detection" && std::string(argv[1]) != "detectionCV") 
        {
            ROS_INFO("Invalid argument: must be \"detection\" or \"detectionCV\"");
            return -1;
        } 
        else 
        {
            arg = std::string(argv[1]);
        }
    }

    TiagoAction ta("server_tiago");
    ros::spin();
    return 0;
}
