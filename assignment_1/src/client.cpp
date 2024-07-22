#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_1/TiagoAction.h>

class TiagoActionClient
{
    protected:
        actionlib::SimpleActionClient<assignment_1::TiagoAction> ac_;

    public:
        TiagoActionClient() : ac_("server_tiago", true)
        {
            ROS_INFO("Waiting for action server to start.");
            ac_.waitForServer();
        }

        void sendTiagoGoal(assignment_1::TiagoGoal goal)
        {
            ROS_INFO("Action server started, sending goal.");
            ac_.sendGoal(goal, boost::bind(&TiagoActionClient::doneCb, this, _1, _2),
				boost::bind(&TiagoActionClient::activeCb, this),
                boost::bind(&TiagoActionClient::feedbackCb, this, _1));
        }

        void doneCb(const actionlib::SimpleClientGoalState& state,
            const assignment_1::TiagoResultConstPtr& result)
        {
            ROS_INFO("Finished in state [%s]", state.toString().c_str());
            int obstacles_number = result->obstacles.size();
            ROS_INFO("Number of obstacles detected: %d.", obstacles_number);
            for (int i = 0; i < obstacles_number; ++i)
            {
                ROS_INFO("Medium point of obstacle [%d] is in coordinates: x = [%lf], y = [%lf].",
                    i+1, result->obstacles[i].x, result->obstacles[i].y);

            }
            ros::shutdown();
        }

	void activeCb()
        {
            ROS_INFO("Goal is active.");
        }

        void feedbackCb(const assignment_1::TiagoFeedbackConstPtr& feedback)
        {
            ROS_INFO("Tiago's current status: [%s].", feedback->status.c_str());
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_tiago");
    /* Check command line arguments. */
    if (argc != 8)
    {
        ROS_INFO("In order to have a pose as input, \nwe need position's parameters (xp, yp, zp) and \nquaternion's parameters (xq, yq, zq, wq) \nwhich are all std_msgs::Float64 (double in cpp).");
        ROS_INFO("usage: client_tiago xp yp zp xq yq zq wq");
        ROS_INFO("Some coordinates could be unreachable by the robot: \nthey could be outside the map or collide with an object.");
        ROS_INFO("A working pose example: client_tiago 11 1.2 0.0 0.0 0.0 -0.3 0.5");
        return -1;
    }

    TiagoActionClient client;
    /* Read parameters and call function to send goal. */
    assignment_1::TiagoGoal goal;
    goal.final_pose.position.x = atof(argv[1]);
    goal.final_pose.position.y = atof(argv[2]);
    goal.final_pose.position.z = atof(argv[3]);
    goal.final_pose.orientation.x = atof(argv[4]);
    goal.final_pose.orientation.y = atof(argv[5]);
    goal.final_pose.orientation.z = atof(argv[6]);
    goal.final_pose.orientation.w = atof(argv[7]);

    client.sendTiagoGoal(goal);

    ros::spin();
    return 0;
}
