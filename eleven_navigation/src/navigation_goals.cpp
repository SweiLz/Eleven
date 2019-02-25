#include "ros/ros.h"
#include "tf/tf.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "eleven_msgs/navigationCommand.h"
#include "geometry_msgs/Pose2D.h"

bool goto_point(eleven_msgs::navigationCommand::Request &req, eleven_msgs::navigationCommand::Response &res);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;

struct point
{
    std::string id;
    std::string name;
    geometry_msgs::Pose2D pose;
};

std::map<std::string, point> point_map;
MoveBaseClient *ac;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "eleven_navigation_node");
    ac = new MoveBaseClient("move_base", true);
    // while (!ac.waitForServer(ros::Duration(5.0)))
    // {
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }

    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    XmlRpc::XmlRpcValue points_param;
    nh_.getParam("/navigation_goals_node/point_map", points_param);

    for (int i = 0; i < points_param.size(); i++)
    {
        geometry_msgs::Pose2D pose2d;
        pose2d.x = points_param[i]["pose"][0];
        pose2d.y = points_param[i]["pose"][1];
        pose2d.theta = points_param[i]["pose"][2];
        point pt;
        pt.id = static_cast<std::string>(points_param[i]["id"]);
        pt.name = static_cast<std::string>(points_param[i]["name"]);
        pt.pose = pose2d;
        point_map[pt.id] = pt;
    }
    ros::ServiceServer nav_srv = nh.advertiseService("goto_point", goto_point);
    ROS_INFO("Ready to move");
    ros::spin();

    return 0;
}

bool goto_point(eleven_msgs::navigationCommand::Request &req, eleven_msgs::navigationCommand::Response &res)
{
    ROS_INFO_STREAM("request: go to -> " << req.instruction);
    point pt = point_map[std::to_string(req.instruction)];
    if (pt.id == "")
    {
        res.status = false;
        return false;
    }
    else
    {
        ROS_INFO_STREAM("data " << pt.name << ", " << pt.pose.x << ", " << pt.pose.y << ", " << pt.pose.theta);
        res.status = true;
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = pt.pose.x;
        goal.target_pose.pose.position.y = pt.pose.y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pt.pose.theta);

        ROS_INFO("Sending goal");
        ac->sendGoal(goal);

        // ac->sendGoalAndWait(goal);
    }
    // ros::Duration(5.0).sleep();

    return true;
}
