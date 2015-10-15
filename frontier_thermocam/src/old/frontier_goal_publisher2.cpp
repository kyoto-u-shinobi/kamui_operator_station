#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <math.h>
#include "nav_msgs/GridCells.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose goal_pose;

void goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	goal_pose = *msg;
	ROS_INFO("goalCallback");//debug
}

int main(int argc, char** argv){
	ros::init(argc, argv, "frontier_goal_publisher");

	ros::NodeHandle n;
	ros::Subscriber sub_map = n.subscribe("frontier_goal", 1000, goalCallback);

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	ros::Rate loop_rate(1);

	bool start_flag = false;
	ros::param::set("nav_start", false);
/*
	while(!start_flag)
	{
		ros::param::get("nav_start", start_flag);
	}
*/
	while(ros::ok())
	{
		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
	
		move_base_msgs::MoveBaseGoal goal;
	
		//we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		ros::spinOnce();
	
		goal.target_pose.pose = goal_pose;
	
		ROS_INFO("Sending goal");
		ac.sendGoal(goal);
	
		ac.waitForResult();
	
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Goal!");
		}
		else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
		{
			ROS_INFO("Aborted...");
		}
		else
		{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

		loop_rate.sleep();
	}
	return 0;
}
