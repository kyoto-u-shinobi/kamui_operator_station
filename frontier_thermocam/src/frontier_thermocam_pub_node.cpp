#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>

class FrontierPub
{
public:
	FrontierPub()
	{
		ros::NodeHandle n;
		sub_map = n.subscribe("frontier_goal", 1000, &FrontierPub::goalCallback, this);
		pub_goal = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
	}

	void goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
	{
		goal_pose.header.frame_id = "map";
		goal_pose.header.stamp = ros::Time::now();
		goal_pose.pose = *msg;

		ROS_INFO("Sending goal x:%f  y:%f  w:%f", goal_pose.pose.position.x, goal_pose.pose.position.y);

		if(goal_pose.pose.orientation.x == 0 && goal_pose.pose.orientation.y == 0 \
		   && goal_pose.pose.orientation.z == 0 && goal_pose.pose.orientation.w == 0)
		{
			goal_pose.pose.orientation.w = 1.0;
		}
		pub_goal.publish(goal_pose);
	}

private:
	ros::Subscriber sub_map;
	ros::Publisher pub_goal;
	geometry_msgs::PoseStamped goal_pose;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "frontier_thermocam_pub_node");

	FrontierPub frontier;

	ros::spin();
}
