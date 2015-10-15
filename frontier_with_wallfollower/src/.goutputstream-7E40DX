#include "ros/ros.h"
#include "std_srvs/Empty.h"

int flag = 0;

bool launch_movebase(std_srvs::Empty::Request  &req,
		     		 std_srvs::Empty::Response &res)
{
	flag = 1;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "launch_move_base");
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("launch_move_base", launch_movebase);
	
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		if(flag)
		{
			flag = 0;
			ROS_INFO("Launch move_base");
			system("roslaunch rbx1_nav fake_move_base.launch");
			//system("roslaunch beginner_tutorials launch_movebase.launch");
			ROS_INFO("move_base is killed");
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
