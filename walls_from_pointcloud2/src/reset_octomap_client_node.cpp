#include "ros/ros.h"
#include "std_srvs/Empty.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reset_octomap_client_node");
	ros::NodeHandle n;

	ros::ServiceClient reset_map_client = n.serviceClient<std_srvs::Empty>("/octomap_server_node/reset");

	ros::Rate loop_rate(0.5);

	while(ros::ok())
	{
		std_srvs::Empty srv;
		reset_map_client.call(srv);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
