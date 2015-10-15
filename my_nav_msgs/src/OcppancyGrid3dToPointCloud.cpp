#include "ros/ros.h"
#include "my_nav_msgs/OccupancyGrid3d.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#define OCCUPIED	100

ros::Publisher pub;
ros::Publisher pub2;

void gridCallback(const my_nav_msgs::OccupancyGrid3d::ConstPtr& msg)
{
	sensor_msgs::PointCloud cloud;
	cloud.header = msg->header;
	int index[msg->data.size()];

	int size = 0;
	for(int i = 0; i < msg->data.size(); i++)
	{
		if(msg->data[i] == OCCUPIED)
		{
			index[size] = i;
			size++;
		}
	}
	cloud.points.resize(size);

	double resol = msg->info.resolution;
	uint32_t width = msg->info.width;
	uint32_t depth = msg->info.depth;
	uint32_t height = msg->info.height;
	if(size > 0)
	{
		for(int i = 0; i < size; i++)
		{
			int j = index[i];
			int ix = j % width;
			int iy = (j / width) % depth;
			int iz = j / width / depth;
			cloud.points[i].x = resol/2 + resol*ix + msg->info.origin.x;
			cloud.points[i].y = resol/2 + resol*iy + msg->info.origin.y;
			cloud.points[i].z = resol/2 + resol*iz + msg->info.origin.z;			
		}
	}

	pub.publish(cloud);
	sensor_msgs::PointCloud2 cloud2;
	sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
	pub2.publish(cloud2);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "OcppancyGrid3dToPointCloud");
	ros::NodeHandle n;

	pub = n.advertise<sensor_msgs::PointCloud>("OcppancyGrid3dCloud", 10);
	pub2 = n.advertise<sensor_msgs::PointCloud2>("OcppancyGrid3dCloud2", 10);
	ros::Subscriber sub = n.subscribe("OcppancyGrid3d", 10, gridCallback);

	ros::spin();
	return 0;
}
