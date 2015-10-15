/*
laserscan_to_pointcloudgrid_gimbal_node2
Z座標が負の点も処理対象に入れる。
ロボットの置かれた平面から見て地下の点を扱うことで、崖の認識に用いる。
*/

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "my_nav_msgs/conversions.h"
#include <vector>

//#define STOCK_SIZE 40
#define Z_MIN   0.05    // [m]
#define Z_MAX   0.60    // [m]
#define Z_BASEMENT_UPPER   -0.04 //[m]
#define Z_BASEMENT_LOWER   -1.0 //[m]
#define R_MIN   0.3     // [m]
#define R_MAX   2.0     // [m]
#define ROBOT_LENGTH  0.498    // [m] 
#define ROBOT_WIDTH   0.350    // [m]
#define EXCEPTION_AREA_FRONT	0.75 // 水平URGが被る部分を除去
#define EXCEPTION_AREA_WIDTH	0.35
#define EXCEPTION_AREA_HEIGHT	0.35

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher pub_, pub;

  LaserScanToPointCloud(ros::NodeHandle n, double resolution_, int STOCK_SIZE_) : 
    n_(n),
    laser_sub_(n_, "scan", 10),
    laser_notifier_(laser_sub_,listener_, "/map", 10)
  {
    laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.1));
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud2",1);
//   pub_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_in",1); //20150502kato
    pub = n_.advertise<sensor_msgs::PointCloud>("/cloud_debug",1);
//	pub = n_.advertise<sensor_msgs::PointCloud2>("/cloud_kato",1);//20150502kato
	loop_count = 0;
	resolution = resolution_;
	STOCK_SIZE = STOCK_SIZE_;
	clouds.resize(STOCK_SIZE);
	origins.resize(STOCK_SIZE);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
	loop_count++;
	if(loop_count > STOCK_SIZE)	loop_count = STOCK_SIZE;

	for(int i = STOCK_SIZE-1; i > 0; i--)
	{
		clouds[i] = clouds[i-1];
		origins[i] = origins[i-1];
	}

	tf::StampedTransform transform;
    try
    {
        projector_.transformLaserScanToPointCloud("/map", *scan_in, clouds[0], listener_);
		listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		origins[0].x = transform.getOrigin().x();
		origins[0].y = transform.getOrigin().y();
		origins[0].z = transform.getOrigin().z();
    }
    catch (tf::TransformException& e)
    {
        ROS_ERROR("%s", e.what());
        return;
    }
	/*
	ROS_INFO("points[0]: x= %f, y= %f, z= %f", clouds[0].points[0].x, clouds[0].points[0].y, clouds[0].points[0].z);//debug
	if(loop_count > 1)
	ROS_INFO("points[1]: x= %f, y= %f, z= %f", clouds[1].points[0].x, clouds[1].points[0].y, clouds[1].points[0].z);//debug
	ROS_INFO("points[N]: x= %f, y= %f, z= %f\n", clouds[loop_count-1].points[0].x, clouds[loop_count-1].points[0].y, clouds[loop_count-1].points[0].z);//debug
	*/

	sensor_msgs::PointCloud all_clouds;
	all_clouds.header = clouds[0].header;
	//all_clouds.header.frame_id = "/map";
	//all_clouds.header.stamp = ros::Time::now();
	int ranges_size = scan_in->ranges.size();
	//ROS_INFO("ranges_size = %d", ranges_size);//debug
	//ROS_INFO("points_size = %d", (int)clouds[0].points.size());//debug
	all_clouds.points.resize(ranges_size*loop_count);
	//ROS_INFO("all_points_size = %d", ranges_size*loop_count);//debug
	
	int valid_points_size = 0;
	for(int i = 0; i < loop_count; i++)
	{
		for(int j = 0; j < ranges_size; j++)
		{
			if( ((clouds[i].points[j].z - origins[i].z > Z_MIN && 
                clouds[i].points[j].z - origins[i].z < Z_MAX) || (clouds[i].points[j].z - origins[i].z < Z_BASEMENT_UPPER && clouds[i].points[j].z - origins[i].z > Z_BASEMENT_LOWER)) &&
				(fabs(clouds[i].points[j].x - origins[i].x) > ROBOT_LENGTH/2 || fabs(clouds[i].points[j].y - origins[i].y) > ROBOT_WIDTH/2) &&
				sqrtf(powf(clouds[i].points[j].x - origins[i].x, 2.0) + powf(clouds[i].points[j].y - origins[i].y, 2.0)) < R_MAX )// &&
                //!(((fabs(clouds[i].points[j].x - origins[i].x) > 0 && fabs(clouds[i].points[j].x - origins[i].x)) < EXCEPTION_AREA_FRONT && fabs(clouds[i].points[j].y - origins[i].y) < EXCEPTION_AREA_WIDTH/2) && clouds[i].points[j].z - origins[i].z < EXCEPTION_AREA_HEIGHT) )
			{
				all_clouds.points[valid_points_size].x = clouds[i].points[j].x;
				all_clouds.points[valid_points_size].y = clouds[i].points[j].y;
				all_clouds.points[valid_points_size].z = clouds[i].points[j].z;
				valid_points_size++;
			}
		}
	}
	all_clouds.points.resize(valid_points_size);
	pub.publish(all_clouds);
	
	sensor_msgs::PointCloud grid_cloud;
	my_nav_msgs::convertPointCloudToGridData(resolution, all_clouds, grid_cloud);

	sensor_msgs::PointCloud2 cloud2;
	sensor_msgs::convertPointCloudToPointCloud2(grid_cloud, cloud2);

	pub_.publish(cloud2);
  }

private:
	//sensor_msgs::PointCloud clouds[STOCK_SIZE];
	//geometry_msgs::Point32 origins[STOCK_SIZE];
	std::vector<sensor_msgs::PointCloud> clouds;
	std::vector<geometry_msgs::Point32> origins;
	int loop_count;
	double resolution;
	int STOCK_SIZE;
};

int main(int argc, char** argv)
{
  
	ros::init(argc, argv, "laserscan_to_pointcloudgrid_gimbal_node2");
	ros::NodeHandle n;

	double resolution = 0.05;
	if(ros::param::has("/laserscan_to_pointcloudgrid_gimbal_node2/resolution"))
	{
		double resolution_;
		ros::param::get("/laserscan_to_pointcloudgrid_gimbal_node2/resolution", resolution_);
		if(resolution_ > 0)
		{
			resolution = resolution_;
		}
	}
	int STOCK_SIZE = 40;
	if(ros::param::has("/laserscan_to_pointcloudgrid_gimbal_node2/stock_size"))
	{
		int STOCK_SIZE_;
		ros::param::get("/laserscan_to_pointcloudgrid_gimbal_node2/stock_size", STOCK_SIZE_);
		if(STOCK_SIZE_ > 0)
		{
			STOCK_SIZE = STOCK_SIZE_;
		}
	}

	LaserScanToPointCloud lstopc(n, resolution, STOCK_SIZE);

	ros::spin();
	ROS_INFO("laserscan_to_pointcloudgrid_gimbal_node2");
	return 0;
}
