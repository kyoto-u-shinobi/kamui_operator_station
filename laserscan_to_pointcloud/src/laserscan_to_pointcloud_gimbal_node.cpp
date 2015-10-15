#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <cmath>

#define Z_MIN   0.08    // [m]
#define Z_MAX   0.70    // [m]
#define R_MIN   0.3     // [m]
#define R_MAX   1.0     // [m]
#define ROBOT_LENGTH  0.498    // [m]
#define ROBOT_WIDTH   0.350    // [m]

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "scan", 1000),
    laser_notifier_(laser_sub_,listener_, "/base_link", 1000)
  {
    laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.1));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_in",1);	//cloud_in// octomapへ渡すためのデータ
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud cloud, cloud_out;
    try
    {
        // LaserScan から PointCloud に変換
        projector_.transformLaserScanToPointCloud("/base_link",*scan_in, cloud, listener_);
    }
    catch (tf::TransformException& e)
    {
        ROS_ERROR("%s", e.what());
        return;
    }

    cloud_out.header = cloud.header;

    size_t size = cloud.points.size();
    geometry_msgs::Point32 points[size];
    int z_count = 0;

    for(int j = 0; j < size; j++)
    {
		// ロボットからの距離がR_MAX以下かつZ座標がZ_MIN以上Z_MAX以下のもののみを取り出す
        //if(cloud.points[j].z > Z_MIN && cloud.points[j].z < Z_MAX && sqrt(pow(cloud.points[j].x, 2.0) + pow(cloud.points[j].y, 2.0)) > R_MIN && sqrt(pow(cloud.points[j].x, 2.0) + pow(cloud.points[j].y, 2.0)) < R_MAX)
		if(cloud.points[j].z > Z_MIN && cloud.points[j].z < Z_MAX && fabs(cloud.points[j].x) > ROBOT_LENGTH/2 && fabs(cloud.points[j].y) > ROBOT_WIDTH/2 && sqrtf(powf(cloud.points[j].x, 2.0) + powf(cloud.points[j].y, 2.0)) < R_MAX)
        {
            points[z_count].x = cloud.points[j].x;   //ROS_INFO("x");
            points[z_count].y = cloud.points[j].y;   //ROS_INFO("y");
            points[z_count].z = cloud.points[j].z;   //ROS_INFO("z");

            z_count++;
        }
    }

    cloud_out.points.resize(z_count);
    for(int k = 0; k < z_count; k++)
    {
        cloud_out.points[k].x = points[k].x;
        cloud_out.points[k].y = points[k].y;
        cloud_out.points[k].z = points[k].z;
    }

	// PointCloud から PointCloud2 に変換
    sensor_msgs::convertPointCloudToPointCloud2(cloud_out, cloud2);

    scan_pub_.publish(cloud2);
  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "laserscan_to_pointcloud_gimbal_node");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}
