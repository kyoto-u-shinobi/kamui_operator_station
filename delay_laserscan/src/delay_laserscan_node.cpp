#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

class DelayLaserScan{

public:

  DelayLaserScan(int offset_)
  {
	pub = n.advertise<sensor_msgs::LaserScan>("scan_delay", 10);
	sub = n.subscribe("scan", 10, &DelayLaserScan::scanCallback, this);
	offset = offset_;
	ROS_INFO("offset = %d", offset);//debug
	scans.resize(offset);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
	sensor_msgs::LaserScan scan_out;

	if(offset > 0)
	{
		for(int i = offset-1; i > 0; i--)
		{
		    scans[i] = scans[i-1];
		}
		scans[0] = *scan_in;

		//scan_out = scans[offset-1];
		//scan_out.header = scan_in->header;
		scan_out = *scan_in;
		scan_out.ranges = scans[offset-1].ranges;

		scan_out.intensities.resize(0);
		//scan_out.header.stamp = (ros::Time)0;
	}
	else
	{
		scan_out = *scan_in;
	}

	pub.publish(scan_out);
  }

private:

  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  //sensor_msgs::LaserScan scans[40];
  std::vector<sensor_msgs::LaserScan> scans;
  int offset;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "delay_laserscan_node");

	int offset = 0;
    if(ros::param::has("/delay_laserscan_node/modify_delay"))
    {
        int offset_;
        ros::param::get("/delay_laserscan_node/modify_delay", offset_);
        if(offset_ >= 0)
        {
            offset = offset_;
        }
    }

	DelayLaserScan dls(offset);

	ros::spin();

	return 0;
}
