#include "ros/ros.h"
#include "sensor_msgs/Range.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thermo_range_node");

    ros::NodeHandle n;	// nodeへのハンドラ

    ros::Publisher pub_range = n.advertise<sensor_msgs::Range>("range", 1000);
    sensor_msgs::Range range;

    double range0;
    range.header.frame_id = "/thermocam_link";
    if(ros::param::has("/thermo_range_node/field_of_view"))
    {
        ros::param::get("/thermo_range_node/field_of_view", range.field_of_view);
    }
    else
    {
        range.field_of_view = 40*M_PI/180;
    }
    if(ros::param::has("/thermo_range_node/range"))
    {
        ros::param::get("/thermo_range_node/range", range0);
        range.range = range0;
        range.min_range = range0;
        range.max_range = range0;
    }
    else
    {
        range.range = 0.4;
        range.min_range = 0.4;
        range.max_range = 0.4;
    }

    ros::Rate loop_rate(10);	// ループ頻度を設定(Hz)

    while (ros::ok())
    {

        pub_range.publish(range);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
