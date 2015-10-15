#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>
#include "random_numbers/random_numbers.h"

#define PI 3.14159265358979

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

	double CONTROL_F = 40;
	ros::Rate loop_rate(CONTROL_F);
	tf::TransformBroadcaster map2base, base2roll, roll2pitch;

	int count = 0;
	while(ros::ok())
	{
		ros::Time current_time = ros::Time::now();

		random_numbers::RandomNumberGenerator rand;

		int speed = 8;
		double roll = 0.5*sin(2*PI*count/CONTROL_F);//*(1+rand.uniform01()/1000.0));
		double pitch = 0.5*cos(2*PI*count/CONTROL_F);//*(1+rand.uniform01()/1000.0));
		double z = 0.3;
		double x = 0.5*(1 + sin(PI*count/CONTROL_F));

		base2roll.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(roll, 0, 0), // 回転行列
				tf::Vector3(0, 0, z)),	// 移動
				current_time,	// タイムスタンプ
				"base_link",	// リンクの親ノードの名前
				"laser_"	// 子ノードの名前
			)
		);
		roll2pitch.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(0, pitch, 0), // 回転行列
				tf::Vector3(0, 0, 0)),	// 移動
				current_time,	// タイムスタンプ
				"laser_",	// リンクの親ノードの名前
				"laser"	// 子ノードの名前
			)
		);
		map2base.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), // 回転行列
				tf::Vector3(x, 0, 0)),	// 移動
				current_time,	// タイムスタンプ
				"map",	// リンクの親ノードの名前
				"base_link"	// 子ノードの名前
			)
		);

		sensor_msgs::LaserScan scan;
		scan.header.frame_id = "laser";
		scan.header.stamp = current_time;

		scan.angle_min = -135.0 * PI / 180.0;
		scan.angle_max = 135.0 * PI / 180.0;
		scan.angle_increment = 0.25 * PI / 180.0;

		int ranges_size = (int)((scan.angle_max - scan.angle_min)/scan.angle_increment);
		scan.ranges.resize(ranges_size);
		scan.intensities.resize(ranges_size);

		scan.range_min = 0.06;
		scan.range_max = 10.0;

		for(int i=0; i<ranges_size; i++)
		{
			scan.ranges[i] = scan.range_min + rand.uniform01()/100.0 + 1.5;//*count/CONTROL_F;
			scan.intensities[i] = 0;
		}

		pub.publish(scan);

		ros::spinOnce();

		loop_rate.sleep();

		++count;
		//if(count > CONTROL_F)	count = 0;
	}

	return 0;
}
