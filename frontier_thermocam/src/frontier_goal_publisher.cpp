#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <math.h>
#include "nav_msgs/GridCells.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

geometry_msgs::Pose goal_pose;
nav_msgs::GridCells obstacles;
geometry_msgs::PoseStamped odom;
double ROBOT_RADIUS;
bool finish_flag;

ros::Publisher ExplorerEnd_pub;

#define MERGIN_FROM_OBSTACLE 1.5
#define DIST_MIN 0.5	// ロボットの半径0.5[m]以内にはgoalを設定しないようにする

class OdomTransform
{
public:
	OdomTransform() : tf_(),	target_frame_("map")
	{
		odom_sub_.subscribe(n_, "odom", 10);
		tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, tf_, target_frame_, 10);
		tf_filter_->registerCallback( boost::bind(&OdomTransform::msgCallback, this, _1) );
	} ;

private:
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;
	ros::NodeHandle n_;
	std::string target_frame_;

	//	Callback to register with tf::MessageFilter to be called when transforms are available
	void msgCallback(const boost::shared_ptr<const nav_msgs::Odometry>& odom_ptr) 
	{
		geometry_msgs::PoseStamped odom_in;
		odom_in.header = odom_ptr->header;
		odom_in.pose = odom_ptr->pose.pose;
		try 
		{
			tf_.transformPose(target_frame_, odom_in, odom);
		}
		catch (tf::TransformException &ex) 
		{
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
		}
	};

};

void obstacleCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	obstacles = *msg; // inflated_obstacleを取得
	//ROS_INFO("obstacleCallback");//debug
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	uint32_t width, height;
	double xi, yi, xg, yg, yaw, target_yaw;
	width = msg->info.width;
	height = msg->info.height;
	yaw = tf::getYaw(msg->info.origin.orientation);
	tf::TransformListener listener(ros::Duration(10));
	geometry_msgs::PoseStamped pose_in, pose_out, pose_tmp;
	double angle, angle_diff;
	double angle_min = 2*M_PI;
	double dist;
	double dist_min = 10000;
	bool flag_obstacle;
	bool flag_no_frontier = true;
	uint8_t flag_orientation;
	geometry_msgs::Pose pre_goal_pose;

	//ROS_INFO("mapCallback");//debug
	
	pre_goal_pose = goal_pose;

	for(uint32_t i = 0; i < msg->data.size(); i++)
	{
		if(msg->data[i] == 0)
		{
			int count = 0;
			target_yaw = 0;
			flag_orientation = 0;
			if(i % width != 0 && i / width != 0 && msg->data[i-1-width] == -1)	//左下
			{
				flag_orientation = 1;
				target_yaw += -3*M_PI/4;//3*M_PI/4;
				count++;
			}
			if(i / width != 0 && msg->data[i-width] == -1)	//真下
			{
				flag_orientation = 2;
				target_yaw += -M_PI/2;//M_PI;
				count++;
			}
			if(i % width != width-1 && i / width != 0 && msg->data[i+1-width] == -1)	//右下
			{
				flag_orientation = 3;
				target_yaw += -M_PI/4;//5*M_PI/4;
				count++;
			}
			if(i % width != 0 && msg->data[i-1] == -1)	//真左
			{
				flag_orientation = 4;
				target_yaw += M_PI;//M_PI/2;
				count++;
			}
			if(i % width != width-1 && msg->data[i+1] == -1)	//真右
			{
				flag_orientation = 5;
				target_yaw += 0;//3*M_PI/2;
				count++;
			}
			if(i % width != 0 && i / width != height-1 && msg->data[i-1+width] == -1)	//左上
			{
				flag_orientation = 6;
				target_yaw += +3*M_PI/4;//M_PI/4;
				count++;
			}
			if(i / width != height-1 && msg->data[i+width] == -1)	//真上
			{
				flag_orientation = 7;
				target_yaw += M_PI/2;//0;
				count++;
			}
			if(i % width != width-1 && i / width != height-1 && msg->data[i+1+width] == -1)	//右上
			{
				flag_orientation = 8;
				target_yaw += M_PI/4;//7*M_PI/4;
				count++;
			}/*
			else
			{
				flag_orientation = 0;
			}*/
			target_yaw /= count;
			if(target_yaw > M_PI)	target_yaw -= 2*M_PI; // -M_PI < target_yaw < M_PI

			if(flag_orientation != 0)
			{
				//ROS_INFO("frontier!");//debug
				
				// i番目のデータ点の座標
				xi = (i%msg->info.width)*msg->info.resolution + msg->info.resolution/2;
				yi = (i/msg->info.width)*msg->info.resolution + msg->info.resolution/2;
				// i番目のデータ点のglobal座標における座標
				xg = msg->info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
				yg = msg->info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);

				flag_obstacle = false;
				for(int j = 0; j < obstacles.cells.size(); j++) // 選んだ点上にinfrated_obstacleがあるかを判定
				{
					if(fabs(obstacles.cells[j].x - xg) < obstacles.cell_width/2//ROBOT_RADIUS*MERGIN_FROM_OBSTACLE//MERGIN_FROM_OBSTACLE*obstacles.cell_width
					   && fabs(obstacles.cells[j].y - yg) < obstacles.cell_height/2)//ROBOT_RADIUS*MERGIN_FROM_OBSTACLE)//MERGIN_FROM_OBSTACLE*obstacles.cell_height)
					{
						flag_obstacle = true;
						break;
					}
				}
				
				if(!flag_obstacle)
				{
					flag_no_frontier = false;

					pose_in.header.frame_id = "map";
					pose_in.header.stamp = ros::Time::now();
					pose_in.pose.position.x = msg->info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
					pose_in.pose.position.y = msg->info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);

					pose_in.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);

					
					dist = sqrt(pow(pose_in.pose.position.x-pre_goal_pose.position.x, 2.0) + pow(pose_in.pose.position.y-pre_goal_pose.position.y, 2.0));
					if(dist < dist_min && dist > DIST_MIN)
					{
						dist_min = dist;
						pose_tmp = pose_in;
					}
					
					/*
					// 現在のロボットに最も近い位置のゴールを選ぶ
					dist = sqrt(pow(pose_in.pose.position.x-odom.pose.position.x, 2.0) + pow(pose_in.pose.position.y-odom.pose.position.y, 2.0));
					if(dist < dist_min)
					{
						dist_min = dist;
						pose_tmp = pose_in;
					}
					*/
					/*
					// 現在のロボットの向きに最も近いゴールを選ぶ
					angle = atan2(pose_in.pose.position.y-odom.pose.position.y, pose_in.pose.position.x-odom.pose.position.x);
					angle_diff = fabs(angle	- tf::getYaw(odom.pose.orientation));
					if(angle_diff > 2*M_PI)	angle_diff -= 2*M_PI;			
					if(angle_diff < angle_min)
					{
						angle_min = angle_diff;
						pose_tmp = pose_in;
					}
					*/
				}
			}
		}	
	}
	if(flag_no_frontier)
	{
		ROS_INFO("Expoleration finish!");
		finish_flag = true;

		std_msgs::Bool end_flag;
		end_flag.data = true;
		ExplorerEnd_pub.publish(end_flag);
		return;
	}
	goal_pose = pose_tmp.pose;
	//ROS_INFO("Set goal x:%f  y:%f", goal_pose.position.x, goal_pose.position.y);//debug
}

int main(int argc, char** argv){
	ros::init(argc, argv, "frontier_thermocam_node");

	ros::NodeHandle n;
	ros::Publisher goal_pub = n.advertise<geometry_msgs::Pose>("frontier_goal", 100);
	ros::Subscriber sub_map = n.subscribe("nav_map", 10, mapCallback);
	ros::Subscriber sub_obstacle = n.subscribe("move_base/local_costmap/inflated_obstacles", 10, obstacleCallback);

	ExplorerEnd_pub = n.advertise<std_msgs::Bool>("explorer_end_flag", 100);

	ros::Rate loop_rate(1.0);

	odom.pose.orientation.w = 1.0;

	if(ros::param::has("/move_base/local_costmap/robot_radius"))
	{
		ros::param::get("/move_base/local_costmap/robot_radius", ROBOT_RADIUS);
	}
	else
	{
		ROS_ERROR("Parameter: %s doesn't exist!", "/move_base/local_costmap/robot_radius");
		ros::isShuttingDown();
	}

	finish_flag = false;

	ROS_INFO("Expoleration start!");

	while(ros::ok())
	{
		goal_pub.publish(goal_pose);
		ROS_INFO("Publish goal x:%f  y:%f", goal_pose.position.x, goal_pose.position.y);//debug

		if(finish_flag)
		{
			return 0;
		}

		ros::spinOnce();
	
		loop_rate.sleep();
	}
	return 0;
}
