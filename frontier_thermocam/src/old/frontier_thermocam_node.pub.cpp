#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <math.h>

geometry_msgs::PoseStamped goal_pose;
/*
class PoseTransform
{
public:
	PoseTransform() : tf_(),	target_frame_("base_link")
	{
		point_sub_.subscribe(n_, "turtle_point_stamped", 10);
		tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(point_sub_, tf_, target_frame_, 10);
		tf_filter_->registerCallback( boost::bind(&PoseTransform::msgCallback, this, _1) );
	} ;

private:
	message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_;
	ros::NodeHandle n_;
	std::string target_frame_;

	//	Callback to register with tf::MessageFilter to be called when transforms are available
	void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr) 
	{
		geometry_msgs::PoseStamped point_out;
		try 
		{
			tf_.transformPose(target_frame_, *point_ptr, point_out);
		}
		catch (tf::TransformException &ex) 
		{
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
		}
	};

	void tf_pose(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr) 
	{
		geometry_msgs::PoseStamped point_out;
		try 
		{
			tf_.transformPose(target_frame_, *point_ptr, point_out);
		}
		catch (tf::TransformException &ex) 
		{
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
		}
	};

};
*/

void transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped& pose_in, geometry_msgs::PoseStamped& pose_out)
{
	try
	{
		listener.transformPose("base_link", pose_in, pose_out);
		//ROS_INFO("Transform!");//debug;
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("TransformPose Error: %s", ex.what());
	}
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	uint32_t width, height;
	double xi, yi, yaw;
	width = msg->info.width;
	height = msg->info.height;
	yaw = tf::getYaw(msg->info.origin.orientation);
	tf::TransformListener listener(ros::Duration(10));
	geometry_msgs::PoseStamped pose_in, pose_out, pose_tmp;
	double dist;
	double dist_min = 10000;

	//ROS_INFO("mapCallback");//debug

	for(uint32_t i = 0; i < msg->data.size(); i++)
	{
		if(msg->data[i] == 0)
		{
			if((i % width != 0 && msg->data[i-1] == -1) \
				 || (i % width != width-1 && msg->data[i+1] == -1) \
				 || (i / width != 0 && msg->data[i-width] == -1) \
				 || (i / width != height-1 && msg->data[i+width] == -1))
			{
				//ROS_INFO("frontier!");//debug
				xi = (i%msg->info.width)*msg->info.resolution + msg->info.resolution/2;
				yi = (i/msg->info.width)*msg->info.resolution + msg->info.resolution/2;
				//goal_pose.pose.position.x = msg->info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
				//goal_pose.pose.position.y = msg->info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);
				pose_in.header.frame_id = "map";
				pose_in.header.stamp = ros::Time::now();
				pose_in.pose.position.x = msg->info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
				pose_in.pose.position.y = msg->info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);
				pose_in.pose.orientation.w = 1.0;
				/*try 
				{
					listener.transformPose("map", pose_in, pose_out);
				}
				catch (tf::TransformException &ex) 
				{
					printf ("Failure %s\n", ex.what()); //Print exception which was caught
				}*/
				boost::bind(&transformPose, boost::ref(listener), pose_in, pose_out);
				dist = sqrt(pow(pose_out.pose.position.x, 2.0) + pow(pose_out.pose.position.y, 2.0));
				if(dist < dist_min && pose_out.pose.position.x >= 0)
				{
					dist_min = dist;
					pose_tmp = pose_in;
				}
			}
		}
		goal_pose = pose_tmp;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "frontier_thermocam_node");

	ros::NodeHandle n;
	ros::Subscriber sub_map = n.subscribe("nav_map", 1000, mapCallback);
	ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);


	ros::Rate loop_rate(0.5);

	while(ros::ok())
	{
		//we'll send a goal to the robot to move 1 meter forward
		goal_pose.header.frame_id = "map";
		goal_pose.header.stamp = ros::Time::now();

		//goal_pose.position.x = 1.0;
		
	
		ROS_INFO("Sending goal x:%f,	y:%f", goal_pose.pose.position.x, goal_pose.pose.position.y);
		pub_goal.publish(goal_pose);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
