#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <math.h>
#include "nav_msgs/GridCells.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose goal_pose;
nav_msgs::GridCells obstacles;

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
	tf::StampedTransform transform;
	try
	{
		//listener.transformPose("base_link", ros::Time(0), pose_in, "map", pose_out);
		ros::Time now = ros::Time::now();
		listener.waitForTransform("/base_link", "/map", now, ros::Duration(0.1));
		//listener.lookupTransform("/base_link", "/map", now, transform);
		//ROS_INFO("Transform!: %f, %f", transform.getOrigin().x(), transform.getOrigin().y());//debug;
		listener.transformPose("base_link", now, pose_in, "map", pose_out);
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("TransformPose Error: %s", ex.what());
	}
}

void obstacleCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	obstacles = *msg;
	ROS_INFO("obstacleCallback");//debug
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	uint32_t width, height;
	double xi, yi, xg, yg, yaw;
	width = msg->info.width;
	height = msg->info.height;
	yaw = tf::getYaw(msg->info.origin.orientation);
	tf::TransformListener listener(ros::Duration(10));
	geometry_msgs::PoseStamped pose_in, pose_out, pose_tmp;
	double angle;
	double angle_min = M_PI;
	bool flag_obstacle;

	//ROS_INFO("mapCallback");//debug
	/*
	tf::StampedTransform transform;
	ros::Time now = ros::Time::now();
	try 
	{
		listener.waitForTransform("/base_link", "/map", now, ros::Duration(3.0));
		listener.lookupTransform("/base_link", "/map", now, transform);
	}
	catch (tf::TransformException &ex) 
	{
		ROS_ERROR("Failure %s\n", ex.what()); //Print exception which was caught
	}
	*/
	for(uint32_t i = 0; i < msg->data.size(); i++)
	{
		flag_obstacle = false;
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
				xg = msg->info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
				yg = msg->info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);

				for(int j = 0; j < obstacles.cells.size(); j++)
				{
					if(fabs(obstacles.cells[j].x - xg) < 4*obstacles.cell_width
					   && fabs(obstacles.cells[j].y - yg) < 4*obstacles.cell_height)
					{
						flag_obstacle = true;
						break;
					}
				}
				
				if(!flag_obstacle)
				{
					//goal_pose.pose.position.x = msg->info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
					//goal_pose.pose.position.y = msg->info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);
					pose_in.header.frame_id = "map";
					pose_in.header.stamp = ros::Time::now();
					pose_in.pose.position.x = msg->info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
					pose_in.pose.position.y = msg->info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);
					pose_in.pose.orientation.w = 1.0;
				
					//transformPose(listener, pose_in, pose_out);
					/*
					pose_out.pose.position.x = pose_in.pose.position.x + transform.getOrigin().x();
					pose_out.pose.position.y = pose_in.pose.position.y + transform.getOrigin().y();
					
					//ROS_INFO("%f %f %f %F", pose_in.pose.position.x,pose_in.pose.position.y,pose_out.pose.position.x,pose_out.pose.position.y);//debug
					//dist = sqrt(pow(pose_out.pose.position.x, 2.0) + pow(pose_out.pose.position.y, 2.0));
					angle = fabs(atan2(pose_out.pose.position.y, pose_out.pose.position.x));
					if(angle < angle_min)
					{
						angle_min = angle;
						pose_tmp = pose_in;
					}
					*/
					break;
				}
			}
		}
		//goal_pose = pose_tmp.pose;
	}
	goal_pose = pose_in.pose;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "frontier_thermocam_node");

	ros::NodeHandle n;
	ros::Subscriber sub_map = n.subscribe("nav_map", 1000, mapCallback);
	ros::Subscriber sub_obstacle = n.subscribe("move_base/local_costmap/inflated_obstacles", 1000, obstacleCallback);

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	ros::Rate loop_rate(1);

	bool start_flag = false;
	ros::param::set("nav_start", false);
/*
	while(!start_flag)
	{
		ros::param::get("nav_start", start_flag);
	}
*/
	while(ros::ok())
	{
		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
	
		move_base_msgs::MoveBaseGoal goal;
	
		//we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
/*
		if(!flag)
		{
			goal.target_pose.header.frame_id = "base_footprint";
			goal_pose.position.x += 0.5;
			goal_pose.position.y = 0.0;
		}
*/
		//goal_pose.orientation.w = 1.0;

		ros::spinOnce();
	
		goal.target_pose.pose = goal_pose;
	
		ROS_INFO("Sending goal");
		ac.sendGoal(goal);
	
		ac.waitForResult();
	
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Hooray, the base moved 1 meter forward");
		}
		else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
		{
			ROS_INFO("Aborted...");
		}
		else
		{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

		loop_rate.sleep();
	}
	return 0;
}
