#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "wall_follower/walls.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GridCells.h"
#include <math.h>
#include <tf/transform_datatypes.h>
#include "dynamixel_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/UInt32MultiArray.h"
#include "set_state/pubState.h"
#include "geometry_msgs/Pose2D.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/PoseArray.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define	ROAD_WIDTH	850		//!< 道の幅[m]//900mm...(JapanOpen),1200mm(WorldCup)
#define	MinF 250//350//270 //ROAD_WIDTH/2-100,				/*前方の閾値 wall_followerの数値＋アルファくらいがいい*/
#define	MaxL ROAD_WIDTH/2+100 //450				/*左方の閾値(max)*/  
#define Ld	350//400		// 壁から離れるべき距離の目標値 [mm]  

#define R_th	0.6//0.4//0.15	// [m]
#define R_th_fin	0.3//0.15	// [m]
#define TH_th	0.2//0.1	// [rad]
#define PARALLEL_th			20 // [mm]
#define WALL_DIST_ERR_th	20 // [mm]
#define CHECK_th	0.5 // [m]
#define CHECK_ANGLE_th	1.0 // [rad]

#define DYNA_ERR_th	0.1//0.05	// [rad]

#define OBSERVED	127		// 緑（rviz上での表示色）
#define UNSIGHT		-2		// 黄

#define GOAL_SET_COUNT_MAX	5	// goal再設定回数の上限
#define WAIT_COUNT 5
#define RATE		10

#define SLEEP_TIME  5.0     // Dynamixel タイムアウト時間 [sec]
#define SLEEP_TIME2 2.5//2.0     // Dynamixel タイムアウト時間 [sec]

#define VICTIM_NUM  2   // victimの総数

//geometry_msgs::Pose curpos;
nav_msgs::OccupancyGrid map0;
double distL, distR;
int flag_init = 0;
int flag_init2 = 0;
int flag_dyna = 0;
int flag_dyna_scan = 0;
int flag_dyna_screen = 0;
int flag_map = 0;
double error_cur = 2*M_PI;
std::string state0 = "Waiting";
nav_msgs::GridCells obs0;
int victim_count;


//====================================================//
//	@class 距離データの定義
//====================================================//
class Distance{
public:
	Distance(){
		Clear();
	}
	~Distance(){
	}
	void Clear(void){//適当に大きな値で初期化
		front  = 10000;
		left   = 10000;
		right  = 10000;
		left2  = 10000;
		right2 = 10000;
	}

public:
	double front;	//前方 [mm]
	double left;	//左前 [mm]
	double right;	//右前 [mm]
	double left2;	//左後 [mm]
	double right2;	//右後 [mm]
};

Distance dist;

void wallsCallback(const wall_follower::walls::ConstPtr& msg)
{
	dist.Clear();
	
	dist.front = msg->front;
	dist.left = msg->left;
	dist.left2 = msg->left2;
	dist.right = msg->right;
	dist.right2 = msg->right2;

	distL = (dist.left + dist.left2)/2;
	distR = (dist.right + dist.right2)/2;
	
	//ROS_INFO("%f %f %f %f %f", dist.front, dist.right, dist.left, dist.right2, dist.left2);//debug
	//ROS_INFO("%f %f", distL, distR);//debug
	
	if(!flag_init2)
	{
		flag_init2 = 1;
		ROS_INFO("wallsCallback");
	}
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	map0 = *msg;
	if(!flag_map)
	{
		flag_map = 1;
		ROS_INFO("mapCallback");
	}
}

void DynaCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{                      
	error_cur = fabs(msg->error);
	//ROS_INFO("current error: %f [rad]", error_cur);	//debug
}

void stateCallback(const std_msgs::String::ConstPtr& state_msg)
{
	// 取得したstate_msgを変数に格納する
	state0 = state_msg -> data;
}

void obstacleCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	obs0 = *msg;
}

bool detectGoalOverObstacle(double x, double y)
{
    nav_msgs::GridCells obs = obs0;
    for(int i=0; i < obs.cells.size(); i++)
    {
        if(fabs(x - obs.cells[i].x) < obs.cell_width && fabs(y - obs.cells[i].y) < obs.cell_height)
        {
            // Goalとinflated_obstacleが重なっていたとき
            return true;
        }
    }

    // Goalとinflated_obstacleが重なっていなかったとき
    return false;
}

void victimsCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    victim_count = msg->poses.size();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "frontier_with_wallfollower_node");

	ros::NodeHandle n;

	ros::Subscriber walls_sub = n.subscribe("walls", 1000, wallsCallback);
	ros::Subscriber map_sub = n.subscribe("filtered_map", 10, mapCallback);
	ros::Subscriber dyna_sub = n.subscribe("tilt_controller/state", 1000, DynaCallback);
	ros::Publisher dyna_pub = n.advertise<std_msgs::Float64>("tilt_controller/command", 1000);
	ros::Publisher cells_pub = n.advertise<nav_msgs::GridCells>("NG_cells", 1000);
	ros::ServiceClient state_client = n.serviceClient<set_state::pubState>("pubState");
	ros::Subscriber State_sub = n.subscribe("state", 100, stateCallback);
    //ros::ServiceClient launch_client = n.serviceClient<std_srvs::Empty>("launch_move_base");
	ros::ServiceClient clear_costmap_client = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    ros::Subscriber obstacle_sub = n.subscribe("/move_base/global_costmap/inflated_obstacles", 100, obstacleCallback);
    ros::Subscriber victimsdata_subscriber = n.subscribe("victims", 100, victimsCallback);


	ros::Publisher inipos_pub = n.advertise<geometry_msgs::PoseStamped>("inipos", 1000);//debug

	std::string mode;	
	geometry_msgs::Pose2D inipos, curpos, chkpos;
	geometry_msgs::PoseStamped iniposStamped;
	std_msgs::Float64 dyna_goal;
	double curdist, chkdist, errang, chkang;
	tf::TransformListener listener;
	geometry_msgs::Pose goal_pose;
	nav_msgs::OccupancyGrid map;
	double yaw;
    std_msgs::UInt32MultiArray ng_index;
	int ng_count = 0;
	nav_msgs::GridCells cells;
	set_state::pubState state_srv;
	tf::StampedTransform transform;
	std_srvs::Empty movebase_srv;

	ros::Rate loop_rate(10);

	std::string state = "Waiting";	// ロボットの状態
	state_srv.request.state = state;
	state_client.call(state_srv);

	int flag = 0;
	ROS_INFO("flag = %d", flag);
	iniposStamped.header.frame_id = "/map";
	iniposStamped.header.stamp = ros::Time::now();
	inipos_pub.publish(iniposStamped);//debug
	int flag_inipos = 0;
	int flag_launch = 0;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

	std::string ac_state;
    int flag_goal_reset = 1;
    double goal_x, goal_y;
    unsigned int index;
    geometry_msgs::Pose2D chkpos2, chkpos20;

    ros::param::set("/frontier", true);
    int flag_beginning = 1;//Robocup2014用 最初は右手法にする
    victim_count = 0;

    double pre_time, interval_time;
    pre_time = ros::Time::now().toSec();


	while (ros::ok())
	{
		// /map座標系でのbase_linkの座標を取得
		try
		{
			//listener.lookupTransform("/map", "/base_link", ros::Time(0), transform); // /mapにおけるbaseの位置
			ros::Time now = ros::Time(0);
			listener.waitForTransform("/map", "/base_link", now, ros::Duration(3.0));
			//listener.lookupTransform("/map", "/base_link", now, transform);
			//ros::param::set("wall_follower_node/right_or_left", "right");    //20150501kami 停止したままにならないために
		}
    	catch (tf::TransformException ex)
		{
            	ROS_ERROR("%s", ex.what());
            	//ros::param::set("wall_follower_node/right_or_left", "stop");    //20150426kami エラー起きてたらその場で止まってみる
            	//ROS_INFO("error_Stop!");
		}
		curpos.x = transform.getOrigin().x();
		curpos.y = transform.getOrigin().y();
		curpos.theta = tf::getYaw(transform.getRotation());
		
		//ROS_INFO("flag = %d", flag);	//debug

        /*
        // 初期位置の再設定（壁と平行になったとき）
		if((flag == 2 || flag == 3 || flag == 4) && flag_inipos == 0)
		{
			double para_dist, wall_dist_err;
			if(mode == "right")
			{
				para_dist = fabs(dist.right - dist.right2);
				wall_dist_err = fabs((dist.right + dist.right2)/2 - Ld);
			}
			else if(mode == "left")
			{
				para_dist = fabs(dist.left - dist.left2);
				wall_dist_err = fabs((dist.left + dist.left2)/2 - Ld);
			}
			if(para_dist < PARALLEL_th && wall_dist_err < WALL_DIST_ERR_th)	// ある程度壁と平行になったときiniposを更新
			{
				inipos = curpos;

				//debug
				iniposStamped.pose.position.x = inipos.x;
				iniposStamped.pose.position.y = inipos.y;
				iniposStamped.pose.orientation = tf::createQuaternionMsgFromYaw(inipos.theta);				
				iniposStamped.header.stamp = ros::Time::now();
				inipos_pub.publish(iniposStamped);//debug

				ROS_INFO("Set initial position 2");
				flag_inipos = 1;
				flag = 2;
				ROS_INFO("flag = %d", flag);
			}
		}
        */

		if(flag == 0 && state0 == "Following")
		{
			flag = 1;
			ROS_INFO("flag = %d", flag);
		}
		if(flag == 1 && flag_init2 == 1 && flag_map == 1)
		{
			ROS_INFO("flag_beginning = %d", flag_beginning);
            if(flag_beginning)
            {
				mode = "right";
                dyna_goal.data = -M_PI/2;
				//mode = "left";
                //dyna_goal.data = M_PI/2;
            }
            else
            {
                map = map0;
                yaw = tf::getYaw(map.info.origin.orientation);

                double xi, yi, xg, yg;
                double thd, target;
                double dist_cur;
                double dist_unsight = 10000;

                for(int i=0; i<map.data.size(); i++)
                {
                    if(map.data[i] == UNSIGHT)
                    {
                        xi = (i%map.info.width)*map.info.resolution + map.info.resolution/2;
                        yi = (i/map.info.width)*map.info.resolution + map.info.resolution/2;
                        xg = map.info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
                        yg = map.info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);

                        dist_cur = sqrt(pow(curpos.x-xg, 2.0) + pow(curpos.y-yg, 2.0));


                        if(dist_unsight > dist_cur)
                        {
                            dist_unsight = dist_cur;
                            thd = atan2(yg-curpos.y, xg-curpos.x);
                        }
                    }
                }
                target = thd - curpos.theta;
                if(target > M_PI)	target -= 2*M_PI;
                else if(target < -M_PI)	target += 2*M_PI;

                if(target < 0)	// 未観測障害物は右の壁に近い
                {
                    mode = "right";
                    dyna_goal.data = -M_PI/2;
                }
                else	// 未観測障害物は左の壁に近い
                {
                    mode = "left";
                    dyna_goal.data = M_PI/2;
                }
            }


            if(state0 == "Following")    dyna_pub.publish(dyna_goal);
			if(flag_dyna == 0 && error_cur > DYNA_ERR_th)
			{
				flag_dyna = 1;
                pre_time = ros::Time::now().toSec();
			}
            else if(flag_dyna == 1 && (error_cur < DYNA_ERR_th || interval_time > SLEEP_TIME))
            {
				inipos = curpos;

				//debug
				iniposStamped.pose.position.x = inipos.x;
				iniposStamped.pose.position.y = inipos.y;
				iniposStamped.pose.orientation = tf::createQuaternionMsgFromYaw(inipos.theta);				
				iniposStamped.header.stamp = ros::Time::now();
				inipos_pub.publish(iniposStamped);//debug

				ROS_INFO("Set initial position");
				ros::param::set("wall_follower_node/right_or_left", mode);
				ROS_INFO("Follow %s walls!", mode.c_str());

				flag = 2;
				ROS_INFO("flag = %d", flag);
				flag_dyna = 0;
				flag_inipos = 0;

                flag_beginning = 0;
			}

		}
		else if(flag == 2)
		{
			curdist = sqrt(pow(inipos.x - curpos.x, 2) + pow(inipos.y - curpos.y, 2));
			if(curdist > R_th)
			{
				flag = 3;
				ROS_INFO("flag = %d", flag);
			}
		}
		else if(flag == 3)
		{
			curdist = sqrt(pow(inipos.x - curpos.x, 2) + pow(inipos.y - curpos.y, 2));
			if(curdist > R_th)
			{
				flag = 4;
				ROS_INFO("flag = %d", flag);
			}
		}
		else if(flag == 4)
		{
			curdist = sqrt(pow(inipos.x - curpos.x, 2) + pow(inipos.y - curpos.y, 2));
			errang = inipos.theta - curpos.theta;

			chkdist = sqrt(pow(chkpos.x - curpos.x, 2) + pow(chkpos.y - curpos.y, 2));	
			chkang = chkpos.theta - curpos.theta;

            
			if(curdist < R_th && fabs(errang) < TH_th)	// 初期位置付近に戻ってきたとき
            {
				ros::param::set("wall_follower_node/right_or_left", "stop");
				ROS_INFO("Stop!");
				flag = 5;
				ROS_INFO("flag = %d", flag);
                dyna_goal.data = 0;
                if(state0 == "Following")    dyna_pub.publish(dyna_goal);

				state = "Moving";	// Movingモードへ
				state_srv.request.state = state;
				state_client.call(state_srv);
                flag_goal_reset = 1;
			}
            
            // 20140610
            /*if(curdist < R_th_fin && fabs(errang) < TH_th)	// 初期位置付近に戻ってきたとき
            {
                ros::param::set("wall_follower_node/right_or_left", "stop");
                ROS_INFO("Stop!");
                flag = 6;
                ROS_INFO("flag = %d", flag);
                dyna_goal.data = 0;
                if(state0 == "Following")    dyna_pub.publish(dyna_goal);

                state = "Finish";	// Finishモードへ
                state_srv.request.state = state;
                state_client.call(state_srv);
                flag_goal_reset = 1;
            }
	    */
            /*
            //RoboCup2014
            if(curdist < R_th && fabs(errang) < TH_th)	// 初期位置付近に戻ってきたとき
            {
                ros::param::set("wall_follower_node/right_or_left", "stop");
                ROS_INFO("Stop!");
                dyna_goal.data = 0;
                if(state0 == "Following")    dyna_pub.publish(dyna_goal);

                if(victim_count < VICTIM_NUM)    // 発見したvictimがVICTIM_NUMより少ないとき
                {
                    state = "Waiting";	// Waitingモードへ
                    state_srv.request.state = state;
                    state_client.call(state_srv);

                    ros::Duration(3.0).sleep();

                    state = "Following";	// Followingモードへ
                    state_srv.request.state = state;
                    state_client.call(state_srv);
                    ros::Duration(3.0).sleep();

                    flag = 0;
                    ROS_INFO("flag = %d", flag);
                    flag_beginning = 1;
                }
                else
                {
                    flag = 6;
                    ROS_INFO("flag = %d", flag);

                    state = "Finish";	// Finishモードへ
                    state_srv.request.state = state;
                    state_client.call(state_srv);
                }
            }
            */
            /*
            else if(curdist < R_th && (fabs(errang) < TH_th + M_PI || fabs(errang) < TH_th - M_PI))	// 初期位置付近に戻ってきたとき
            {
                ros::param::set("wall_follower_node/right_or_left", "stop");
                ROS_INFO("Returned to initial position!");
                flag = 6;
                ROS_INFO("flag = %d", flag);
                dyna_goal.data = 0;
                if(state0 == "Following")    dyna_pub.publish(dyna_goal);

                state = "Finish";	// Finishモードへ
                state_srv.request.state = state;
                state_client.call(state_srv);
                flag_goal_reset = 1;
            }
            */
            
            // 曲がり角の隅をよく見渡す
            else if ((state0 == "Following" && dist.front < MinF && flag_dyna_scan == 0) || flag_dyna_scan == 1 || flag_dyna_scan == 2)	// followingかつ前方に壁があるとき
			{
				ros::param::set("wall_follower_node/right_or_left", "stop");
				//if(!flag_dyna_screen) ROS_INFO("Stop! (temp)");

				chkpos = curpos;

				if(flag_dyna_scan == 0)
				{
					flag_dyna_scan = 1;
                    ROS_INFO("front= %f",dist.front);
					ROS_INFO("Stop! (temp)");
				}
				else if(flag_dyna_scan == 1)
				{
                    dyna_goal.data = 0;


                    //20150423debug_kami ちゃんと反対側まで向く
                    if(mode == "right")
                    {
                        dyna_goal.data = M_PI/4;
                    }
                    else if(mode == "left")
                    {
                        dyna_goal.data = -M_PI/4;
                    }


				}
				else if(flag_dyna_scan == 2)
				{
					if(mode == "right")
					{
                        dyna_goal.data = -M_PI/2;
					}
					else if(mode == "left")
					{
                        dyna_goal.data = M_PI/2;
					}
				}
				
                if(state0 == "Following")    dyna_pub.publish(dyna_goal);

                if(flag_dyna == 0 && (error_cur > DYNA_ERR_th || interval_time < SLEEP_TIME2))  //20150426kami error_curの不等号修正
				{
					flag_dyna = 1;
                    interval_time = 0;  //20150426kami intervalのリセット
                    pre_time = ros::Time::now().toSec();
				}
                else if(flag_dyna == 1 && interval_time > SLEEP_TIME2)//(error_cur > DYNA_ERR_th || interval_time > SLEEP_TIME2))//20150426kami error_curの条件を抜いてみるとうまくいった
                {
					flag_dyna_scan++;
					flag_dyna = 0;
					flag_dyna_screen = 0;
                    interval_time = 0;
				}

				if(!flag_dyna_screen)
				{
					ROS_INFO("flag_dyna_scan = %d", flag_dyna_scan);
					flag_dyna_screen = 1;
				}

                if(flag_dyna_scan == 1 || flag_dyna_scan ==2 )  ROS_INFO("cur_time: %f, pre_time: %f, interval: %f", ros::Time::now().toSec(), pre_time, interval_time);//debug
			}
            //else if (dist.front > MinF && flag_dyna_scan >= 3 && (chkdist > CHECK_th || fabs(chkang) > CHECK_ANGLE_th))	// 前方に壁が無く、前回停止位置から離れたとき
            else if (flag_dyna_scan >= 3 && (chkdist > CHECK_th || fabs(chkang) > CHECK_ANGLE_th))	// 20150426kami front条件を廃止
			{
                flag_dyna = 0;
				flag_dyna_scan = 0;
				flag_dyna_screen = 0;
                ROS_INFO("flag_dyna_scan = %d", flag_dyna_scan);
			}
			else
			{
				if(flag_dyna_scan >= 3)	ros::param::set("wall_follower_node/right_or_left", mode);
			}
            /*ここまで曲がり角を見渡す*/
		}
		else if(flag == 5)
        {
            nav_msgs::OccupancyGrid pre_map = map; // 1ループ前のmapデータ
            map = map0; // Movingモード移行時のMapを記憶


            //----------- mapが更新された時の対処 ------------------
            std_msgs::UInt32MultiArray ng_index_new;
            ng_index_new.data.resize(ng_index.data.size());

            for(int i = 0; i < ng_index.data.size(); i++)
            {
                int index = ng_index.data[i];
                double xi, yi, xg, yg;
                double yaw = tf::getYaw(pre_map.info.origin.orientation);
                xi = (i%pre_map.info.width)*pre_map.info.resolution + pre_map.info.resolution/2;
                yi = (i/pre_map.info.width)*pre_map.info.resolution + pre_map.info.resolution/2;
                xg = pre_map.info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
                yg = pre_map.info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);

                double xr, yr;
                xr = xg - map.info.origin.position.x;
                yr = xg - map.info.origin.position.y;
                double yaw_new = tf::getYaw(map.info.origin.orientation);
                double xi_new, yi_new;
                xi_new = xr*cos(yaw_new) + yr*sin(yaw_new);
                xi_new = -xr*sin(yaw_new) + yr*cos(yaw_new);

                int index_new;
                index_new = (unsigned int)(xi_new/map.info.resolution)*map.info.width + (unsigned int)(xi_new/map.info.resolution);

                ng_index_new.data[i] = index_new;
            }
            ng_index = ng_index_new;    // NG indexリストを更新
            //-------------------------------------------------------

            int flag_ng_index = 0;

            if(flag_goal_reset)
            {
                yaw = tf::getYaw(map.info.origin.orientation);

                double xi, yi, xg, yg;
                double xd, yd, thd;
                double dist_cur;
                double dist_unsight = 10000;
                int flag_ng;

                int flag_unsight = 0;

                for(int i=0; i<map.data.size(); i++)
                {
                    flag_ng = 0;
                    if(map.data[i] == UNSIGHT)
                    {
                        for(int j=0; j<ng_index.data.size(); j++)
                        {
                            ROS_INFO("i=%d, ng_index.data[j]=%d", i, ng_index.data[j]);
                            if(i == ng_index.data[j])
                            {
                                flag_ng = 1;
                            }
                        }

                        if(flag_ng == 0)
                        {
                            xi = (i%map.info.width)*map.info.resolution + map.info.resolution/2;
                            yi = (i/map.info.width)*map.info.resolution + map.info.resolution/2;
                            xg = map.info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
                            yg = map.info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);

                            dist_cur = sqrt(pow(curpos.x-xg, 2.0) + pow(curpos.y-yg, 2.0));

                            if(dist_unsight > dist_cur)
                            {
                                dist_unsight = dist_cur;
                                xd = xg;
                                yd = yg;
                                thd = atan2(yd-curpos.y, xd-curpos.x);
                                index = i;
                                flag_unsight = 1;
                                ROS_INFO("dist_cur %f :index %d ",dist_cur,index);
                            }
                        }
                    }
                }


				for(double rj=0; rj < Ld/1000.0; rj += map.info.resolution/RATE)	// NG indexかどうか判定
				{
					unsigned int check_i;
                    double xr, yr;
                    xr = xd - rj*cos(thd) - map.info.origin.position.x;
                    yr = yd - rj*sin(thd) - map.info.origin.position.y;
					check_i = (unsigned int)(yr/map.info.resolution)*map.info.width + (unsigned int)(xr/map.info.resolution);
                    //ROS_INFO("rj=%f, i=%d, %d", rj, check_i, map.data.size());//debug
                    if(check_i < map.data.size() && map.data[check_i] == OBSERVED)
					{
                        ROS_INFO("NG Index!");
						flag_ng_index = 1;
						break;
					}
                }

                if(!flag_unsight)
                {
                    /*
                    if(flag_launch)
                    {
                        system("rosnode kill move_base");
                        flag_launch = 0;
                    }
                    */
                    ROS_INFO("Finish!!");
                    flag = 6;
                    ROS_INFO("flag = %d", flag);
                    state = "Finish";	// Finish!!
                    state_srv.request.state = state;
                    state_client.call(state_srv);
                }
                else if(flag_ng_index)
                {
                    ng_count++;
                    ng_index.data.resize(ng_count);
                    ng_index.data[ng_count-1] = (uint32_t)index;
                    ROS_INFO("Index %d is added to NG index list", index);

                    cells.header.frame_id = "/map";
                    cells.header.stamp = ros::Time::now();
                    cells.cell_width = map.info.resolution;
                    cells.cell_height = map.info.resolution;

                    cells.cells.resize(ng_count);
                    double xi, yi;
                    xi = (index%map.info.width)*map.info.resolution + map.info.resolution/2;
                    yi = (index/map.info.width)*map.info.resolution + map.info.resolution/2;
                    cells.cells[ng_count-1].x = map.info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
                    cells.cells[ng_count-1].y = map.info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);

                    cells_pub.publish(cells);
                    flag_goal_reset = 1;
                }
                else
                {
                    /*
                    if(!flag_launch)
                    {
                        //system("roslaunch rbx1_nav fake_move_base.launch"); //→これだと別プロセスに移行してしまう
                        launch_client.call(movebase_srv);
                        flag_launch = 1;
                    }
                    */
                    //int wait_count = 0;
                    //wait for the action server to come up
                    while(!ac.waitForServer(ros::Duration(5.0))){
                        ROS_INFO("Waiting for the move_base action server to come up");
                        /*
                        wait_count++;
                        if(wait_count > WAIT_COUNT) // move_base再起動
                        {
                            system("rosnode kill move_base");
                            launch_client.call(movebase_srv);
                            wait_count = 0;
                        }
                        */
                    }

                    move_base_msgs::MoveBaseGoal goal;

                    //we'll send a goal to the robot to move 1 meter forward
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();

                    goal_x = xd - Ld/1000.0*cos(thd);
                    goal_y = yd - Ld/1000.0*sin(thd);
                    goal_pose.position.x = goal_x;
                    goal_pose.position.y = goal_y;

                    //goal_pose.orientation.w = 1.0;

                    double direc;
                    if(thd > 0)
                    {
                        direc = M_PI/2;
                    }
                    else
                    {
                        direc = -M_PI/2;
                    }
                    goal_pose.orientation = tf::createQuaternionMsgFromYaw(thd + direc);

                    goal.target_pose.pose = goal_pose;

                    ROS_INFO("Clear costmap");
                    clear_costmap_client.call(movebase_srv);// GoalをセットするまえにCostmapをクリア
                    ROS_INFO("Send goal");
                    ac.sendGoal(goal);
                    flag_goal_reset = 0;
                    chkpos20 = curpos;
                }
            }
            /*
            //---test---------------------------------------------------//
            std::string ac_state, ac_pre_state;
            int ac_flag = 0;
            while(1)
            {
                ac_state = ac.getState().toString();
                if(ac_pre_state != ac_state || ac_flag == 0)
                {
                    ROS_INFO("ac state = %s", ac_state.c_str());
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)	break;
                }
                ac_flag++;
                if(ac_flag > 1000000) ac_flag = 0;
                ac_pre_state = ac_state;
            }
            //---------------------------------------------------------//
            */

            if(flag != 6 && !flag_ng_index)
            {
				/*
                ac_state = ac.getState().toString();
                ROS_INFO("ac state = %s", ac_state.c_str());//debug
                while(ac_state == "PENDING")
                {
                    ROS_INFO("ac state = %s", ac_state.c_str());//debug
                    ac_state = ac.getState().toString();
                    if(ac_state == "ACTIVE")
                    {
                        ROS_INFO("ac state = %s", ac_state.c_str());
                        break;
                    }
                }

                double chkdist2;
                chkdist2 = sqrt(pow(curpos.x - chkpos20.x, 2) + pow(curpos.y - chkpos20.y, 2));
                if(ac_state =="ACTIVE" && detectGoalOverObstacle(goal_x, goal_y) && chkdist2 > R_th)
                {
                    ROS_INFO("Goal reset flag");
                    flag_goal_reset = 1;
                }
				*/
                
                while(!ac.waitForResult(ros::Duration(10.0)))
                {
					double chkdist2;
		            chkdist2 = sqrt(pow(curpos.x - chkpos20.x, 2) + pow(curpos.y - chkpos20.y, 2));
		            if(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE && detectGoalOverObstacle(goal_x, goal_y) && chkdist2 > R_th)
		            {
		                ROS_INFO("Goal reset flag");
		                flag_goal_reset = 1;
						break;
		            }
                }
                
                /*
                geometry_msgs::Pose2D chkpos2, chkpos20;
                double chkdist2;
                int set_count = 0;

                while(!ac.waitForResult(ros::Duration(10.0)))
                {
                    ROS_INFO("Send goal again");
                    ac.sendGoal(goal);
                    chkpos20 = chkpos2;
                    chkpos2 = curpos;
                    chkdist2 = sqrt(pow(chkpos2.x - chkpos20.x, 2) + pow(chkpos2.y - chkpos20.y, 2));
                    if(chkdist2 < CHECK_th)	// 移動量が少ないときにカウント
                    {
                        set_count++;
                    }
                    if(set_count > GOAL_SET_COUNT_MAX)	// 何回か再設定してもたどり着けないとき
                    {
                        if(distR < distL)	// 右の壁に近い
                        {
                            mode = "right";
                        }
                        else	// 左の壁に近い
                        {
                            mode = "left";
                        }
                        ros::param::set("wall_follower_node/right_or_left", mode);	// 少し移動する
                        ROS_INFO("Follow %s walls for about %f seconds", mode.c_str(), 3.0);

                        ros::Duration(3.0).sleep();	//	3秒間走る

                        ros::param::set("wall_follower_node/right_or_left", "stop");
                        ROS_INFO("Stop!", mode.c_str());

                        set_count = 0;	// カウントリセット

                        ROS_INFO("Send goal again");
                        ac.sendGoal(goal);
                    }
                }
                */
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                //if(ac_state == "SUCCEEDED")
                {
                    ROS_INFO("Goal!");
                    flag = 0;
                    ROS_INFO("flag = %d", flag);

                    clear_costmap_client.call(movebase_srv);
                    /*
                    system("rosnode kill move_base");
                    flag_launch = 0;
                    */

                    state = "Following";	// Followingモードへ
                    state_srv.request.state = state;
                    state_client.call(state_srv);
                }
                else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
                //else if(ac_state == "ABORTED")
                {
                    ROS_INFO("Aborted...");
                    ng_count++;
                    ng_index.data.resize(ng_count);
                    ng_index.data[ng_count-1] = (uint32_t)index;
                    ROS_INFO("Index %d is added to NG index list", index);

                    cells.header.frame_id = "/map";
                    cells.header.stamp = ros::Time::now();
                    cells.cell_width = map.info.resolution;
                    cells.cell_height = map.info.resolution;

                    cells.cells.resize(ng_count);
                    double xi, yi;
                    xi = (index%map.info.width)*map.info.resolution + map.info.resolution/2;
                    yi = (index/map.info.width)*map.info.resolution + map.info.resolution/2;
                    cells.cells[ng_count-1].x = map.info.origin.position.x + xi*cos(yaw) - yi*sin(yaw);
                    cells.cells[ng_count-1].y = map.info.origin.position.y + xi*sin(yaw) + yi*cos(yaw);

                    cells_pub.publish(cells);
                    flag_goal_reset = 1;
                }
                /*
                else
                {
                    ROS_INFO("The base failed to move for some reason");
                }
                */
            }
		}
		else if(flag == 6)
		{
			cells_pub.publish(cells);
			//finish!
		}
		
		ros::spinOnce();

		loop_rate.sleep();

        interval_time = ros::Time::now().toSec() - pre_time;
	}
}
