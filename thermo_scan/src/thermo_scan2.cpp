//=================================================//
/*! @file
 @brief カメラユニットの首振り処理のノード
 @author D.Furutani
 @date 2013/10/25
 @attention Dynamixcelを用いて首振りを行う

 @TODO プログラムの整理と待機時間の調整
*/
//=================================================//

using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include <std_msgs/Int32.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "dynamixel_msgs/JointState.h"
#include "set_state/pubState.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

#include <sstream>

// グローバル変数設定
bool blob_detect = false;	// 熱源発見の有無
bool Init_flag = true;		// ダイナミクセルの初期位置への移動フラグ
bool Scanning_flag = false;		// ダイナミクセルの初期位置への移動フラグ
double camera_yaw;	// カメラ角度 [rad]

int Scan_state = 0;	// カメラユニットの首振り状態
			// Scan_state=0:通常状態
			// Scan_state=1:強化探査状態
			// Scan_state=2:待機状態
			// Scan_state=3:通常状態でVictim発見
			// Scan_state=4:強化探査状態でVictim発見
			// Scan_state=5:Stop_temp状態
			// Scan_state=6:Stop_tempの折り返し条件を満たした状態
std::string state = "Waiting";	// ロボットの状態
		// state=0:Normal
		// state=1:Tracking
		// state=2:Rotating
		// state=3:Closing
		// state=4:Scanning
		// state=5:GetInfo
		// state=6:Leaving
		// state=7:Stop_temp
		// state=8:Waiting

//首ふり関連☆
//CAM_2は強化探査時のカメラユニットの動き
#define MAX_YAW_ANGLE_CAM	(M_PI*90)/180	// センサユニットヨー角の最大角度(rad)
#define MIN_YAW_ANGLE_CAM	-MAX_YAW_ANGLE_CAM	// センサユニットヨー角の最少角度(rad)
#define STEP_COUNT			5				// ステップ数 通常探索
#define STEP_COUNT2			(STEP_COUNT*2)				// ステップ数 強化探索
#define ONE_STEP_ANGLE_CAM	((MAX_YAW_ANGLE_CAM - MIN_YAW_ANGLE_CAM)/2/STEP_COUNT)	// １ステップ(rad) 通常探索
#define ONE_STEP_ANGLE_CAM2	((MAX_YAW_ANGLE_CAM - MIN_YAW_ANGLE_CAM)/2/STEP_COUNT2)	// １ステップ(rad) 強化探索
#define MOTOR_SPEED		2.0		// speed of motor (normal)
#define MOTOR_SPEED2	2.0//1.0		// speed of motor (scanning)
#define SLEEP_TIME	0.4	// １ステップの時間 [sec]

#define	OCCUPIED	100		// 黒
#define OBSERVED	127		// 緑（rviz上での表示色）
#define UNSIGHT		-2		// 黄

#define R_STEP		10

#define VICTIM_DIST_MIN		0.5	// [m]
#define VICTIM_ANGLE_MIN	0.5	// [rad]

#define error_limit             (M_PI*3)/180    //  首振りDynamixel初期位置出しエラー判定値(rad)   
double error_cur = error_limit*2;

std_msgs::Float64 tilt_controller_command_msg; 	
geometry_msgs::PoseArray victims0;
nav_msgs::OccupancyGrid map0;
sensor_msgs::LaserScan scan0;
double range= 0.6;
int victim_count;	// 発見したvictimの数
bool flag_victim_sub;
bool flag_leaving;
bool init_scanning_flag;
bool detect_flag;

//#define WAGGING_SPEED_CAM	100	// １ステップの所要時間の1/4の値[ms]
//#define WAGGING_SPEED_CAM2	100	// １ステップの所要時間の1/4の値[ms]
//#define PROCESSING_TIME	600	// 画像処理の待機時間
//#define SCAN_SPEED_PI		200	// 熱画像のスキャン間隔[ms]

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{                      
	range = msg->range;
}

void DynaCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
	camera_yaw = msg->current_pos;
	error_cur = fabs(msg->error);
    //ROS_INFO("current error: %f [rad]", error_cur);	//debug
}

//カメラユニットの首振り制御のクラス=========================================//
class ThermalScanning
{
private:
	int pm;			// pm=1:左回り、pm=-1:右回り
	double start_angle;	// スタート角度
	double end_angle;		// 折り返し角度
	double Dyna_angle;	// ダイナミクセルの角度
	int switch_num;	// stop_tempの時の折り返し回数
	dynamixel_controllers::SetSpeed srv;
	set_state::pubState state_srv;
	int scan_count;	// 強化探索スキャン回数カウント

public:
	int cw;

	ThermalScanning()
	{
		//cw = false;
	    cw=-1;
		switch_num = 0;
		scan_count = 0;
		dyna_client = n.serviceClient<dynamixel_controllers::SetSpeed>("tilt_controller/set_speed");
		state_client = n.serviceClient<set_state::pubState>("pubState");
	}

	void InitScan(void);
	void TurnScan(void);
	bool SuccessiveScan(void);
    void set_scan_count(int count){ scan_count = count;}

protected:
	ros::NodeHandle n;
	ros::ServiceClient dyna_client;
	ros::ServiceClient state_client;
};

//=============================================================
//名前：TurnScan
//説明：首振りの方向折り返し
//=============================================================
void ThermalScanning::TurnScan(void)
{
	//std_msgs::Float64 tilt_controller_command_msg; 


    std::string mode;
    ros::param::get("wall_follower_node/right_or_left", mode);

    if(mode == "right")
    {
        if(cw == -1)	// 左回りの場合
        {
            start_angle = MIN_YAW_ANGLE_CAM;
            end_angle   = MAX_YAW_ANGLE_CAM;
            pm = 1;
        }else if(cw == 1){	// 右回りの場合
            start_angle = MAX_YAW_ANGLE_CAM;
            end_angle   = MIN_YAW_ANGLE_CAM;
            pm = -1;
        }
    }
    else
    {
        if(cw == -1)	// 左回りの場合
        {
            start_angle = MAX_YAW_ANGLE_CAM;
            end_angle   = MIN_YAW_ANGLE_CAM;
            pm = -1;
        }else if(cw == 1){	// 右回りの場合
            start_angle = MIN_YAW_ANGLE_CAM;
            end_angle   = MAX_YAW_ANGLE_CAM;
            pm = 1;
        }
    }

    /*
    if(cw == -1)	// 左回りの場合
    {
        start_angle = MIN_YAW_ANGLE_CAM;
        end_angle   = MAX_YAW_ANGLE_CAM;
        pm = 1;
    }else if(cw == 1){	// 右回りの場合
        start_angle = MAX_YAW_ANGLE_CAM;
        end_angle   = MIN_YAW_ANGLE_CAM;
        pm = -1;
    }
    */

	// ダイナミクセルのスピードの設定
	if(Scan_state == 0 || Scan_state == 5 || Scan_state == 6)	// 通常探索
	{ 
		srv.request.speed = MOTOR_SPEED;
	}
	else if(Scan_state == 1)	// 強化探索
	{
		srv.request.speed = MOTOR_SPEED2;
	}	
	dyna_client.call(srv);
		
	// ダイナミクセルの角度を初期位置に移動する
	// TODO ダイナミクセルの角度をDyna_angleに変更する処理を入れる
    //Dyna_angle = Init_flag ? 0.0 : start_angle;
    Dyna_angle = start_angle;

	tilt_controller_command_msg.data = Dyna_angle;
}

//=============================================================
//名前：InitScan
//説明：首振りの初期位置合わせ
//=============================================================
void ThermalScanning::InitScan(void)
{
	srv.request.speed = MOTOR_SPEED;	
	dyna_client.call(srv);
		
	// ダイナミクセルの角度を初期位置に移動する
	Dyna_angle = 0.0;
	tilt_controller_command_msg.data = Dyna_angle;
}

//=============================================================
//名前：SuccessiveScan
//説明：熱源探知のための首振り
//=============================================================
bool ThermalScanning::SuccessiveScan(void)
{
	// ダイナミクセルのスピードの設定
	if(Scan_state == 0 || Scan_state == 5 || Scan_state == 6)	// 通常探索
	{ 
		srv.request.speed = MOTOR_SPEED;
	}
	else if(Scan_state == 1)	// 強化探索
	{
		srv.request.speed = MOTOR_SPEED2;
	}	
	dyna_client.call(srv);

	// 強化探索で1周スキャンしてみつからなかったらNormalへ戻る
	if(Scan_state == 1 && state == "Scanning")
	{
		if(scan_count < STEP_COUNT2*4)
		{
			scan_count++;
		}
		else
		{
			scan_count = 0;
			Scan_state = 0;
			//state = "Normal";
			state = "Following";
			state_srv.request.state = state;
			state_client.call(state_srv);
			srv.request.speed = MOTOR_SPEED;
			dyna_client.call(srv);

            init_scanning_flag = false;
		}
	}


	// 首振りの処理
	if(pm*Dyna_angle < pm*end_angle - ((Scan_state == 1) ? ONE_STEP_ANGLE_CAM2 : ONE_STEP_ANGLE_CAM)) // 首振りがend_angle以下なら首振り
	{
		if(Scan_state == 1) // 強化探査状態
		{
			// TODO ダイナミクセルの角度をDyna_angleに変更する処理を入れる
			Dyna_angle = Dyna_angle  + (pm*ONE_STEP_ANGLE_CAM2);	// ONE_STEP_ANGLE_CAM2だけ首振り(強化探索)
			tilt_controller_command_msg.data = Dyna_angle;
		}
        else if(Scan_state != 4)
		{
			// TODO ダイナミクセルの角度をDyna_angleに変更する処理を入れる
			Dyna_angle = Dyna_angle + (pm*ONE_STEP_ANGLE_CAM);	// ONE_STEP_ANGLE_CAMだけ首振り
			tilt_controller_command_msg.data = Dyna_angle;
		}
	}
    else// if(pm*Dyna_angle >= pm*end_angle - ((Scan_state == 1) ? ONE_STEP_ANGLE_CAM2 : ONE_STEP_ANGLE_CAM))
	{
		ROS_INFO("4");

		//cw = !cw;
		cw = -cw;
		TurnScan();	// 首振り方向を反転させて初期位置へ移動

		ROS_INFO("5");

		// Stop_temp状態の場合の首振り回数カウント
		if (Scan_state == 5)
		{
			switch_num = switch_num + 1;
			if (switch_num >= 2)
			{
				switch_num = 0;
				return true;	// Stop_temp終了ならtrueを返す
			}
		}
		
	return false;
	}
}

//=============================================
//@name distinguish_victim
//@brief 新たに見つけたvictimが発見済みのvictimと近すぎないかを判定
//@data 2014/03/24
//@attention
//=============================================
bool distinguish_victim(void)
{
	geometry_msgs::PoseArray victims;

	double xr, yr, rot;
	double target_x, target_y, target_angle;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	nav_msgs::OccupancyGrid map;
	try
	{
		//listener.lookupTransform("/map", "/thermocam_link", ros::Time(0), transform);
		ros::Time now = ros::Time(0);
		listener.waitForTransform("/map", "/thermocam_link", now, ros::Duration(3.0));
		listener.lookupTransform("/map", "/thermocam_link", now, transform);
	}
    catch (tf::TransformException ex)
	{
  		ROS_ERROR("%s",ex.what());
	}
	rot = getYaw(transform.getRotation());

	// 新たに発見したvictimのposeの計算
    target_angle = rot + /*camera_yaw +*/ M_PI;
	if(target_angle > M_PI)	target_angle -= 2*M_PI;
	else if(target_angle < -M_PI)	target_angle += 2*M_PI;

	map = map0;
    int flag_no_map = 1;
	for(double rj=0; rj <= range; rj += range/R_STEP)
	{
		//ROS_INFO("rj=%f", rj);//debug
		unsigned int index;
        xr = rj * cos(/*camera_yaw +*/ rot) + transform.getOrigin().x() - map.info.origin.position.x;
        yr = rj * sin(/*camera_yaw +*/ + rot) + transform.getOrigin().y() - map.info.origin.position.y;
		index = (unsigned int)(yr/map.info.resolution)*map.info.width + (unsigned int)(xr/map.info.resolution);
		if(index < map.data.size() && map.data[index] == OCCUPIED)
		{
            target_x = xr + map.info.origin.position.x;
            target_y = yr + map.info.origin.position.y;
            flag_no_map = 0;
			break;
		}
	}

    if(flag_no_map)
    {
        sensor_msgs::LaserScan scan;
        scan = scan0;
        int index;
        index = (int)((camera_yaw - scan.angle_min)/scan.angle_increment);
        //ROS_INFO("camera_yaw = %f, min = %f, step = %f, index = %d", camera_yaw, scan.angle_min, scan.angle_increment, index);//debug
        if(index < scan.ranges.size())
        {
            double rd = scan.ranges[index];
            target_x = rd * cos(rot) + transform.getOrigin().x() - map.info.origin.position.x;
            target_y = rd * sin(rot) + transform.getOrigin().y() - map.info.origin.position.y;
            //ROS_INFO("detect victim position by LRF");//debug
        }
        else
        {
            ROS_ERROR("Index ERROR!");//
        }
    }
	
	double victim_angle, victim_dist;
	bool new_victim_or_not = true;
	// 発見済みのvictimと十分離れているか判定
	victims = victims0;
    ROS_INFO("Victim num = %d", (int)victims.poses.size());//debug
    for(int i = 0; i < victims.poses.size(); i++)
	{
		victim_dist = sqrt(pow(victims.poses[i].position.x - target_x, 2.0) + pow(victims.poses[i].position.y - target_y, 2.0));
		victim_angle = fabs(tf::getYaw(victims.poses[i].orientation) - target_angle);
        ROS_INFO("vic_dist = %f, vic_ang = %f", victim_dist, victim_angle);//debug
		if(victim_dist < VICTIM_DIST_MIN && victim_angle < VICTIM_ANGLE_MIN)	// 発見済みのvictimのなかで位置or角度が近いものが1つでもある場合
		{
			new_victim_or_not = false;
            ROS_INFO("New victim is close to previous victims");
			break;
		}
	}
	
	return new_victim_or_not;
}

//=============================================
//@name Blob_detect_Callback
//@brief msgで受け取った熱源の有無を変数に格納する
//@data 2013/10/25
//@attention
//=============================================	
void Blobcallback(const std_msgs::Bool::ConstPtr& blobmsg)
{ 
	ros::NodeHandle n;
	ros::ServiceClient state_client = n.serviceClient<set_state::pubState>("pubState");
	ros::ServiceClient dyna_client = n.serviceClient<dynamixel_controllers::SetSpeed>("tilt_controller/set_speed");
		
	dynamixel_controllers::SetSpeed dyna_srv;
	set_state::pubState state_srv;

	bool search_flag = false;
	bool new_victim_flag = true;
	
    if(blobmsg->data)
    {
        if(state == "Normal" || (state == "Scanning" && Scanning_flag == false) || state == "Stop_temp" || state == "Following")
        {
            if(victim_count > 0)	// 2個目以降のvictimのとき
            {
                // 発見済みのvictimと近すぎないかを判定
                search_flag = distinguish_victim();
		new_victim_flag = search_flag;
            }
            else	// 最初のvictimのとき
            {
                search_flag = true;
            }
        }
    }
  
	// 通常、強化探査(初期位置へ向かう間を除く)、Stop_temp状態でVictimを検出したら検出状態へ移行
    if (search_flag == true)
	{  
		if (Scan_state == 0 ||Scan_state == 5 || Scan_state == 6)
		{
			Scan_state = 3;
			ROS_INFO("Victim Found!(Normal)");
        	state = "Tracking";
			state_srv.request.state = state;
			state_client.call(state_srv);
			//dyna_srv.request.speed = 0.0;
			//dyna_client.call(dyna_srv);
		}
        else if (Scan_state == 1 && detect_flag)
		{
			Scan_state = 4;
			ROS_INFO("Victim Found!(Scanning)");
			state = "GetInfo";

            tilt_controller_command_msg.data = camera_yaw;  // GetInfoになったら現在位置を目標角度にして停止させる

			state_srv.request.state = state;
			state_client.call(state_srv);



            //dyna_srv.request.speed = 0.0;   // GetInfoになったらDynamixel止める
            //dyna_client.call(dyna_srv);
		}
	}
	//20150427kami 
	if (new_victim_flag == false)
	{
		Scan_state = 0;
		ROS_INFO("Victim Found second time!(ignore)");
        	state = "Leaving";
		state_srv.request.state = state;
		state_client.call(state_srv);
		new_victim_flag = true;
	}
}

//=============================================
//@name state_Callback
//@brief msgで受け取ったロボットの状態を変数に格納する
//@data 2013/10/25
//@attention
//=============================================	
void state_Callback(const std_msgs::String::ConstPtr& state_msg)
{
	// 取得したstate_msgを変数に格納する
	state = state_msg -> data;

	// 強化探査、待機、Stop_temp(終了)状態から通常状態への移行
    if ((Scan_state == 1 || Scan_state == 2 || Scan_state == 4 || Scan_state == 6) && (state == "Normal" || state == "Following"))
	{
		Scan_state = 0;
		Init_flag = true;
	}
	// 待機もしくはVictim検出(通常)状態から強化探査状態への移行
	else if ((Scan_state == 2 || Scan_state == 3) && state == "Scanning")	
	{
		Scan_state = 1;
		Init_flag = true;
        detect_flag = false;
	}
	// 待機状態への移行
	else if (state == "Tracking" || state == "Rotating" || state == "Closing" || state == "GetInfo" || state == "Leaving")
	{
		Scan_state = 2;
	}
}

//=============================================
//@name victims_Callback
//@brief 発見済みのvictimのposeを取得
//@data 2014/03/24
//@attention
//=============================================	
void victims_Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    victims0 = *msg;
    flag_victim_sub = true;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //ROS_INFO("scanCallback");//debug
    scan0 = *msg;
}

//=============================================
//@name check_and_go
//@brief GUIからCheck&Goの合図を受けるサービス
//@data 2014/03/24
//@attention
//=============================================	
bool check_and_go(std_srvs::Empty::Request  &req,
		 	      std_srvs::Empty::Response &res)
{
    ros::NodeHandle n;
	ros::ServiceClient victim_client = n.serviceClient<std_srvs::Empty>("addVictim");
    ros::Publisher Dyna_pub = n.advertise<std_msgs::Float64>("tilt_controller/command", 1000);

    std_srvs::Empty victim_srv;
    victim_client.call(victim_srv);		// 発見したvictimの位置を記録するサービスの呼び出し

    /*
    set_state::pubState state_srv;
    state = "Following";				// Followingモードに戻す
    state_srv.request.state = state;
    state_client.call(state_srv);
    */

    std::string mode;
    ros::param::get("wall_follower_node/right_or_left", mode);
    if(mode == "left")	//左手
    {
        ROS_INFO("left");//debug
        tilt_controller_command_msg.data = M_PI/2;
    }
    else if(mode == "right")	//右手
    {
        tilt_controller_command_msg.data = -M_PI/2;
    }
    else
    {
        tilt_controller_command_msg.data = 0;
    }

	victim_count++;	// victim数をカウント
    Dyna_pub.publish(tilt_controller_command_msg);
    ros::Duration(2.0).sleep(); // 2秒スリープ

    flag_leaving = true;
}

//=============================================
//@name main
//@brief メインループ
//@data 2013/10/25
//@attention
//=============================================	
int main(int argc, char **argv)
{
	// ROSの初期設定
	ros::init(argc, argv, "thermo_scan");

	// 各種設定
	ThermalScanning ts;	// 首振りクラスの定義

	ROS_INFO("1");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);	// ループの待機時間(Hz)

	ros::Publisher Scan_state_pub = n.advertise<std_msgs::Int16>("Scan_state", 100);
	ros::Subscriber dyna_sub = n.subscribe("tilt_controller/state", 1000, DynaCallback);
	std_msgs::Int16 Scan_state_msg;
	ros::Publisher Dyna_pub = n.advertise<std_msgs::Float64>("tilt_controller/command", 1000);
    ros::Subscriber sub_victims = n.subscribe("victims", 100, victims_Callback);
    ros::Subscriber Blob_detect_sub = n.subscribe("optris/ThermoBlob", 10, Blobcallback);
	ros::Subscriber sub_range = n.subscribe("range", 1000, rangeCallback);
    ros::Subscriber sub_scan = n.subscribe("scan", 40, scanCallback);

	ros::Subscriber State_sub = n.subscribe("state", 100, state_Callback);

	ros::ServiceServer service = n.advertiseService("CheckAndGo", check_and_go);

	double cur_time, interval_time;

	int count=0;
    bool initial_pos_flag;

	// ダイナミクセルの初期化
	if (Init_flag == true)
	{
		Init_flag == false;
        ts.cw = -1;
        ts.InitScan();
        initial_pos_flag = true;
	}

	cur_time = ros::Time::now().toSec();
	victim_count = 0;
    flag_victim_sub = false;
    flag_leaving = false;
    init_scanning_flag = false;

	// メインループ
	while (ros::ok())
	{
		// コールバックを呼び出してVictim検出結果とロボットの状態を取得
		ros::spinOnce();

		// Dynamixelが目標値近傍へ収束するか、一定時間経つまで待つ
		if(error_cur < error_limit && interval_time > SLEEP_TIME)
		{
			cur_time = ros::Time::now().toSec();
	
			Scanning_flag = false;

			if (Init_flag == true && Scan_state == 1)
            {
				ts.TurnScan();
				Init_flag = false;
				Scanning_flag = true;
                init_scanning_flag = true;
			}

            if(init_scanning_flag)
            {
                detect_flag = true;
            }

			// スキャン状態が通常、強化探査、Stop_tempなら首振り
            //if (state != "GetInfo" && state != "Waiting" && state != "Following" && state != "Moving" && state != "Leaving" && state != "Teleope" && (Scan_state == 0 || Scan_state == 1 || Scan_state == 5 || Scan_state == 6))
            if(state == "Normal" || state == "Stop_temp" || (state == "Scanning" && init_scanning_flag))
            {
				if(ts.SuccessiveScan() == true)
				{
					Scan_state == 6;
					count++;
				}
			}
            else if(state == "GetInfo" && flag_leaving)
            {
                ros::ServiceClient state_client = n.serviceClient<set_state::pubState>("pubState");
                set_state::pubState state_srv;
                std::string state = "Leaving";//"Following";				//20150411kamimura Leavingにするのが正しいようです// Followingモードに戻す
                state_srv.request.state = state;
                state_client.call(state_srv);
                flag_leaving = false;
                ts.set_scan_count(0);   // Reset scan_count
                ts.cw = -1;
            }

            if(initial_pos_flag)
            {
                initial_pos_flag = false;
            }
		}

        if(state == "Normal" || state == "Stop_temp" || (state == "Scanning" && init_scanning_flag) || initial_pos_flag) 	Dyna_pub.publish(tilt_controller_command_msg);

        //ROS_INFO("Scan_state: %d", Scan_state);//debug
        //ROS_INFO("Victim num = %d", (int)victims0.poses.size());//debug

		// カメラユニットの首振り状態の配信
		Scan_state_msg.data = Scan_state;
		Scan_state_pub.publish(Scan_state_msg);             

		// 次のデータを送る前にloop_rateだけ待機
		loop_rate.sleep();

		interval_time = ros::Time::now().toSec() - cur_time;
	}
	return 0;
}
