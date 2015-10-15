#include <ros/ros.h>
#include <kamui_control/base_velocity.h>
#include <kamui_control/flipper_velocity.h>
#include <set_state/pubState.h>
#include <sensor_msgs/Joy.h>
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <string>
#include <cmath>

#define CONTROL_F 10	// 制御周期 [Hz]
#define DYNA_ANGLE_STEP 0.1
#define PI 3.14159265358979
#define MOTOR_SPEED		2.0		// speed of motor

int vel, avel;
double frvel, flvel;
double cur_dyna_pos;
ros::Publisher dyna_pub;
std::string state;

void DynaCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
    cur_dyna_pos = msg->current_pos;
}

void stateCallback(const std_msgs::String::ConstPtr& state_msg)
{
    state = state_msg -> data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    set_state::pubState state_srv;
    ros::NodeHandle n;
    ros::ServiceClient state_client = n.serviceClient<set_state::pubState>("pubState");
    if(joy->buttons[8])   // STARTボタン
    {
        state_srv.request.state = "Teleope";
        state_client.call(state_srv);
    }
    else if(joy->buttons[9])   // SELECTボタン
    {
        state_srv.request.state = "Following";
        state_client.call(state_srv);
    }
    else if(joy->buttons[2])   // ×ボタン
    {
        state_srv.request.state = "Waiting";
        state_client.call(state_srv);
    }

    if(state == "Teleope")
    {
        //ROS_INFO("%f", joy->axes[4]);//debug
        // カメラの向き
        std_msgs::Float64 dyna_goal;
        if(joy->axes[4] != 0)   // 十字ボタン左右
        {
            double dyna_pos_d;
            dyna_pos_d = cur_dyna_pos + (double)(joy->axes[4])*DYNA_ANGLE_STEP*2;
            dyna_goal.data = dyna_pos_d;
            if(fabs(dyna_pos_d) < PI/2.0)  dyna_pub.publish(dyna_goal);
        }
        else if(joy->axes[5] != 0)   // 十字ボタン上下（ゆっくり）
        {
            double dyna_pos_d;
            dyna_pos_d = cur_dyna_pos - (double)(joy->axes[5])*DYNA_ANGLE_STEP;
            dyna_goal.data = dyna_pos_d;
            if(fabs(dyna_pos_d) < PI/2.0)  dyna_pub.publish(dyna_goal);
        }
        else if(joy->buttons[0])   // △ボタン
        {
            dyna_goal.data = 0.0;
            dyna_pub.publish(dyna_goal);
        }
        else if(joy->buttons[1])   // ○ボタン
        {
            dyna_goal.data = -PI/2.0;
            dyna_pub.publish(dyna_goal);
        }
        else if(joy->buttons[3])   // □ボタン
        {
            dyna_goal.data = PI/2.0;
            dyna_pub.publish(dyna_goal);
        }

        // クローラー速度
        /*
        if(joy->axes[3] < 0.2) vel = 0;
        else vel = (int)(joy->axes[3] * 40.0);
        if(joy->axes[0] < 0.2) avel = 0;
        else avel = (int)(joy->axes[0] * 30.0);
        */
        /*
        double axis1, axis2;
        axis1 = (fabs(joy->axes[1]) < 0.1) ? 0 : joy->axes[1];
        axis2 = (fabs(joy->axes[2]) < 0.1) ? 0 : joy->axes[2];
        vel = (int)(axis1 * 40.0);
        avel = (int)(axis2 * 30.0);
        */
        vel = (int)(joy->axes[1] * 40.0);
        avel = (int)(joy->axes[2] * 30.0);

        // フリッパー速度（今はフィードフォワード、段差乗り越えるならフィードバックにしないといけない）
        if(joy->buttons[6]) flvel = 8.0;
        else if(joy->buttons[4]) flvel = -8.0;
        else flvel = 0.0;
        if(joy->buttons[7]) frvel = 8.0;
        else if(joy->buttons[5]) frvel = -8.0;
        else frvel = 0.0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kamui_teleop_node");
    ros::NodeHandle n;
    kamui_control::base_velocity bvel;
    kamui_control::flipper_velocity fvel;

    ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);
    ros::Publisher pub_vel = n.advertise<kamui_control::base_velocity>("Velocity_tele", 50);
    ros::Publisher pub_fvel = n.advertise<kamui_control::flipper_velocity>("Flipper_Velocity", 50);
    ros::Subscriber dyna_sub = n.subscribe("tilt_controller/state", 100, DynaCallback);
    ros::Subscriber state_sub = n.subscribe("state", 100, stateCallback);
    dyna_pub = n.advertise<std_msgs::Float64>("tilt_controller/command", 1);
    ros::ServiceClient dyna_client = n.serviceClient<dynamixel_controllers::SetSpeed>("tilt_controller/set_speed");


    ros::Rate loop_rate(20);

    vel = 0;
    avel = 0;
    frvel = 0;
    flvel = 0;

    dynamixel_controllers::SetSpeed srv;
    srv.request.speed = MOTOR_SPEED;
    dyna_client.call(srv);

    while(ros::ok())
    {
        bvel.linear = vel;
        bvel.angular = avel;
        fvel.fvelleft = flvel;
        fvel.fvelright = frvel;

        pub_vel.publish(bvel);
        pub_fvel.publish(fvel);
        ROS_INFO("bvel linear: %d,  angular: %d", (int)bvel.linear, (int)bvel.angular);//debug

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
