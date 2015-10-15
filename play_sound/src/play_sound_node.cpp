#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

std::string state;
bool flag;

void state_callback(const std_msgs::String::ConstPtr& msg)
{
    state = msg->data;
    flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "play_sound_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("state", 10, state_callback);

    ros::Rate loop_rate(10);

    flag = false;

    std::string filename;//

    if(ros::param::has("play_sound_node/sound_file"))
    {
        ros::param::get("play_sound_node/sound_file", filename);
    }
    else
    {
        filename = "~/catkin_ws/sound/victim.wav";
    }

    std::string command;
    command = "aplay " + filename;

    while (ros::ok())
    {
        if(flag && state == "GetInfo")
        {
            int result = system(command.c_str());
            ROS_INFO("%d", result);
            if(result == 2)
            {
                return 0;
            }
        }

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}
