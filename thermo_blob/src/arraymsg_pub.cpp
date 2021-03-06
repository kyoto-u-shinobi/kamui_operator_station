#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv)
{
	// ROSの初期設定
	ros::init(argc, argv, "arrayPublisher");
	ros::NodeHandle n;

	// データ配信用のパブリッシャーの定義
	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("Thermo", 100);

	int k = 0;

	// メインループ
	while (ros::ok())
	{
		// 配信する配列のmsgを定義
		std_msgs::Float32MultiArray thermo_msg;

		// msg配列の初期化
		thermo_msg.data.clear();

		// msg配列にデータを入れるループ
		for (int j = 0; j < 80; j++){
			for (int i = 0; i < 60; i++) // 多次元配列用に追加
			{
				// 値をmsg配列に入力する
				if(k < 2){
					if(i < j){
						if(k == 0){
							thermo_msg.data.push_back(40);
						}else{
							thermo_msg.data.push_back(0);
						}
					}else{
						if(k == 0){
							thermo_msg.data.push_back(0);
						}else{
							thermo_msg.data.push_back(40);
						}
					}
				}else{
					if(i < 10 && j < 10){
						thermo_msg.data.push_back(40);
					}else{
						thermo_msg.data.push_back(0);
					}
				}
			}
		}

		if(k < 2){
			k = k + 1;
		}else{
			k = 0;
		}

		// msg配列を配信する
		pub.publish(thermo_msg);

		// ROS_INFOで配信していることを伝える
		ROS_INFO("I published something!");
		ros::spinOnce();

		// 次のデータを送る前に少し待機
		sleep(2);
	}
	return 0;
}
