#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
#include "wall_follower/walls.h"

//各種定義
#define	ROAD_WIDTH	850		//!< 道の幅[m]//900mm...(JapanOpen),1200mm(WorldCup)

//☆ロボットの筐体，自律走行のパラメータ
#define	ROBOT_WIDTH 350				/*横幅*/

ros::Publisher walls_pub;//

//====================================================//
/*!
*	@class 距離データの定義
*/
//====================================================//
class Distance{
public:
	Distance(){
		Clear();
	}
	~Distance(){
	}
	void Clear(void){//適当に大きな値で初期化
                frontL = 10000;
                frontR = 10000;
		left   = 10000;
		right  = 10000;
		left2  = 10000;
		right2 = 10000;

                frontWidthR = 10000;
                frontWidthL = 10000;
	}

public:
        double frontL;	//前(右) [mm]
        double frontR;	//前（左） [mm]
	double left;	//左前 [mm]
	double right;	//右前 [mm]
	double left2;	//左後 [mm]
	double right2;	//右後 [mm]

        double frontWidthR; //前（左）と右前の間の距離
        double frontWidthL; //前（右）と左前の間の距離
};

Distance dist, dist2;

//-----------------------------------------------------
// URGデータのコールバック関数
// 計測距離データをロボット座標での3次元点データに変換したものから
// ロボット周囲の障害物までの距離を計算する
//-----------------------------------------------------
void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	double urg_x, urg_y,frontLx,frontLy,frontRx,frontRy,rightx,righty,leftx,lefty;
	wall_follower::walls walls;

	dist.Clear();
	
	for(int i = 0; i < (int)msg->points.size(); i++){
		urg_x = msg->points[i].x * 1000;	// [m] から [mm] に変換
		urg_y = msg->points[i].y * 1000;	// [m] から [mm] に変換
		
		if(urg_x >= 100){
                    //前（左）
                    if((urg_y > 0) && (urg_y < +(ROBOT_WIDTH/2+10)))
                    {
                        if(urg_x < dist.frontL)
                        {
                                dist.frontL = (int)urg_x;
                                frontLx = (int)urg_x;
                                frontLy = (int)urg_y;
                        }
                    }

                    //前（右）
                    if((-urg_y > 0) && (-urg_y < +(ROBOT_WIDTH/2+10)))
                    {
                        if(urg_x < dist.frontR)
                        {
                                dist.frontR = (int)urg_x;
                                frontRx = (int)urg_x;
                                frontRy = (int)urg_y;
                        }
                    }

			if(urg_x <= ROAD_WIDTH/2){
				//右前の方向
				if ((-urg_y > 21) && (-urg_y < dist.right) ) 
				{
					dist.right = -(int)urg_y;
                                        rightx = (int)urg_x;
                                        righty = (int)urg_y;
				}
				//左前の方向
				if( (urg_y > 21) && (urg_y < dist.left)) 
				{
					dist.left = (int)urg_y;
                                        leftx = (int)urg_x;
                                        lefty = (int)urg_y;
				}
			}
		}
		else if(urg_x < 100 && urg_x >= -ROBOT_WIDTH/2+100)//後方のデータ
		{
			//右後ろの方向
			if ( (-urg_y > 21) && (-urg_y < dist.right2)) 
			{
				dist.right2 = -(int)urg_y;
			}
			//左後ろの方向
			if ( (urg_y > 21) && (urg_y < dist.left2)) 
			{
				dist.left2 = (int)urg_y;
			}
		}
	}

        dist.frontWidthR = sqrt(pow(frontLx-rightx,2.0)+pow(frontLy-righty,2.0));
        dist.frontWidthL = sqrt(pow(frontRx-leftx,2.0)+pow(frontRy-lefty,2.0));
    //ROS_INFO("FR=%4.0f FL=%4.0f L=%4.0f  L2=%4.0f  R=%4.0f  R2=%4.0f", dist.frontR, dist.frontL, dist.left, dist.left2, dist.right, dist.right2);

    // 水平LRFとジンバルLRFのうち小さい方を採用
    walls.frontR = (dist.frontR < dist2.frontR) ? dist.frontR : dist2.frontR;
    walls.frontL = (dist.frontL < dist2.frontL) ? dist.frontL : dist2.frontL;
    walls.left = (dist.left < dist2.left) ? dist.left : dist2.left;
    walls.left2 = (dist.left2 < dist2.left2) ? dist.left2 : dist2.left2;
    walls.right = (dist.right < dist2.right) ? dist.right : dist2.right;
    walls.right2 = (dist.right2 < dist2.right2) ? dist.right2 : dist2.right2;

    walls.frontWidthR = dist.frontWidthR;
    walls.frontWidthL = dist.frontWidthL;

    ROS_INFO("FR=%4.0f  FL=%4.0f  L=%4.0f  L2=%4.0f  R=%4.0f  R2=%4.0f Width=%4.0f", walls.frontR,  walls.frontL, walls.left, walls.left2, walls.right, walls.right2, walls.frontWidthR);


	walls_pub.publish(walls);
}

// 3Dマップから得たwallsデータ
void wallsCallback(const wall_follower::walls::ConstPtr& msg)
{
    dist2.Clear();

    dist2.frontR = msg->frontR;
    dist2.frontL = msg->frontL;
    dist2.left = msg->left;
    dist2.left2 = msg->left2;
    dist2.right = msg->right;
    dist2.right2 = msg->right2;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detect_near_wall_node3");

	ros::NodeHandle n;

    ros::Subscriber urg_sub = n.subscribe("cloud", 1000, cloudCallback);
    ros::Subscriber walls_sub = n.subscribe("gimbal_walls", 1000, wallsCallback);
	walls_pub = n.advertise<wall_follower::walls>("walls", 50);

	ros::spin();
}
