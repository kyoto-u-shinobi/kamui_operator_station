//=================================================//
/*! @file
 @brief MAP画像保存クライアント
 @author K.Muro
 @date 2014/05/01
 @attention MAP画像を保存するだけ

 @TODO
*/
//=================================================//

#include <ros/ros.h>
#include <std_srvs/Empty.h>

//=============================================
//@name main
//@brief メインループ
//@data 2013/11/15
//@attention
//=============================================
int main(int argc, char** argv)
{

  // ROSの初期設定
  ros::init(argc, argv, "save_map_client_node");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(0.1);	// ループの待機時間(Hz)

  // srvのクライアントの立ち上げ
  ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("SaveMap");
  std_srvs::Empty srv;

  while(ros::ok())
  {
      client.call(srv);
    loop_rate.sleep();
  }
  return 0;
}
