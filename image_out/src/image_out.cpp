//=================================================//
/*! @file
 @brief 画像保存のノード
 @author D.Furutani
 @date 2013/11/15
 @attention srvで保存処理のフラグを立ててトピックの画像を保存する

 @TODO 例外処理を追加する.トピックから読み込む形式ではなくsrvだけで処理出きるように。
*/
//=================================================//

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include "image_out/msg2file.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

// 各種変数の定義

// Opencv用の変数
cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);		// 汎用
cv_bridge::CvImagePtr thermal_cv_ptr(new cv_bridge::CvImage);	// 熱画像(二値化)
cv_bridge::CvImagePtr camera_cv_ptr(new cv_bridge::CvImage);	// 光学カメラ画像
cv_bridge::CvImagePtr thermal_raw_cv_ptr(new cv_bridge::CvImage);	// 熱画像

// 保存用のファイル名の初期値
static string filename = "IMAGEFILE.bmp";
static string filename2;

// 保存のモード(0:汎用,1:熱画像,2:光学カメラ画像)
static int savemode = 0;

// ファイル保存のフラグ(true:保存を行う,false:保存を行わない)
static bool save_f = false;

// 保存ディレクトリ名(絶対パス)の初期値
static string directoryname = "/home/furutani/image/";

// デバッグ用
//static const string OPENCV_WINDOW = "Image window";
//static cv::Mat M_img;

// image購読用のクラス=========================================//
class Imagemsg2Imagefile
{
  // ROSの設定
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber thermal_image_sub_;
  image_transport::Subscriber camera_image_sub_;
  image_transport::Subscriber thermal_image_raw_sub_;
  
public:
//=============================================
//@name Imagemsg2Imagefile
//@brief コンストラクタ
//@data 2013/11/15
//@attention
//=============================================
  Imagemsg2Imagefile()
    : it_(nh_)
  {
	// サブスクライバーの定義
	image_sub_ = it_.subscribe("Imagemsg2file", 5,&Imagemsg2Imagefile::imageCb, this);
	thermal_image_sub_ = it_.subscribe("optris/thermal_image_out", 1,&Imagemsg2Imagefile::thermal_imageCb, this);
	camera_image_sub_ = it_.subscribe("usb_cam/image_raw", 1,&Imagemsg2Imagefile::camera_imageCb, this);
    thermal_image_raw_sub_ = it_.subscribe("optris/thermal_image_view", 1,&Imagemsg2Imagefile::thermal_raw_imageCb, this);

  }

//=============================================
//@name Imagemsg2Imagefile
//@brief デストラクタ
//@data 2013/11/15
//@attention
//=============================================
  ~Imagemsg2Imagefile()
  {
  }

//=============================================
//@name imageCb
//@brief 汎用のimageの購読用のコールバック
//@data 2013/11/15
//@attention
//=============================================
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		ROS_INFO("IMAGE_CB");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}    
  }

//=============================================
//@name imageCb
//@brief 二値化熱画像imageの購読用のコールバック
//@data 2013/11/15
//@attention
//=============================================
  void thermal_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	try
	{
		thermal_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		ROS_INFO("IMAGE_THERMAL");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}    
  }

//=============================================
//@name imageCb
//@brief 光学カメラimageの購読用のコールバック
//@data 2013/11/15
//@attention
//=============================================
  void camera_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	try
	{
		camera_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		ROS_INFO("IMAGE_PHOTO");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}    
  }

//=============================================
//@name imageCb
//@brief 熱画像imageの購読用のコールバック
//@data 2014/04/24
//@attention
//=============================================
  void thermal_raw_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
        thermal_raw_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        ROS_INFO("IMAGE_THERMAL_RAW");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
  }
};

//=============================================
//@name msg2file
//@brief imageを画像ファイルに変換して保存するためのsrvの処理
//@data 2013/11/15
//@attention
//=============================================
bool msg2file(image_out::msg2file::Request  &req,
		image_out::msg2file::Response &res)
  {
	// 保存処理を実行中かどうかを確認する
	if (save_f == false){
		// 送られてきた情報を保存する
		filename = req.filename;
		directoryname = req.directoryname;
		filename2 = directoryname;
		filename2 += filename;
		savemode = req.savemode;
	
		// 保存処理フラグを立てる
		save_f = true;
		return true;
	}else{
		ROS_INFO("Now image file saving. Please wait.");
		return false;
	}
  }

//=============================================
//@name msg2file_save
//@brief imageを画像ファイルに変換して保存するための処理
//@data 2013/11/15
//@attention
//=============================================
bool msg2file_save()
  {
	// imageデータをTopicから受け取っているか確認
	if( (savemode == 0 && cv_ptr->image.empty()) ||
		(savemode == 1 && thermal_cv_ptr->image.empty()) ||
        (savemode == 2 && camera_cv_ptr->image.empty()) ||
        (savemode == 3 && thermal_raw_cv_ptr->image.empty()) )
	{
	  cout << "No Image Data" << endl;
	  return false;
	}
	
	// savemodeごとに保存処理を行う
	// 注：ファイルに保存後読み込んだデータを消す.
	//    そうしないと次回保存時に古いデータを保存する可能性がある．
	switch (savemode) {
	  case 0:
	    cv::imwrite(filename2, cv_ptr->image);
	    cv_ptr->image.release();
	    break;
	  case 1:
	    cv::imwrite(filename2, thermal_cv_ptr->image);
	    thermal_cv_ptr->image.release();
	    break;
	  case 2:
	    cv::imwrite(filename2, camera_cv_ptr->image);
	    camera_cv_ptr->image.release();
	    break;
      case 3:
        cv::imwrite(filename2, thermal_raw_cv_ptr->image);
        thermal_raw_cv_ptr->image.release();
        break;
	  default:
	    cout << "SAVE MODE ERROR" << endl;
	    return false;
	}

	cout << "FINISH SAVE!" << endl;

	/*
	// デバッグ用
	M_img = cv::imread(filename2, 1);
	cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::imshow(OPENCV_WINDOW, M_img);
	cv::waitKey(1);
	cv::destroyWindow(OPENCV_WINDOW);
	*/

	return true;
  }

//=============================================
//@name main
//@brief メインループ
//@data 2013/11/15
//@attention
//=============================================
int main(int argc, char** argv)
{
	// ROSの初期設定
	ros::init(argc, argv, "image_out");
	ros::NodeHandle n;
	ros::Rate loop_rate(5);	// ループの待機時間(Hz)

	// imageデータを読み込むクラス
	Imagemsg2Imagefile imif;

	// srvの立ち上げ
	ros::ServiceServer service = n.advertiseService("msg2file_srv", msg2file);

	// メインの処理
	while(ros::ok())
	{
		ros::spinOnce();

		// 保存フラグが立っている場合は保存処理を実行する
		if (save_f == true){
			if(msg2file_save()){
				save_f = false;
			}
		}
		// loop_rateだけ待機
		loop_rate.sleep();
	}
	return 0;
}
