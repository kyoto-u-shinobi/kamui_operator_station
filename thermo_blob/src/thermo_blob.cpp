//=================================================//
/*! @file
 @brief 熱画像処理のノード
 @author D.Furutani
 @date 2013/10/17
 @attention 画像処理にOpenCVとCVblobslibを使用

 @TODO プログラムの整理とコメントの追加
*/
//=================================================//

using namespace std;

#include "ros/ros.h"
#include "opencv/cxcore.h" 
#include "opencv/cvwimage.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>
#include <iostream>
#include <cvblobs/BlobResult.h> //cvBlobslibを追加

//===============各種定義===============

// PI_160の諸元
// 画角は水平画角が17°で垂直画角が23°．横向きにして使用するのでこの値は横向きにした時の値
static const int width_th_max    = 120;	//画像の幅の最大値
static const int height_th_max   = 160;	//画像の高さの最大値
static const int width_th    = 120;	//画像の幅(偶数推奨)
static const int height_th   = 160;	//画像の高さ(偶数推奨)
static const double skip_num_w = 2;	//取得した温度データを幅何点ごとに取得するか(2にすれば2回に1度thermoに温度データを入れる)
static const double skip_num_h = 2;	//取得した温度データを高さ何点ごとに取得するか(2にすれば2回に1度thermoに温度データを入れる)
static const int angleOfView = 17;	//画角

// 画像判定に使用する閾値
static const double drawMaxTemp0 = 42;	//表示する最大温度
static const double drawMinTemp0 = 36;	//表示する最小温度
static const double minArea      = 1500;	//Victimと判断する最小の面積
static const int number_of_thresh = 20;		//複数の閾値で画像判別する場合、閾値の個数
static const int number_of_thresh_Upper = 5;	//２値化大きい方の閾値の個数
static const int thresh_num_blob = 10;	//ラベリングした後、Victim判別するBlobの個数
static const int imagescale = 7;	//表示する熱画像の元データからの表示倍率(2013/03/19 古谷)

// snakeImageに必要なパラメータ
static const int SEGMENT = 100;

// 熱画像の表示設定
static const char* windowNameTH = "ThermalImageView";	//ラベリング処理後の画像を表示するwindow
static const char* windowNameGR = "ThermalImage_grayscaleView";	//２値化前の画像を表示するwindow

// 許容面積比率の設定
static const double threshold_Aspect_Ratio_Upper = 1.7;	//許容面積比率の上限//要Conf化
//static const double threshold_Degree_of_Circularity_Upper = 1.2;	//許容面積比率の上限
static const double threshold_Degree_of_Circularity_Lower = 0.7; //許容面積比率の下限//要Conf化 円形度：完全な円で１になり、円から形が崩れるほど１より小さくなる。りんごで0.8-0.9ぐらい。ばななで0.2ぐらい。

// 温度データのmsg配列
double Arr[width_th][height_th];

//熱画像データ用のクラス=========================================//
class ThermalImage
{
private:
	////簡易的に最大温度値部分検出
	double maxTempSimple;  //現在の画像で温度の最高値
	int maxTempRowSimple;  //温度の最高値を出すピクセルの行番号
	int maxTempColSimple;  //温度の最高値を出すピクセルの列番号


public:
	int width;			//画像の幅(fix)
	int height;			//画像の高さ(fix)
	double drawMaxTemp;		//表示温度の上限値
	double drawMinTemp;		//表示温度の下限値
	double thermo[width_th][height_th];	//熱データ格納用配列
	IplImage *image_original;	//熱画像(拡大前)
	IplImage *image_grayscale;  	//熱画像（２値化前）
	IplImage *image;		//熱画像(２値化後)
	IplImage *image2;		//表示用の熱画像(２値化後，白黒)
	IplImage *image3;		//表示用の熱画像（２値化前，グレースケール）
	IplImage *image4;		//熱画像（高い閾値で２値化した後)

	
	CvPoint p1, p2; //熱源の包含最小格子（傾き無し）の左下と右上
	CBlobResult blobs;
	CBlob blob;

	//画像処理した結果を格納
	double maxTemp;  //最大温度
	double maxArea;	 //最大の領域の面積

	double Aspect_Ratio; 		//blobの縦横比
	double Degree_of_Circularity; 	//blobの円形度
	double Area_Ratio;		//blobの面積（blob内部の空領域を除いた面積）とsnakeの面積（blob外周内の総面積）の比率
	bool detected_victim; 		//True: 画像中にvictimが存在する False: 存在しない
	
	int maxTempRow;	//
	int maxTempCol; //温度の最高値を出すピクセルの列番号
	double pan;	//上下角[deg](画面上での角度)
	double tilte;	//左右角[deg](画面上での角度．左が正)

	//コンストラクタ
	ThermalImage()
	{
		memset(thermo,0,sizeof(thermo));		// 温度データの初期化
		width       = (int)(width_th/skip_num_w);	// 熱画像の横幅(固定)
		height      = (int)(height_th/skip_num_h);	// 熱画像の高さ(固定)
		drawMaxTemp = drawMaxTemp0;	//
		drawMinTemp = drawMinTemp0;
		maxTempSimple     = drawMinTemp-1;
		maxTempRowSimple  = 0;
		maxTempColSimple  = 0;

		pan = 0;
		tilte = 0;

		Aspect_Ratio = 0; //blobの縦横比
		Degree_of_Circularity = 0; //blobの円形度
		Area_Ratio = 0;	//blobの面積（blob内部の空領域を除いた面積）とsnakeの面積（blob外周内の総面積）の比率
		detected_victim = false;
		p1.x = 0;
		p1.y = 0;
		p2.x = 0;
		p2.y = 0;

		//各種認識結果をリセット
		maxTemp = drawMinTemp;  //最大温度
		maxArea =  0;	//最大の領域の面積
		maxTempRow = 0;	//
		maxTempCol = 0; //温度の最高値を出すピクセルの列番号

		//領域の確保
		image_original  = cvCreateImage( cvSize(width,height),IPL_DEPTH_8U, 1);
		image_grayscale  = cvCreateImage( cvSize(width*imagescale,height*imagescale),IPL_DEPTH_8U, 1);
		image  = cvCreateImage( cvSize(width*imagescale,height*imagescale),IPL_DEPTH_8U, 1);
		image4  = cvCreateImage( cvSize(width*imagescale,height*imagescale),IPL_DEPTH_8U, 1);
	
	};

	//画像を保存する
	bool saveThermalImage(char* filename)
	{
		cvSaveImage(filename,image,0);
		return true;
	}

	//温度の上限値を設定
	void setMaxTemp(double drawMaxTemp0){drawMaxTemp = drawMaxTemp0;}

	//温度の下限値を設定
	void setMinTemp(double drawMinTemp0){drawMinTemp = drawMinTemp0;}

	//現在の画像の温度の最大値を取得
	double getMaxTemp(){return maxTemp;}

	//画素値(0-255)を温度(℃)に変換する
	double pixel2deg(unsigned char pixel0)
	{
		return ((int)pixel0*(drawMaxTemp-drawMinTemp)/255.0+drawMinTemp);	
	}

	//温度(℃)を画素値(0-255)に変換する
	unsigned char deg2pixel(double deg0)
	{
		unsigned char pixel = 0;
		if(deg0 <= drawMinTemp)pixel = (unsigned char)0;
		else if (deg0 >= 50)pixel = (unsigned char)0;
		else if(deg0 >= drawMaxTemp)pixel = (unsigned char)255;
		else pixel = (unsigned char)(255.0/(drawMaxTemp-drawMinTemp)*(deg0-drawMinTemp));	
		return pixel;
	}

	// 取得したデータの処理
	bool processThermalImage()
	{
		convertImage();		//画像を作成
		imageProcessing();	//画像を二値化してラベリングを行う
		show_image_for_debugging();	//二値化した画像を表示する

		return detected_victim;
	}

	// 温度データをIplImage型に変更する
	void convertImage()
	{
		maxTempSimple = drawMinTemp -1;

		for(int i= 0;i<width;i++)
		{
			for(int j= 0;j<height;j++)
			{
				image_original->imageData[j*(width)+i] = deg2pixel(thermo[i][j]);
				if(maxTempSimple < thermo[i][j] && thermo[i][j] < 100 )//明らかに変なものは削除
				{
					maxTempSimple = thermo[i][j];
					maxTempRowSimple = i;
					maxTempColSimple = j;
				}
			}
		}

		// オリジナルIplImageをコピーして画像処理用のIplImageにする
		cvResize(image_original,image_grayscale,1);
	
	}

//=============================================
//@name imageProcessing
//@brief 複数閾値で２値化する
//@data 2012/4/27
//@attention　閾値を複数回処理する（hanamoto）
//=============================================	
	 void imageProcessing()
	 {
		detected_victim = false;//vicitm判定ブーリアン
		CvTermCriteria crit;
		crit.type = CV_TERMCRIT_ITER;
		crit.max_iter = 10;
		int t;
		int s;
		int num_blob;

		//スムージング
		cvSmooth(image_grayscale,image_grayscale,3,3,0,0,0);

		//注釈：閾値の上限(drawMaxTemp0)から下限(drawMinTemp0)まで閾値thrを変更しながら繰り返し２値化とラベリングを行う
		for(t = 0; t < number_of_thresh; t++){
			//for(s = 0; s < number_of_thresh_Upper; s++){

				//2値化用の閾値を温度(℃)から画素値(0-255)に変換する
				unsigned char thr = this->deg2pixel(drawMaxTemp0 - ((drawMaxTemp0 - drawMinTemp0) / number_of_thresh) * t);
				//unsigned char thr2 = this->deg2pixel(drawMaxTemp0 - ((drawMaxTemp0 - (drawMaxTemp0 - ((drawMaxTemp0 - drawMinTemp0) / number_of_thresh) * t)) * s) / number_of_thresh);
				//unsigned char thr2 = this->deg2pixel(drawMaxTemp0 - ((drawMaxTemp0 - drawMinTemp0) / number_of_thresh) * t );

				// 2値化処理
				//注釈：cvThresholdはimage_grayscaleの各データの値がthr以上なら２５５にするという２値化関数
				cvThreshold(image_grayscale, image, thr, 255, CV_THRESH_BINARY);
				//cvThreshold(image_grayscale, image4, thr2, 255, CV_THRESH_BINARY);

				//cvXor(image, image4, image);

				//膨張３回→圧縮３回でノイズ除去
				cvDilate(image, image, NULL, 3);
				cvErode(image, image, NULL, 3);
				
				//ラべリングを実行
				blobs = CBlobResult(image, NULL, 0);						//ラべリング
				blobs.Filter (blobs, B_INCLUDE, CBlobGetArea (), B_INSIDE, minArea, 130000);	//面積でフィルタリング			
			
				if(blobs.GetNumBlobs() > 0)//ラべリングの結果，該当領域が存在した場合
				{	
					num_blob = blobs.GetNumBlobs();
					if(num_blob > thresh_num_blob)
					{
						num_blob = thresh_num_blob;
					}
					for (int i = 0; i < num_blob; i++) 
					{
						
						blobs.GetNthBlob(CBlobGetArea(), i, blob);
						Aspect_Ratio = (blob.MaxY() - blob.MinY()) / (blob.MaxX() - blob.MinX());
						Degree_of_Circularity = (4 * M_PI * blob.Area()) / pow(blob.Perimeter(), 2);
						
						if(victim_searching(Degree_of_Circularity, Aspect_Ratio))//victim発見基準を満たすとき
						{
							maxTemp = pixel2deg(thr);
							detected_victim = true;
							break;
						}
					}

						
				}
				else//調べるべき領域が存在しなかった場合
				{
					detected_victim = false;
				}

				if(detected_victim)
				{
					break;
				}

			}
		//}
	 }

//=============================================
//@name victim_searching
//@brief 画像中にVictimが存在するかしないかを判断し，thermalimage内のブールを書き換える
//@data 2012/4/17
//@attention
//=============================================	
	 bool victim_searching(double D, double A)
	 {
		 if(threshold_Degree_of_Circularity_Lower <= 1)
		 {

			 if(1 / threshold_Aspect_Ratio_Upper < A && A < threshold_Aspect_Ratio_Upper){
				 if(threshold_Degree_of_Circularity_Lower <= D) //円形度とアスペクト比がvictim発見基準を満たすとき
				 {
					 return true;// victimと判断
				 }
			}
		 }
		 else
		 {
			 cout<<"形状判定できませんでした。threshold_Degree_of_Circularity_Lowerは1以下に設定してください。"<<endl;
		 }
		 
		 return false;//基準を満たさないときfalseを返す。
	 }
//=============================================
//@name show_image_for_debugging
//@brief ２値化，ラベリング処理後の画像を表示する．Victimが存在する場合，blobをマークして表示
//@data 2012/4/17
//@attention
//=============================================	
	 void show_image_for_debugging()
	 {
		//cout<<"画像の表示"<<endl;
		image2 = cvCreateImage( cvSize(width*imagescale,height*imagescale),IPL_DEPTH_8U, 3);
		cvCvtColor( image, image2, CV_GRAY2RGB );
		if(detected_victim){
			p1.x = blob.MinX();
			p1.y = blob.MinY();
			p2.x = blob.MaxX();
			p2.y = blob.MaxY();
			cvRectangle(image2, p1, p2, CV_RGB(255, 0, 0), 2, 8, 0);
			blob.FillBlob(image2, CV_RGB (0, 255, 0));//blobを緑でぬる
		}
		cvNamedWindow(windowNameTH,CV_WINDOW_AUTOSIZE);
		cvShowImage("ThermalImageView", image2);

		//ウインドウを表示するために１ミリ秒待機
		cvWaitKey(1);
	 }
//=============================================
//@name show_image_grayscale
//@brief ２値化前の画像を表示する
//@data 2012/4/17
//@attention
//=============================================	
	 void show_image_for_grayscale()
	 {
		 image3 = cvCreateImage( cvSize(width*imagescale,height*imagescale),IPL_DEPTH_8U, 3);
		 cvCvtColor( image_grayscale, image3, CV_GRAY2RGB );
		 cvNamedWindow(windowNameGR,CV_WINDOW_AUTOSIZE);
		 cvShowImage("ThermalImage_grayscaleView", image3);
	 }

};

//=============================================
//@name thermoCallback
//@brief msgで受け取った温度データを配列に格納する
//@data 2013/10/17
//@attention
//=============================================	
void thermoCallback(const std_msgs::Float32MultiArray::ConstPtr& thermo_msg)
{
  // 取得を示すROS_INFOを流す
  //ROS_INFO("Now get thermo datas");

  int i = 0;
  int j = 0;
  // 購読したデータを配列に格納する
  for(std::vector<float>::const_iterator it = thermo_msg->data.begin(); it != thermo_msg->data.end(); ++it)
  {
	Arr[i][j] = *it;
	i++;
	if(i == (int)(width_th/skip_num_w)){
		i = 0;
		j++;
	}
  }
}

//=============================================
//@name main
//@brief メインループ
//@data 2013/10/17
//@attention
//=============================================	
int main(int argc, char **argv)
{
	// 各種設定
	ThermalImage ti;	// 画像処理クラスの定義

	// ROSの初期化設定
	ros::init(argc, argv, "thermo_blob"); 
	ros::NodeHandle n;
	ros::Publisher thermoblob_pub = n.advertise<std_msgs::Bool>("ThermoBlob", 100);
	ros::Subscriber thermoblob_sub = n.subscribe("Thermo", 100, thermoCallback);
	ros::Rate loop_rate(5);	// ループの待機時間(Hz)

	// 各種変数の設定
	bool blobdetect = false;	// Victimの有無を示す変数
	std_msgs::Bool blobmsg;		// Victimの有無を流すmsg

	// メインの処理
	while(ros::ok())
	{
		// ループが回るごとにROS_INFOを流す
		//ROS_INFO("Thermo_Blob Running!");

		// コールバックを呼び出して温度データの取得
		ros::spinOnce();
		
		// 取得した温度データを熱画像処理クラスに送る
		memcpy(ti.thermo,Arr,sizeof(Arr));

		// 熱画像の処理
		if(ti.processThermalImage() == true){
			//ROS_INFO("Blob detect : True!");
			blobdetect = true;
		}else{
			//ROS_INFO("Blob detect : False!");
			blobdetect = false;
		}

		// Victim検出の有無をmsgで流す
		blobmsg.data = blobdetect;
		thermoblob_pub.publish(blobmsg);	//PublisherとしてVictimの有無をBool型で流す
		loop_rate.sleep();	// loop_rateだけ待機

	}
	return 0;
}
