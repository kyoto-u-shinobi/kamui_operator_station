//***********************************************************************************************//
//* main.cpp									     2015/07/01 *//
//*												*//
//* nityc/select										*//
//*												*//
//*								(c) HAREHIME 2015 NITYC H.MAEDA *//
//***********************************************************************************************//


//-- [ヘッダファイル] ---------------------------------------------------------------------------//
#include "select.h"											// selectヘッダファイル読み込み
//-----------------------------------------------------------------------------------------------//


//-- [DEBUG定義] --------------------------------------------------------------------------------//
#define DEBUG_SELECT_DATA										// DEBUG定義 [デバッグ用]
#define DEBUG_SELECT_TWIST										// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_LEADER_KOHGA3_1									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_LEADER_KOHGA3_2									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_LEADER_FRIGO_1									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_LEADER_FRIGO_2									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_LEADER_UMRS2009_1									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_LEADER_UMRS2009_2									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_LEADER_UMRS2010_1									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_LEADER_UMRS2010_2									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_IN_KOHGA3_1										// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_IN_KOHGA3_2										// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_IN_FRIGO_1										// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_IN_FRIGO_2										// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_IN_UMRS2009_1									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_IN_UMRS2009_2									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_IN_UMRS2010_1									// DEBUG定義 [デバッグ用]
#define DEBUG_TWIST_IN_UMRS2010_2									// DEBUG定義 [デバッグ用]
//-----------------------------------------------------------------------------------------------//


//-- [プロトタイプ宣言] -------------------------------------------------------------------------//
void select_twist_callback(const nityc::SelectStamped::ConstPtr &);					// コールバック関数
void twist_leader_kohga3_1_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_leader_kohga3_2_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_leader_frigo_1_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_leader_frigo_2_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_leader_umrs2009_1_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_leader_umrs2009_2_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_leader_umrs2010_1_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_leader_umrs2010_2_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_in_kohga3_1_callback(const geometry_msgs::TwistStamped::ConstPtr &);				// コールバック関数
void twist_in_kohga3_2_callback(const geometry_msgs::TwistStamped::ConstPtr &);				// コールバック関数
void twist_in_frigo_1_callback(const geometry_msgs::TwistStamped::ConstPtr &);				// コールバック関数
void twist_in_frigo_2_callback(const geometry_msgs::TwistStamped::ConstPtr &);				// コールバック関数
void twist_in_umrs2009_1_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_in_umrs2009_2_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_in_umrs2010_1_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
void twist_in_umrs2010_2_callback(const geometry_msgs::TwistStamped::ConstPtr &);			// コールバック関数
//-----------------------------------------------------------------------------------------------//


//-- [変数宣言] ---------------------------------------------------------------------------------//
int twist_select_data[8];										// 切替データ格納配列
int twist_out_flag[8];											// フラグ格納配列
geometry_msgs::TwistStamped twist_out_kohga3_1;								// 速度ベクトル格納メッセージ
geometry_msgs::TwistStamped twist_out_kohga3_2;								// 速度ベクトル格納メッセージ
geometry_msgs::TwistStamped twist_out_frigo_1;								// 速度ベクトル格納メッセージ
geometry_msgs::TwistStamped twist_out_frigo_2;								// 速度ベクトル格納メッセージ
geometry_msgs::TwistStamped twist_out_umrs2009_1;							// 速度ベクトル格納メッセージ
geometry_msgs::TwistStamped twist_out_umrs2009_2;							// 速度ベクトル格納メッセージ
geometry_msgs::TwistStamped twist_out_umrs2010_1;							// 速度ベクトル格納メッセージ
geometry_msgs::TwistStamped twist_out_umrs2010_2;							// 速度ベクトル格納メッセージ
//-----------------------------------------------------------------------------------------------//


//-- [メイン関数] -------------------------------------------------------------------------------//
//- int main(int argc, char *argv[])								-//
//- 機能   : メインのプログラム処理を行う							-//
//- 引数   : (int)   コマンドライン引数の数							-//
//-          (char*) コマンドライン引数								-//
//- 戻り値 : (int)   -										-//
//-----------------------------------------------------------------------------------------------//
int main(int argc, char *argv[])
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using geometry_msgs::TwistStamped;								// geometry_msgs::TwistStamped指定
	using nityc::SelectStamped;									// nityc::SelectStamped指定
	using ros::init;										// ros::init指定
	using ros::ok;											// ros::ok指定
	using ros::spinOnce;										// ros::spinOnce指定
	using ros::NodeHandle;										// ros::NodeHandle指定
	using ros::Publisher;										// ros::Publisher指定
	using ros::Rate;										// ros::Rate指定
	using ros::Subscriber;										// ros::Subscriber指定

	// メッセージ表示
	message_display(PROGRAM_BEGINNING);								// メッセージ表示関数
	message_display(INITIALIZATION_BEGINNING);							// メッセージ表示関数

	//---------------------------------------------------------------------------------------//


	//-- [変数宣言] -------------------------------------------------------------------------//

	int i;												// カウンタ格納変数

	//---------------------------------------------------------------------------------------//


	//-- [変数初期化] -----------------------------------------------------------------------//

	memset(twist_select_data, 0, sizeof(twist_select_data));					// 切替データ初期化
	memset(twist_out_flag, 0, sizeof(twist_out_flag));						// 切替データ初期化

	//---------------------------------------------------------------------------------------//


	//-- [ROS初期化] ------------------------------------------------------------------------//

	// ROS処理初期化
	init(argc, argv, "select");									// ROS初期化
	NodeHandle select_twist_node;									// ノードハンドル生成
	NodeHandle twist_leader_kohga3_1_node;								// ノードハンドル生成
	NodeHandle twist_leader_kohga3_2_node;								// ノードハンドル生成
	NodeHandle twist_leader_frigo_1_node;								// ノードハンドル生成
	NodeHandle twist_leader_frigo_2_node;								// ノードハンドル生成
	NodeHandle twist_leader_umrs2009_1_node;							// ノードハンドル生成
	NodeHandle twist_leader_umrs2009_2_node;							// ノードハンドル生成
	NodeHandle twist_leader_umrs2010_1_node;							// ノードハンドル生成
	NodeHandle twist_leader_umrs2010_2_node;							// ノードハンドル生成
	NodeHandle twist_in_kohga3_1_node;								// ノードハンドル生成
	NodeHandle twist_in_kohga3_2_node;								// ノードハンドル生成
	NodeHandle twist_in_frigo_1_node;								// ノードハンドル生成
	NodeHandle twist_in_frigo_2_node;								// ノードハンドル生成
	NodeHandle twist_in_umrs2009_1_node;								// ノードハンドル生成
	NodeHandle twist_in_umrs2009_2_node;								// ノードハンドル生成
	NodeHandle twist_in_umrs2010_1_node;								// ノードハンドル生成
	NodeHandle twist_in_umrs2010_2_node;								// ノードハンドル生成
	NodeHandle twist_out_kohga3_1_node;								// ノードハンドル生成
	NodeHandle twist_out_kohga3_2_node;								// ノードハンドル生成
	NodeHandle twist_out_frigo_1_node;								// ノードハンドル生成
	NodeHandle twist_out_frigo_2_node;								// ノードハンドル生成
	NodeHandle twist_out_umrs2009_1_node;								// ノードハンドル生成
	NodeHandle twist_out_umrs2009_2_node;								// ノードハンドル生成
	NodeHandle twist_out_umrs2010_1_node;								// ノードハンドル生成
	NodeHandle twist_out_umrs2010_2_node;								// ノードハンドル生成
	Subscriber select_twist_msg = select_twist_node.subscribe<SelectStamped>(
		"select_twist", 1000, select_twist_callback);						// サブスクライブ生成-セレクター信号-
	Subscriber twist_leader_kohga3_1_msg = twist_leader_kohga3_1_node.subscribe<TwistStamped>(
		"twist_leader_kohga3_1", 1000, twist_leader_kohga3_1_callback);				// サブスクライブ生成
	Subscriber twist_leader_kohga3_2_msg = twist_leader_kohga3_2_node.subscribe<TwistStamped>(
		"twist_leader_kohga3_2", 1000, twist_leader_kohga3_2_callback);				// サブスクライブ生成
	Subscriber twist_leader_frigo_1_msg = twist_leader_frigo_1_node.subscribe<TwistStamped>(
		"twist_leader_frigo_1", 1000, twist_leader_frigo_1_callback);				// サブスクライブ生成
	Subscriber twist_leader_frigo_2_msg = twist_leader_frigo_2_node.subscribe<TwistStamped>(
		"twist_leader_frigo_2", 1000, twist_leader_frigo_2_callback);				// サブスクライブ生成
	Subscriber twist_leader_umrs2009_1_msg = twist_leader_umrs2009_1_node.subscribe<TwistStamped>(
		"twist_leader_umrs2009_1", 1000, twist_leader_umrs2009_1_callback);			// サブスクライブ生成
	Subscriber twist_leader_umrs2009_2_msg = twist_leader_umrs2009_2_node.subscribe<TwistStamped>(
		"twist_leader_umrs2009_2", 1000, twist_leader_umrs2009_2_callback);			// サブスクライブ生成
	Subscriber twist_leader_umrs2010_1_msg = twist_leader_umrs2010_1_node.subscribe<TwistStamped>(
		"twist_leader_umrs2010_1", 1000, twist_leader_umrs2010_1_callback);			// サブスクライブ生成
	Subscriber twist_leader_umrs2010_2_msg = twist_leader_umrs2010_2_node.subscribe<TwistStamped>(
		"twist_leader_umrs2010_2", 1000, twist_leader_umrs2010_2_callback);			// サブスクライブ生成
	Subscriber twist_in_kohga3_1_msg = twist_in_kohga3_1_node.subscribe<TwistStamped>(
		"twist_in_kohga3_1", 1000, twist_in_kohga3_1_callback);					// サブスクライブ生成
	Subscriber twist_in_kohga3_2_msg = twist_in_kohga3_2_node.subscribe<TwistStamped>(
		"twist_in_kohga3_2", 1000, twist_in_kohga3_2_callback);					// サブスクライブ生成
	Subscriber twist_in_frigo_1_msg = twist_in_frigo_1_node.subscribe<TwistStamped>(
		"twist_in_frigo_1", 1000, twist_in_frigo_1_callback);					// サブスクライブ生成
	Subscriber twist_in_frigo_2_msg = twist_in_frigo_2_node.subscribe<TwistStamped>(
		"twist_in_frigo_2", 1000, twist_in_frigo_2_callback);					// サブスクライブ生成
	Subscriber twist_in_umrs2009_1_msg = twist_in_umrs2009_1_node.subscribe<TwistStamped>(
		"twist_in_umrs2009_1", 1000, twist_in_umrs2009_1_callback);				// サブスクライブ生成
	Subscriber twist_in_umrs2009_2_msg = twist_in_umrs2009_2_node.subscribe<TwistStamped>(
		"twist_in_umrs2009_2", 1000, twist_in_umrs2009_2_callback);				// サブスクライブ生成
	Subscriber twist_in_umrs2010_1_msg = twist_in_umrs2010_1_node.subscribe<TwistStamped>(
		"twist_in_umrs2010_1", 1000, twist_in_umrs2010_1_callback);				// サブスクライブ生成
	Subscriber twist_in_umrs2010_2_msg = twist_in_umrs2010_2_node.subscribe<TwistStamped>(
		"twist_in_umrs2010_2", 1000, twist_in_umrs2010_2_callback);				// サブスクライブ生成
	Publisher twist_out_kohga3_1_msg
		= twist_out_kohga3_1_node.advertise<TwistStamped>("twist_out_kohga3_1", 1000);		// パブリッシャ生成
	Publisher twist_out_kohga3_2_msg
		= twist_out_kohga3_2_node.advertise<TwistStamped>("twist_out_kohga3_2", 1000);		// パブリッシャ生成
	Publisher twist_out_frigo_1_msg
		= twist_out_frigo_1_node.advertise<TwistStamped>("twist_out_frigo_1", 1000);		// パブリッシャ生成
	Publisher twist_out_frigo_2_msg
		= twist_out_frigo_2_node.advertise<TwistStamped>("twist_out_frigo_2", 1000);		// パブリッシャ生成
	Publisher twist_out_umrs2009_1_msg
		= twist_out_umrs2009_1_node.advertise<TwistStamped>("twist_out_umrs2009_1", 1000);	// パブリッシャ生成
	Publisher twist_out_umrs2009_2_msg
		= twist_out_umrs2009_2_node.advertise<TwistStamped>("twist_out_umrs2009_2", 1000);	// パブリッシャ生成
	Publisher twist_out_umrs2010_1_msg
		= twist_out_umrs2010_1_node.advertise<TwistStamped>("twist_out_umrs2010_1", 1000);	// パブリッシャ生成
	Publisher twist_out_umrs2010_2_msg
		= twist_out_umrs2010_2_node.advertise<TwistStamped>("twist_out_umrs2010_2", 1000);	// パブリッシャ生成
	Rate loop_rate(100);										// ループ頻度設定

	// メッセージ表示
	message_display(ROS);										// メッセージ表示関数
	message_display(INITIALIZATION_END);								// メッセージ表示関数

	//---------------------------------------------------------------------------------------//


	//-- [メインプログラム] -----------------------------------------------------------------//
	//変数宣言
	double masterspeed = 0.0;
	double masteromega = 0.0;

	
	// メッセージ表示
	message_display(THE_MAIN_PROGRAM_BEGINNING);							// メッセージ表示関数

	// メインループ
	while(ok()){											// メインループ
		//強制代入
		printf("speed[m/s] = ");
		i=scanf("%lf",&masterspeed);

		printf("omega[rad/s] = ");
		i=scanf("%lf",&masteromega);

		

		//-- [メッセージ更新処理] ---------------------------------------------------------------//
		//twist_out_umrs2009_1.header.stamp    = 0;			// タイムスタンプ格納
		//twist_out_umrs2009_1.header.frame_id = 0;
		twist_out_umrs2009_1.twist.linear.x  = masterspeed;
		twist_out_umrs2009_1.twist.linear.y  = 0.0;			// 速度ベクトルy成分格納
		twist_out_umrs2009_1.twist.linear.z  = 0.0;			// 速度ベクトルz成分格納
		twist_out_umrs2009_1.twist.angular.x = 0.0;			// 角速度ベクトルx成分格納
		twist_out_umrs2009_1.twist.angular.y = 0.0;
		twist_out_umrs2009_1.twist.angular.z = masteromega;
		twist_out_flag[4] = ON; 

		
		//---------------------------------------------------------------------------------------//


		//-- [メッセージ出力処理] -------------------------------------------------------//

		for(i = 0; i < 8; i++){

			if(twist_out_flag[i] == ON){							// 更新チェック

				if(i == 0) twist_out_kohga3_1_msg.publish(twist_out_kohga3_1);		// ROSメッセージ発信
				if(i == 1) twist_out_kohga3_2_msg.publish(twist_out_kohga3_2);		// ROSメッセージ発信
				if(i == 2) twist_out_frigo_1_msg.publish(twist_out_frigo_1);		// ROSメッセージ発信
				if(i == 3) twist_out_frigo_2_msg.publish(twist_out_frigo_2);		// ROSメッセージ発信
				if(i == 4) twist_out_umrs2009_1_msg.publish(twist_out_umrs2009_1);	// ROSメッセージ発信
				if(i == 5) twist_out_umrs2009_2_msg.publish(twist_out_umrs2009_2);	// ROSメッセージ発信
				if(i == 6) twist_out_umrs2010_1_msg.publish(twist_out_umrs2010_1);	// ROSメッセージ発信
				if(i == 7) twist_out_umrs2010_2_msg.publish(twist_out_umrs2010_2);	// ROSメッセージ発信

				twist_out_flag[i] == OFF;						// フラグリセット

			}

		}

		//-------------------------------------------------------------------------------//


		//-- [コールバック処理] ---------------------------------------------------------//

		spinOnce();										// メッセージコールバック呼び出し
		loop_rate.sleep();									// 待機

		//-------------------------------------------------------------------------------//


	}

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_SELECT_TWIST									// デバッグ実行設定開始
	printf("\n\n\n\n");										// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_LEADER_KOHGA3_1								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_LEADER_KOHGA3_2								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_LEADER_FRIGO_1								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_LEADER_FRIGO_2								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_LEADER_UMRS2009_1								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_LEADER_UMRS2009_2								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_LEADER_UMRS2010_1								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_LEADER_UMRS2010_2								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_IN_KOHGA3_1									// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_IN_KOHGA3_2									// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_IN_FRIGO_1									// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_IN_FRIGO_2									// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_IN_UMRS2009_1								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_IN_UMRS2009_2								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_IN_UMRS2010_1								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifndef DEBUG_TWIST_IN_UMRS2010_2								// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	#ifdef DEBUG_SELECT_DATA									// デバッグ実行設定開始
	printf("\n\n\n\n\n\n\n\n");									// メッセージ表示
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [プログラム終了処理] ---------------------------------------------------------------//

	// メッセージ表示
	message_display(THE_MAIN_PROGRAM_END);								// メッセージ表示関数
	message_display(PROGRAM_END);									// メッセージ表示関数

	//---------------------------------------------------------------------------------------//


	return SUCCESS;											// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void select_twist_callback(const nityc::SelectStamped::ConstPtr& select_twist)		-//
//- 機能   : コールバック処理を行う [nityc/SelectStamped]					-//
//- 引数   : (const nityc::SelectStamped::ConstPtr&) 更新メッセージアドレス			-//
//- 戻り値 : (void)                                  なし					-//
//-----------------------------------------------------------------------------------------------//
void select_twist_callback(const nityc::SelectStamped::ConstPtr& select_twist)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [変数宣言] -------------------------------------------------------------------------//

	int i;												// カウンタ格納変数

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_SELECT_TWIST									// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ select_twist sequence number             ]");						// メッセージ表示
	printf("[ %010d                               ]\n", select_twist->header.seq);			// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ select_twist timestamp                   ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", select_twist->header.stamp.sec,
		select_twist->header.stamp.nsec);							// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ select_twist frame id                    ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << select_twist->header.frame_id;								// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ select_twist select_data                 ]");						// メッセージ表示
	printf("[ %2d %2d %2d %2d %2d %2d %2d %2d                  ]\n",
		select_twist->select_data[0], select_twist->select_data[1], select_twist->select_data[2],
		select_twist->select_data[3], select_twist->select_data[4], select_twist->select_data[5],
		select_twist->select_data[6], select_twist->select_data[7]);				// メッセージ表示
	printf("\033[%dA", 4);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [切替表示処理] ---------------------------------------------------------------------//

	#ifdef DEBUG_SELECT_DATA									// デバッグ実行設定開始
	for(i = 0; i < 8; i++){										// 切替表示処理

		printf("[ nityc/select              ]");						// メッセージ表示

		if(select_twist->select_data[i] == 0)
			printf("[ twist_leader_kohga3_1                    ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 1)
			printf("[ twist_leader_kohga3_2                    ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 2)
			printf("[ twist_leader_frigo_1                     ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 3)
			printf("[ twist_leader_frigo_2                     ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 4)
			printf("[ twist_leader_umrs2009_1                  ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 5)
			printf("[ twist_leader_umrs2009_2                  ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 6)
			printf("[ twist_leader_umrs2010_1                  ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 7)
			printf("[ twist_leader_umrs2010_2                  ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 8)
			printf("[ twist_in_kohga3_1                        ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 9)
			printf("[ twist_in_kohga3_2                        ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 10)
			printf("[ twist_in_frigo_1                         ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 11)
			printf("[ twist_in_frigo_2                         ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 12)
			printf("[ twist_in_umrs2009_1                      ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 13)
			printf("[ twist_in_umrs2009_2                      ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 14)
			printf("[ twist_in_umrs2010_1                      ]");				// メッセージ表示
		else if(select_twist->select_data[i] == 15)
			printf("[ twist_in_umrs2010_2                      ]");				// メッセージ表示

		if(i == 0) printf("[ -> twist_out_kohga3_1                    ]\n");			// メッセージ表示
		else if(i == 1) printf("[ -> twist_out_kohga3_2                    ]\n");		// メッセージ表示
		else if(i == 2) printf("[ -> twist_out_frigo_1                     ]\n");		// メッセージ表示
		else if(i == 3) printf("[ -> twist_out_frigo_2                     ]\n");		// メッセージ表示
		else if(i == 4) printf("[ -> twist_out_umrs2009_1                  ]\n");		// メッセージ表示
		else if(i == 5) printf("[ -> twist_out_umrs2009_2                  ]\n");		// メッセージ表示
		else if(i == 6) printf("[ -> twist_out_umrs2010_1                  ]\n");		// メッセージ表示
		else if(i == 7) printf("[ -> twist_out_umrs2010_2                  ]\n");		// メッセージ表示

		twist_select_data[i] = select_twist->select_data[i];					// 切替データ格納

	}

	printf("\033[%dA", 8);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_leader_kohga3_1_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_leader_kohga3_1)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_leader_kohga3_1_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_leader_kohga3_1)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_LEADER_KOHGA3_1								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 sequence number    ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_leader_kohga3_1->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 timestamp          ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_leader_kohga3_1->header.stamp.sec,
		twist_leader_kohga3_1->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 frame id           ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_leader_kohga3_1->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 linear x           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_1->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 linear y           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_1->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 linear z           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_1->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 angular x          ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_1->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 angular y          ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_1->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_1 angular z          ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_1->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(LEADER_KOHGA3_1_NUMBER, twist_select_data, twist_out_flag, twist_leader_kohga3_1,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_leader_kohga3_2_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_leader_kohga3_2)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_leader_kohga3_2_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_leader_kohga3_2)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_LEADER_KOHGA3_2								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 sequence number    ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_leader_kohga3_2->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 timestamp          ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_leader_kohga3_2->header.stamp.sec,
		twist_leader_kohga3_2->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 frame id           ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_leader_kohga3_2->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 linear x           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_2->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 linear y           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_2->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 linear z           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_2->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 angular x          ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_2->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 angular y          ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_2->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_kohga3_2 angular z          ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_kohga3_2->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(LEADER_KOHGA3_2_NUMBER, twist_select_data, twist_out_flag, twist_leader_kohga3_2,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_leader_frigo_1_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_leader_frigo_1)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_leader_frigo_1_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_leader_frigo_1)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_LEADER_FRIGO_1								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 sequence number     ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_leader_frigo_1->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 timestamp           ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_leader_frigo_1->header.stamp.sec,
		twist_leader_frigo_1->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 frame id            ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_leader_frigo_1->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 linear x            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_1->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 linear y            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_1->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 linear z            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_1->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 angular x           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_1->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 angular y           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_1->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_1 angular z           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_1->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(LEADER_FRIGO_1_NUMBER, twist_select_data, twist_out_flag, twist_leader_frigo_1,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_leader_frigo_2_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_leader_frigo_2)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_leader_frigo_2_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_leader_frigo_2)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_LEADER_FRIGO_2								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 sequence number     ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_leader_frigo_2->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 timestamp           ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_leader_frigo_2->header.stamp.sec,
		twist_leader_frigo_2->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 frame id            ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_leader_frigo_2->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 linear x            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_2->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 linear y            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_2->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 linear z            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_2->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 angular x           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_2->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 angular y           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_2->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_frigo_2 angular z           ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_frigo_2->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(LEADER_FRIGO_2_NUMBER, twist_select_data, twist_out_flag, twist_leader_frigo_2,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_leader_umrs2009_1_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_leader_umrs2009_1)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_leader_umrs2009_1_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_leader_umrs2009_1)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_LEADER_UMRS2009_1								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 sequence number  ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_leader_umrs2009_1->header.seq);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 timestamp        ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_leader_umrs2009_1->header.stamp.sec,
		twist_leader_umrs2009_1->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 frame id         ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_leader_umrs2009_1->header.frame_id;						// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 linear x         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_1->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 linear y         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_1->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 linear z         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_1->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 angular x        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_1->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 angular y        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_1->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_1 angular z        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_1->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(LEADER_UMRS2009_1_NUMBER, twist_select_data, twist_out_flag,
		twist_leader_umrs2009_1, &twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1,
		&twist_out_frigo_2, &twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_leader_umrs2009_2_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_leader_umrs2009_2)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_leader_umrs2009_2_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_leader_umrs2009_2)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_LEADER_UMRS2009_2								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 sequence number  ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_leader_umrs2009_2->header.seq);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 timestamp        ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_leader_umrs2009_2->header.stamp.sec,
		twist_leader_umrs2009_2->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 frame id         ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_leader_umrs2009_2->header.frame_id;						// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 linear x         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_2->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 linear y         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_2->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 linear z         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_2->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 angular x        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_2->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 angular y        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_2->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2009_2 angular z        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2009_2->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(LEADER_UMRS2009_2_NUMBER, twist_select_data, twist_out_flag,
		twist_leader_umrs2009_2, &twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1,
		&twist_out_frigo_2, &twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_leader_umrs2010_1_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_leader_umrs2010_1)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_leader_umrs2010_1_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_leader_umrs2010_1)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_LEADER_UMRS2010_1								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 sequence number  ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_leader_umrs2010_1->header.seq);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 timestamp        ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_leader_umrs2010_1->header.stamp.sec,
		twist_leader_umrs2010_1->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 frame id         ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_leader_umrs2010_1->header.frame_id;						// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 linear x         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_1->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 linear y         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_1->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 linear z         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_1->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 angular x        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_1->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 angular y        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_1->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_1 angular z        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_1->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(LEADER_UMRS2010_1_NUMBER, twist_select_data, twist_out_flag,
		twist_leader_umrs2010_1, &twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1,
		&twist_out_frigo_2, &twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_leader_umrs2010_2_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_leader_umrs2010_2)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_leader_umrs2010_2_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_leader_umrs2010_2)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_LEADER_UMRS2010_2								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 sequence number  ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_leader_umrs2010_2->header.seq);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 timestamp        ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_leader_umrs2010_2->header.stamp.sec,
		twist_leader_umrs2010_2->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 frame id         ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_leader_umrs2010_2->header.frame_id;						// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 linear x         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_2->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 linear y         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_2->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 linear z         ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_2->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 angular x        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_2->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 angular y        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_2->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_leader_umrs2010_2 angular z        ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_leader_umrs2010_2->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(LEADER_UMRS2010_2_NUMBER, twist_select_data, twist_out_flag,
		twist_leader_umrs2010_2, &twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1,
		&twist_out_frigo_2, &twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_in_kohga3_1_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_in_kohga3_1)										-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_in_kohga3_1_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_in_kohga3_1)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_IN_KOHGA3_1									// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 sequence number        ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_in_kohga3_1->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 timestamp              ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_in_kohga3_1->header.stamp.sec,
		twist_in_kohga3_1->header.stamp.nsec);							// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 frame id               ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_in_kohga3_1->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 linear x               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_1->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 linear y               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_1->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 linear z               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_1->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 angular x              ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_1->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 angular y              ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_1->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_1 angular z              ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_1->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(IN_KOHGA3_1_NUMBER, twist_select_data, twist_out_flag, twist_in_kohga3_1,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_in_kohga3_2_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_in_kohga3_2)										-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_in_kohga3_2_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_in_kohga3_2)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_IN_KOHGA3_2									// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 sequence number        ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_in_kohga3_2->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 timestamp              ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_in_kohga3_2->header.stamp.sec,
		twist_in_kohga3_2->header.stamp.nsec);							// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 frame id               ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_in_kohga3_2->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 linear x               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_2->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 linear y               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_2->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 linear z               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_2->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 angular x              ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_2->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 angular y              ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_2->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_kohga3_2 angular z              ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_kohga3_2->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(IN_KOHGA3_2_NUMBER, twist_select_data, twist_out_flag, twist_in_kohga3_2,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_in_frigo_1_callback(const geometry_msgs::TwistStamped::ConstPtr&			-//
//-  twist_in_frigo_1)										-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_in_frigo_1_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_in_frigo_1)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_IN_FRIGO_1									// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 sequence number         ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_in_frigo_1->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 timestamp               ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_in_frigo_1->header.stamp.sec,
		twist_in_frigo_1->header.stamp.nsec);							// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 frame id                ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_in_frigo_1->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 linear x                ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_1->twist.linear.x);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 linear y                ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_1->twist.linear.y);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 linear z                ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_1->twist.linear.z);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 angular x               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_1->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 angular y               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_1->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_1 angular z               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_1->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(IN_FRIGO_1_NUMBER, twist_select_data, twist_out_flag, twist_in_frigo_1,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_in_frigo_2_callback(const geometry_msgs::TwistStamped::ConstPtr&			-//
//-  twist_in_frigo_2)										-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_in_frigo_2_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_in_frigo_2)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_IN_FRIGO_2									// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 sequence number         ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_in_frigo_2->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 timestamp               ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_in_frigo_2->header.stamp.sec,
		twist_in_frigo_2->header.stamp.nsec);							// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 frame id                ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_in_frigo_2->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 linear x                ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_2->twist.linear.x);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 linear y                ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_2->twist.linear.y);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 linear z                ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_2->twist.linear.z);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 angular x               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_2->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 angular y               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_2->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_frigo_2 angular z               ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_frigo_2->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(IN_FRIGO_2_NUMBER, twist_select_data, twist_out_flag, twist_in_frigo_2,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_in_umrs2009_1_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_in_umrs2009_1)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_in_umrs2009_1_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_in_umrs2009_1)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_IN_UMRS2009_1								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 sequence number      ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_in_umrs2009_1->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 timestamp            ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_in_umrs2009_1->header.stamp.sec,
		twist_in_umrs2009_1->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 frame id             ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_in_umrs2009_1->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 linear x             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_1->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 linear y             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_1->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 linear z             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_1->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 angular x            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_1->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 angular y            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_1->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_1 angular z            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_1->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(IN_UMRS2009_1_NUMBER, twist_select_data, twist_out_flag, twist_in_umrs2009_1,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_in_umrs2009_2_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_in_umrs2009_2)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_in_umrs2009_2_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_in_umrs2009_2)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_IN_UMRS2009_2								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 sequence number      ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_in_umrs2009_2->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 timestamp            ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_in_umrs2009_2->header.stamp.sec,
		twist_in_umrs2009_2->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 frame id             ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_in_umrs2009_2->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 linear x             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_2->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 linear y             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_2->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 linear z             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_2->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 angular x            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_2->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 angular y            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_2->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2009_2 angular z            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2009_2->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(IN_UMRS2009_2_NUMBER, twist_select_data, twist_out_flag, twist_in_umrs2009_2,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_in_umrs2010_1_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_in_umrs2010_1)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_in_umrs2010_1_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_in_umrs2010_1)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_IN_UMRS2010_1								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 sequence number      ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_in_umrs2010_1->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 timestamp            ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_in_umrs2010_1->header.stamp.sec,
		twist_in_umrs2010_1->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 frame id             ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_in_umrs2010_1->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 linear x             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_1->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 linear y             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_1->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 linear z             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_1->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 angular x            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_1->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 angular y            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_1->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_1 angular z            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_1->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(IN_UMRS2010_1_NUMBER, twist_select_data, twist_out_flag, twist_in_umrs2010_1,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [コールバック関数] -------------------------------------------------------------------------//
//- void twist_in_umrs2010_2_callback(const geometry_msgs::TwistStamped::ConstPtr&		-//
//-  twist_in_umrs2010_2)									-//
//- 機能   : コールバック処理を行う [geometry_msgs/TwistStamped]				-//
//- 引数   : (const geometry_msgs::TwistStamped::ConstPtr&) 更新メッセージアドレス		-//
//- 戻り値 : (void)                                         なし				-//
//-----------------------------------------------------------------------------------------------//
void twist_in_umrs2010_2_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_in_umrs2010_2)
{

	//-- [名前空間指定] ---------------------------------------------------------------------//

	using std::cout;										// std::cout指定

	//---------------------------------------------------------------------------------------//


	//-- [デバッグ処理] ---------------------------------------------------------------------//

	#ifndef DEBUG_TWIST_IN_UMRS2010_2								// デバッグ実行設定開始
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 sequence number      ]");						// メッセージ表示
	printf("[ %010d                               ]\n", twist_in_umrs2010_2->header.seq);		// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 timestamp            ]");						// メッセージ表示
	printf("[ %10d.%09d                     ]\n", twist_in_umrs2010_2->header.stamp.sec,
		twist_in_umrs2010_2->header.stamp.nsec);						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 frame id             ]");						// メッセージ表示
	printf("[ \"");											// メッセージ表示
	cout << twist_in_umrs2010_2->header.frame_id;							// メッセージ表示
	printf("\"                                       ]\n");						// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 linear x             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_2->twist.linear.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 linear y             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_2->twist.linear.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 linear z             ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_2->twist.linear.z);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 angular x            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_2->twist.angular.x);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 angular y            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_2->twist.angular.y);	// メッセージ表示
	printf("[ nityc/select              ]");							// メッセージ表示
	printf("[ twist_in_umrs2010_2 angular z            ]");						// メッセージ表示
	printf("[ %10.6f                               ]\n", twist_in_umrs2010_2->twist.angular.z);	// メッセージ表示
	printf("\033[%dA", 9);										// メッセージ表示
	fflush(stdout);											// バッファフラッシュ [printf表示]
	#endif												// デバッグ実行設定終了

	//---------------------------------------------------------------------------------------//


	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	message_select(IN_UMRS2010_2_NUMBER, twist_select_data, twist_out_flag, twist_in_umrs2010_2,
		&twist_out_kohga3_1, &twist_out_kohga3_2, &twist_out_frigo_1, &twist_out_frigo_2,
		&twist_out_umrs2009_1, &twist_out_umrs2009_2, &twist_out_umrs2010_1,
		&twist_out_umrs2010_2);									// メッセージ更新処理関数

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//
