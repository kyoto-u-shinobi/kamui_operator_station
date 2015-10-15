//***********************************************************************************************//
//* select.h									     2015/07/01 *//
//*												*//
//* 関数 :											*//
//*  メッセージ表示関数     -> void message_display(unsigned int)				*//
//*  メッセージ更新処理関数 -> void message_select(int, int*, int*,				*//
//*                             geometry_msgs::TwistStamped::ConstPtr,				*//
//*                             geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*,	*//
//*                             geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*,	*//
//*                             geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*,	*//
//*                             geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*);	*//
//*												*//
//*								(c) HAREHIME 2015 NITYC H.MAEDA *//
//***********************************************************************************************//

//-- [ヘッダファイル二重読み込み防止処理] -------------------------------------------------------//
#ifndef __SELECT__											// 条件コンパイル開始
#define __SELECT__											// 条件判断用定義セット
//-----------------------------------------------------------------------------------------------//


//-- [ヘッダファイル] ---------------------------------------------------------------------------//
//- geometry_msgs/TwistStamped.h : geometry_msgs::TwistStamped					-//
//- nityc/RelativePoseStamped.h  : nityc::RelativePoseStamped					-//
//- nityc/SelectStamped.h        : nityc::SelectStamped						-//
//- stdio.h                      : fflush, printf						-//
//- string                       : memset, std::string						-//
//- ros/ros.h                    : ros::init, ros::ok, ros::spinOnce, ros::NodeHandle,		-//
//-                                ros::Publisher, ros::Rate, ros::Time::now			-//
//- sys/time.h                   : localtime, time						-//
//-----------------------------------------------------------------------------------------------//
#include <geometry_msgs/TwistStamped.h>									// geometry_msgs/TwistStamped
													//  ヘッダファイル読み込み
#include <nityc/RelativePoseStamped.h>									// nityc/RelativePoseStamped
													//  ヘッダファイル読み込み
#include <nityc/SelectStamped.h>									// nityc/SelectStamped
													//  ヘッダファイル読み込み
#include <stdio.h>											// stdioヘッダファイル読み込み
#include <string>											// stringファイル読み込み
#include <ros/ros.h>											// ros/rosヘッダファイル読み込み
#include <sys/time.h>											// sys/timeヘッダファイル読み込み
//-----------------------------------------------------------------------------------------------//


//-- [置換] -------------------------------------------------------------------------------------//
//- SUCCESS                    : 0								-//
//- FAILURE                    : - 1								-//
//- ON                         : 1								-//
//- OFF                        : 0								-//
//- PROGRAM_BEGINNING          : 0								-//
//- PROGRAM_END                : 1								-//
//- INITIALIZATION_BEGINNING   : 2								-//
//- INITIALIZATION_END         : 3								-//
//- THE_MAIN_PROGRAM_BEGINNING : 4								-//
//- THE_MAIN_PROGRAM_END       : 5								-//
//- ROS                        : 6								-//
//- LEADER_KOHGA3_1_NUMBER     : 0								-//
//- LEADER_KOHGA3_2_NUMBER     : 1								-//
//- LEADER_FRIGO_1_NUMBER      : 2								-//
//- LEADER_FRIGO_2_NUMBER      : 3								-//
//- LEADER_UMRS2009_1_NUMBER   : 4								-//
//- LEADER_UMRS2009_2_NUMBER   : 5								-//
//- LEADER_UMRS2010_1_NUMBER   : 6								-//
//- LEADER_UMRS2010_2_NUMBER   : 7								-//
//- IN_KOHGA3_1_NUMBER         : 8								-//
//- IN_KOHGA3_2_NUMBER         : 9								-//
//- IN_FRIGO_1_NUMBER          : 10								-//
//- IN_FRIGO_2_NUMBER          : 11								-//
//- IN_UMRS2009_1_NUMBER       : 12								-//
//- IN_UMRS2009_2_NUMBER       : 13								-//
//- IN_UMRS2010_1_NUMBER       : 14								-//
//- IN_UMRS2010_2_NUMBER       : 15								-//
//-----------------------------------------------------------------------------------------------//
#define SUCCESS				0								// 成功
#define FAILURE				- 1								// 失敗
#define ON				1								// ON
#define OFF				0								// OFF
#define PROGRAM_BEGINNING		0								// メッセージ番号0
#define PROGRAM_END			1								// メッセージ番号1
#define INITIALIZATION_BEGINNING	2								// メッセージ番号2
#define INITIALIZATION_END		3								// メッセージ番号3
#define THE_MAIN_PROGRAM_BEGINNING	4								// メッセージ番号4
#define THE_MAIN_PROGRAM_END		5								// メッセージ番号5
#define ROS				6								// メッセージ番号6
#define LEADER_KOHGA3_1_NUMBER		0								// 切替番号
#define LEADER_KOHGA3_2_NUMBER		1								// 切替番号
#define LEADER_FRIGO_1_NUMBER		2								// 切替番号
#define LEADER_FRIGO_2_NUMBER		3								// 切替番号
#define LEADER_UMRS2009_1_NUMBER	4								// 切替番号
#define LEADER_UMRS2009_2_NUMBER	5								// 切替番号
#define LEADER_UMRS2010_1_NUMBER	6								// 切替番号
#define LEADER_UMRS2010_2_NUMBER	7								// 切替番号
#define IN_KOHGA3_1_NUMBER		8								// 切替番号
#define IN_KOHGA3_2_NUMBER		9								// 切替番号
#define IN_FRIGO_1_NUMBER		10								// 切替番号
#define IN_FRIGO_2_NUMBER		11								// 切替番号
#define IN_UMRS2009_1_NUMBER		12								// 切替番号
#define IN_UMRS2009_2_NUMBER		13								// 切替番号
#define IN_UMRS2010_1_NUMBER		14								// 切替番号
#define IN_UMRS2010_2_NUMBER		15								// 切替番号
//-----------------------------------------------------------------------------------------------//


//-- [プロトタイプ宣言] -------------------------------------------------------------------------//
void message_display(unsigned int);									// メッセージ表示関数
void message_select(int, int*, int*, geometry_msgs::TwistStamped::ConstPtr,
	geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*,
	geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*,
	geometry_msgs::TwistStamped*, geometry_msgs::TwistStamped*);					// メッセージ更新処理関数
//-----------------------------------------------------------------------------------------------//


//-- [ヘッダファイル末尾] -----------------------------------------------------------------------//
#endif													// 条件コンパイル終了
//-----------------------------------------------------------------------------------------------//
