//***********************************************************************************************//
//* select.cpp											*//
//***********************************************************************************************//


//-- [ヘッダファイル] ---------------------------------------------------------------------------//
#include "select.h"											// selectヘッダファイル読み込み
//-----------------------------------------------------------------------------------------------//


//-- [メッセージ表示関数] -----------------------------------------------------------------------//
//- void message_display(unsigned int message_number)						-//
//- 機能   : メッセージの表示を行う								-//
//- 引数   : (unsigned int) メッセージ番号							-//
//- 戻り値 : (void)         なし								-//
//-----------------------------------------------------------------------------------------------//
void message_display(unsigned int message_number)
{

	//-- [メッセージ処理] -------------------------------------------------------------------//

	switch(message_number){										// メッセージ番号チェック

		case PROGRAM_BEGINNING:									// メッセージ0

			printf("[ nityc/select              ]");					// メッセージ表示
			printf("[ Program Beginning                        ]");				// メッセージ表示
			printf("[ ---------------------------------------- ]\n");			// メッセージ表示
			fflush(stdout);									// バッファフラッシュ [printf表示用]
			break;										// ブレイク

		case PROGRAM_END:									// メッセージ1

			printf("[ nityc/select              ]");					// メッセージ表示
			printf("[ Program End                              ]");				// メッセージ表示
			printf("[ ---------------------------------------- ]\n");			// メッセージ表示
			fflush(stdout);									// バッファフラッシュ [printf表示用]
			break;										// ブレイク

		case INITIALIZATION_BEGINNING:								// メッセージ2

			printf("[ nityc/select              ]");					// メッセージ表示
			printf("[ Initialization Beginning                 ]");				// メッセージ表示
			printf("[ ---------------------------------------- ]\n");			// メッセージ表示
			fflush(stdout);									// バッファフラッシュ [printf表示用]
			break;										// ブレイク

		case INITIALIZATION_END:								// メッセージ3

			printf("[ nityc/select              ]");					// メッセージ表示
			printf("[ Initialization End                       ]");				// メッセージ表示
			printf("[ ---------------------------------------- ]\n");			// メッセージ表示
			fflush(stdout);									// バッファフラッシュ [printf表示用]
			break;										// ブレイク

		case THE_MAIN_PROGRAM_BEGINNING:							// メッセージ4

			printf("[ nityc/select              ]");					// メッセージ表示
			printf("[ The Main Program Beginning               ]");				// メッセージ表示
			printf("[ ---------------------------------------- ]\n");			// メッセージ表示
			fflush(stdout);									// バッファフラッシュ [printf表示用]
			break;										// ブレイク

		case THE_MAIN_PROGRAM_END:								// メッセージ5

			printf("[ nityc/select              ]");					// メッセージ表示
			printf("[ The Main Program End                     ]");				// メッセージ表示
			printf("[ ---------------------------------------- ]\n");			// メッセージ表示
			fflush(stdout);									// バッファフラッシュ [printf表示用]
			break;										// ブレイク

		case ROS:										// メッセージ6

			printf("[ nityc/select              ]");					// メッセージ表示
			printf("[ Initialization Completion                ]");				// メッセージ表示
			printf("[ ROS                                      ]\n");			// メッセージ表示
			fflush(stdout);									// バッファフラッシュ [printf表示用]
			break;										// ブレイク

		default:										// デフォルト

			break;										// ブレイク

	}

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//


//-- [メッセージ更新処理関数] -------------------------------------------------------------------//
//- void message_select(int number, int *twist_select_data, int *twist_out_flag,		-//
//-  geometry_msgs::TwistStamped::ConstPtr twist_data,						-//
//-  geometry_msgs::TwistStamped *twist_out_kohga3_1,						-//
//-  geometry_msgs::TwistStamped *twist_out_kohga3_2,						-//
//-  geometry_msgs::TwistStamped *twist_out_frigo_1,						-//
//-  geometry_msgs::TwistStamped *twist_out_frigo_2,						-//
//-  geometry_msgs::TwistStamped *twist_out_umrs2009_1,						-//
//-  geometry_msgs::TwistStamped *twist_out_umrs2009_2,						-//
//-  geometry_msgs::TwistStamped *twist_out_umrs2010_1,						-//
//-  geometry_msgs::TwistStamped *twist_out_umrs2010_2)						-//
//- 機能   : メッセージの更新処理を行う								-//
//- 引数   : (int)                                   切替番号格納変数				-//
//-          (int*)                                  切替データ格納配列ポインタ			-//
//-          (int*)                                  フラグ格納配列ポインタ			-//
//-          (geometry_msgs::TwistStamped::ConstPtr) メッセージデータポインタ			-//
//-          (geometry_msgs::TwistStamped*)          twist_out_kohga3_1メッセージポインタ	-//
//-          (geometry_msgs::TwistStamped*)          twist_out_kohga3_2メッセージポインタ	-//
//-          (geometry_msgs::TwistStamped*)          twist_out_frigo_1メッセージポインタ	-//
//-          (geometry_msgs::TwistStamped*)          twist_out_frigo_2メッセージポインタ	-//
//-          (geometry_msgs::TwistStamped*)          twist_out_umrs2009_1メッセージポインタ	-//
//-          (geometry_msgs::TwistStamped*)          twist_out_umrs2009_2メッセージポインタ	-//
//-          (geometry_msgs::TwistStamped*)          twist_out_umrs2010_1メッセージポインタ	-//
//-          (geometry_msgs::TwistStamped*)          twist_out_umrs2010_2メッセージポインタ	-//
//- 戻り値 : (void)                                  なし					-//
//-----------------------------------------------------------------------------------------------//
void message_select(int number, int *twist_select_data, int *twist_out_flag,
	geometry_msgs::TwistStamped::ConstPtr twist_data, geometry_msgs::TwistStamped *twist_out_kohga3_1,
	geometry_msgs::TwistStamped *twist_out_kohga3_2, geometry_msgs::TwistStamped *twist_out_frigo_1,
	geometry_msgs::TwistStamped *twist_out_frigo_2, geometry_msgs::TwistStamped *twist_out_umrs2009_1,
	geometry_msgs::TwistStamped *twist_out_umrs2009_2, geometry_msgs::TwistStamped *twist_out_umrs2010_1,
	geometry_msgs::TwistStamped *twist_out_umrs2010_2)
{

	//-- [メッセージ更新処理] ---------------------------------------------------------------//

	if(*twist_select_data == number){								// twist_out_kohga3_1

		twist_out_kohga3_1->header.stamp    = twist_data->header.stamp;				// タイムスタンプ格納
		twist_out_kohga3_1->header.frame_id = twist_data->header.frame_id;			// フレームID格納
		twist_out_kohga3_1->twist.linear.x  = twist_data->twist.linear.x;			// 速度ベクトルx成分格納
		twist_out_kohga3_1->twist.linear.y  = twist_data->twist.linear.y;			// 速度ベクトルy成分格納
		twist_out_kohga3_1->twist.linear.z  = twist_data->twist.linear.z;			// 速度ベクトルz成分格納
		twist_out_kohga3_1->twist.angular.x = twist_data->twist.angular.x;			// 角速度ベクトルx成分格納
		twist_out_kohga3_1->twist.angular.y = twist_data->twist.angular.y;			// 角速度ベクトルy成分格納
		twist_out_kohga3_1->twist.angular.z = twist_data->twist.angular.z;			// 角速度ベクトルz成分格納
		*twist_out_flag                     = ON;						// フラグチェック

	}

	if(*(twist_select_data + 1) == number){								// twist_out_kohga3_2

		twist_out_kohga3_2->header.stamp    = twist_data->header.stamp;				// タイムスタンプ格納
		twist_out_kohga3_2->header.frame_id = twist_data->header.frame_id;			// フレームID格納
		twist_out_kohga3_2->twist.linear.x  = twist_data->twist.linear.x;			// 速度ベクトルx成分格納
		twist_out_kohga3_2->twist.linear.y  = twist_data->twist.linear.y;			// 速度ベクトルy成分格納
		twist_out_kohga3_2->twist.linear.z  = twist_data->twist.linear.z;			// 速度ベクトルz成分格納
		twist_out_kohga3_2->twist.angular.x = twist_data->twist.angular.x;			// 角速度ベクトルx成分格納
		twist_out_kohga3_2->twist.angular.y = twist_data->twist.angular.y;			// 角速度ベクトルy成分格納
		twist_out_kohga3_2->twist.angular.z = twist_data->twist.angular.z;			// 角速度ベクトルz成分格納
		*(twist_out_flag + 1)               = ON;						// フラグチェック

	}

	if(*(twist_select_data + 2) == number){								// twist_out_frigo_1

		twist_out_frigo_1->header.stamp    = twist_data->header.stamp;				// タイムスタンプ格納
		twist_out_frigo_1->header.frame_id = twist_data->header.frame_id;			// フレームID格納
		twist_out_frigo_1->twist.linear.x  = twist_data->twist.linear.x;			// 速度ベクトルx成分格納
		twist_out_frigo_1->twist.linear.y  = twist_data->twist.linear.y;			// 速度ベクトルy成分格納
		twist_out_frigo_1->twist.linear.z  = twist_data->twist.linear.z;			// 速度ベクトルz成分格納
		twist_out_frigo_1->twist.angular.x = twist_data->twist.angular.x;			// 角速度ベクトルx成分格納
		twist_out_frigo_1->twist.angular.y = twist_data->twist.angular.y;			// 角速度ベクトルy成分格納
		twist_out_frigo_1->twist.angular.z = twist_data->twist.angular.z;			// 角速度ベクトルz成分格納
		*(twist_out_flag + 2)              = ON;						// フラグチェック

	}

	if(*(twist_select_data + 3) == number){								// twist_out_frigo_2

		twist_out_frigo_2->header.stamp    = twist_data->header.stamp;				// タイムスタンプ格納
		twist_out_frigo_2->header.frame_id = twist_data->header.frame_id;			// フレームID格納
		twist_out_frigo_2->twist.linear.x  = twist_data->twist.linear.x;			// 速度ベクトルx成分格納
		twist_out_frigo_2->twist.linear.y  = twist_data->twist.linear.y;			// 速度ベクトルy成分格納
		twist_out_frigo_2->twist.linear.z  = twist_data->twist.linear.z;			// 速度ベクトルz成分格納
		twist_out_frigo_2->twist.angular.x = twist_data->twist.angular.x;			// 角速度ベクトルx成分格納
		twist_out_frigo_2->twist.angular.y = twist_data->twist.angular.y;			// 角速度ベクトルy成分格納
		twist_out_frigo_2->twist.angular.z = twist_data->twist.angular.z;			// 角速度ベクトルz成分格納
		*(twist_out_flag + 3)              = ON;						// フラグチェック

	}

	if(*(twist_select_data + 4) == number){								// twist_out_umrs2009_1

		twist_out_umrs2009_1->header.stamp    = twist_data->header.stamp;			// タイムスタンプ格納
		twist_out_umrs2009_1->header.frame_id = twist_data->header.frame_id;			// フレームID格納
		twist_out_umrs2009_1->twist.linear.x  = twist_data->twist.linear.x;			// 速度ベクトルx成分格納
		twist_out_umrs2009_1->twist.linear.y  = twist_data->twist.linear.y;			// 速度ベクトルy成分格納
		twist_out_umrs2009_1->twist.linear.z  = twist_data->twist.linear.z;			// 速度ベクトルz成分格納
		twist_out_umrs2009_1->twist.angular.x = twist_data->twist.angular.x;			// 角速度ベクトルx成分格納
		twist_out_umrs2009_1->twist.angular.y = twist_data->twist.angular.y;			// 角速度ベクトルy成分格納
		twist_out_umrs2009_1->twist.angular.z = twist_data->twist.angular.z;			// 角速度ベクトルz成分格納
		*(twist_out_flag + 4)                 = ON;						// フラグチェック

	}

	if(*(twist_select_data + 5) == number){								// twist_out_umrs2009_2

		twist_out_umrs2009_2->header.stamp    = twist_data->header.stamp;			// タイムスタンプ格納
		twist_out_umrs2009_2->header.frame_id = twist_data->header.frame_id;			// フレームID格納
		twist_out_umrs2009_2->twist.linear.x  = twist_data->twist.linear.x;			// 速度ベクトルx成分格納
		twist_out_umrs2009_2->twist.linear.y  = twist_data->twist.linear.y;			// 速度ベクトルy成分格納
		twist_out_umrs2009_2->twist.linear.z  = twist_data->twist.linear.z;			// 速度ベクトルz成分格納
		twist_out_umrs2009_2->twist.angular.x = twist_data->twist.angular.x;			// 角速度ベクトルx成分格納
		twist_out_umrs2009_2->twist.angular.y = twist_data->twist.angular.y;			// 角速度ベクトルy成分格納
		twist_out_umrs2009_2->twist.angular.z = twist_data->twist.angular.z;			// 角速度ベクトルz成分格納
		*(twist_out_flag + 5)                 = ON;						// フラグチェック

	}

	if(*(twist_select_data + 6) == number){								// twist_out_umrs2010_1

		twist_out_umrs2010_1->header.stamp    = twist_data->header.stamp;			// タイムスタンプ格納
		twist_out_umrs2010_1->header.frame_id = twist_data->header.frame_id;			// フレームID格納
		twist_out_umrs2010_1->twist.linear.x  = twist_data->twist.linear.x;			// 速度ベクトルx成分格納
		twist_out_umrs2010_1->twist.linear.y  = twist_data->twist.linear.y;			// 速度ベクトルy成分格納
		twist_out_umrs2010_1->twist.linear.z  = twist_data->twist.linear.z;			// 速度ベクトルz成分格納
		twist_out_umrs2010_1->twist.angular.x = twist_data->twist.angular.x;			// 角速度ベクトルx成分格納
		twist_out_umrs2010_1->twist.angular.y = twist_data->twist.angular.y;			// 角速度ベクトルy成分格納
		twist_out_umrs2010_1->twist.angular.z = twist_data->twist.angular.z;			// 角速度ベクトルz成分格納
		*(twist_out_flag + 6)                 = ON;						// フラグチェック

	}

	if(*(twist_select_data + 7) == number){								// twist_out_umrs2010_2

		twist_out_umrs2010_2->header.stamp    = twist_data->header.stamp;			// タイムスタンプ格納
		twist_out_umrs2010_2->header.frame_id = twist_data->header.frame_id;			// フレームID格納
		twist_out_umrs2010_2->twist.linear.x  = twist_data->twist.linear.x;			// 速度ベクトルx成分格納
		twist_out_umrs2010_2->twist.linear.y  = twist_data->twist.linear.y;			// 速度ベクトルy成分格納
		twist_out_umrs2010_2->twist.linear.z  = twist_data->twist.linear.z;			// 速度ベクトルz成分格納
		twist_out_umrs2010_2->twist.angular.x = twist_data->twist.angular.x;			// 角速度ベクトルx成分格納
		twist_out_umrs2010_2->twist.angular.y = twist_data->twist.angular.y;			// 角速度ベクトルy成分格納
		twist_out_umrs2010_2->twist.angular.z = twist_data->twist.angular.z;			// 角速度ベクトルz成分格納
		*(twist_out_flag + 7)                 = ON;						// フラグチェック

	}

	//---------------------------------------------------------------------------------------//


	return;												// 戻り値 [正常終了]

}
//-----------------------------------------------------------------------------------------------//
