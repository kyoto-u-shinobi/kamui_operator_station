#-- [CMake設定] ---------------------------------------------------------------------------------#

# CMakeバージョン
cmake_minimum_required(VERSION 2.8.3)

#------------------------------------------------------------------------------------------------#


#-- [パッケージ設定] ----------------------------------------------------------------------------#

# パッケージ名
project(nityc)

# 依存CMakeパッケージ
# catkin			: ROSビルドシステムマクロパッケージ
# message_generation		: メッセージ生成パッケージ
# roscpp			: C++言語ライブラリパッケージ
# rospy				: Python言語ライブラリパッケージ
# std_msgs			: 標準メッセージパッケージ
# geometry_msgs			: 幾何学メッセージパッケージ
# sensor_msgs			: センサメッセージパッケージ
find_package(			catkin REQUIRED COMPONENTS
				message_generation
				roscpp
				rospy
				std_msgs
				geometry_msgs
				sensor_msgs)

#------------------------------------------------------------------------------------------------#


#-- [メッセージ設定] ----------------------------------------------------------------------------#

# メッセージ処理マクロ
# SelectStamped.msg		: 自作メッセージ
# RelativePoseStamped.msg	: 自作メッセージ
add_message_files(		FILES
				SelectStamped.msg
				RelativePoseStamped.msg)
	
# メッセージ生成マクロ
# std_msgs			: 標準メッセージパッケージ
# geometry_msgs			: 幾何学メッセージパッケージ
# sensor_msgs			: センサメッセージパッケージ
generate_messages(		DEPENDENCIES
				std_msgs
				geometry_msgs
				sensor_msgs)

#------------------------------------------------------------------------------------------------#


#-- [コンパイル設定] ----------------------------------------------------------------------------#

# CMakeマクロ
catkin_package()

# 挿入デレクトリ
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

# 実行ファイルターゲット
add_executable(select src/select/main.cpp src/select/select.cpp)

# ライブラリターゲット
target_link_libraries(select ${catkin_LIBRARIES})

# メッセージターゲット
add_dependencies(select ${PROJECT_NAME}_generate_messages_cpp)

#------------------------------------------------------------------------------------------------#
