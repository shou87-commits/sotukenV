// Robotclass.h
// ロボット本体 prj4 の世界座標と向きを読み出す、設定する

#include <iostream>
#include <cmath>                   // M_PI を使用するため
#include <vector>

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h> // Gazeboのモデル状態を取得するサービスメッセージ
#include <gazebo_msgs/SetModelState.h> // Gazeboのモデル状態を設定するサービスメッセージ
#include <geometry_msgs/Point.h>       // 位置のメッセージ
#include <geometry_msgs/Pose.h>        // ポーズ（位置と向き）のメッセージ
#include <geometry_msgs/Quaternion.h>  // geometry_msgs::Quaternion を使用するため
#include <tf/tf.h>                 // tf::Quaternion, tf::Matrix3x3, tf::quaternionMsgToTF を含む
#include <tf/transform_datatypes.h>    // ヨー・ピッチ・ロールからクォータニオンへの変換

#include "const.h"

class ROBOT
{
public:
    ROBOT()
    {
        target_model_name = "prj4";
//      target_model_name = "gzclient_camera"; // できるはずだがGazeboのカメラを指定できない
//      target_model_name = "user_camera";     // できるはずだがGazeboのカメラを指定できない
	getsrv.request.model_name = target_model_name;
	model_state.model_name = target_model_name;
// 相対座標を取得したい場合は relative_entity_name を指定、世界座標の場合は下記
	getsrv.request.relative_entity_name = "world"; // or empty ""
	model_state.reference_frame = "world"; 	
	p_x = p_y = p_z = 0.0;
	q_x = q_y = q_z = q_w = 0.0;
	o_roll = o_pitch = o_yaw = 0.0;
    }
    ~ROBOT(){}

    void setrobotpos(float xyzrpy[6]) // meter,degree
    {
	model_state.pose.position.x = xyzrpy[0];
	model_state.pose.position.y = xyzrpy[1];
	model_state.pose.position.z = xyzrpy[2];
	tf::Quaternion q;
	q.setRPY(xyzrpy[3]*M_PI/180.0,
		 xyzrpy[4]*M_PI/180.0,
		 xyzrpy[5]*M_PI/180.0); // 引数はロール、ピッチ、ヨーの順, DEGREE->RADIAN
	model_state.pose.orientation.x = q.x();
	model_state.pose.orientation.y = q.y();
	model_state.pose.orientation.z = q.z();
	model_state.pose.orientation.w = q.w();

// サービスリクエストにモデルの状態を設定
	setsrv.request.model_state = model_state;

// サービス呼び出しを試みる
	if (setc.call(setsrv)) {
	    if (setsrv.response.success) {
//		ROS_INFO("Set model state for '%s' to x=%.2f, y=%.2f, z=%.2f",
//			 model_state.model_name.c_str(),
//			 model_state.pose.position.x,
//			 model_state.pose.position.y,
//			 model_state.pose.position.z);
	    }
	    else {
		ROS_ERROR("Failed to set model state for '%s': %s",
			  model_state.model_name.c_str(),
			  setsrv.response.status_message.c_str());
	    }
	}
    }
    
    void getrobotpos(float xyzrpy[6])
    {
	if (getc.call(getsrv)) {
	    if (getsrv.response.success) {
		position = getsrv.response.pose.position;
		orientation = getsrv.response.pose.orientation;
// 物体の世界座標（位置）と向きを取得
		p_x = position.x; xyzrpy[0] = p_x;
		p_y = position.y; xyzrpy[1] = p_y;
		p_z = position.z - 0.05; xyzrpy[2] = p_z;// 木材格納用の左右の壁の分、下げる
		q_x = orientation.x;
		q_y = orientation.y;
		q_z = orientation.z;
		q_w = orientation.w;
// 取得した座標を表示してみる
//		ROS_INFO("Model: %s", target_model_name.c_str()); 
//		ROS_INFO("Position: x=%f, y=%f, z=%f", p_x, p_y, p_z);
//		ROS_INFO("Quaternion: x=%f, y=%f, z=%f, w=%f", q_x, q_y, q_z, q_w);
	    }
	    else {
		ROS_WARN("Failed to get state for model '%s': %s",
			 target_model_name.c_str(), getsrv.response.status_message.c_str());
	    }
// geometry_msgs::Quaternion を tf::Quaternion に変換
	    tf::Quaternion q_tf;
	    tf::quaternionMsgToTF(orientation, q_tf);
//	    q_tf.normalize(); // 正規化の警告が出たらこの行を実行
// tf::Quaternion から tf::Matrix3x3 を作成（オプションだが推奨される中間ステップ）
	    tf::Matrix3x3 m(q_tf);
// tf::Matrix3x3 からロール、ピッチ、ヨーを取得
	    double roll_rad, pitch_rad, yaw_rad;
	    m.getRPY(roll_rad, pitch_rad, yaw_rad); // ロール、ピッチ、ヨーは radian
	    o_roll = roll_rad * 180.0 / M_PI;   xyzrpy[3] = o_roll; // DEGREE
	    o_pitch = pitch_rad * 180.0 / M_PI; xyzrpy[4] = o_pitch;
	    o_yaw = yaw_rad * 180.0 / M_PI;     xyzrpy[5] = o_yaw;
//	    ROS_INFO("Euler Angles (Roll, Pitch, Yaw): (%.3f deg, %.3f deg, %.3f deg)",
//		     o_roll, o_pitch, o_yaw);
	}
    }
    std::string target_model_name;
    float p_x, p_y, p_z;    
    float q_x, q_y, q_z, q_w;
    float o_roll, o_pitch, o_yaw;
private:
// /gazebo/get_model_state サービスのためのクライアントを作成
    ros::NodeHandle nh;
    ros::ServiceClient getc = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceClient setc = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::GetModelState getsrv;
    gazebo_msgs::SetModelState setsrv;
    gazebo_msgs::ModelState model_state; // 設定したいモデルの状態を格納するメッセージ

    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;
};
