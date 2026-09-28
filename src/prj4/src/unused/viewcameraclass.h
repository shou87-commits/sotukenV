// この機能は使えない、SetCameraPose.h がなくなったため
// ViewCameraclass.h
// Gazebo の Window に表示される映像を撮影するカメラの設定

//#include <iostream>
#include <cmath>                   // M_PI を使用するため
//#include <vector>

#include <ros/ros.h>
#include <gazebo_msgs/SetCameraPose.h> // Gazeboのカメラポーズを設定するサービスメッセージ
// SetCameraPose.h は今はない
// WARNING: package 'gazebo_msgs' is deprecated (This package has been deprecated
//		      as of January 2025 with Gazebo classic 11 reaching
#include <geometry_msgs/Point.h>       // 位置のメッセージ
#include <geometry_msgs/Pose.h>        // ポーズ（位置と向き）のメッセージ
#include <geometry_msgs/Quaternion.h>  // geometry_msgs::Quaternion を使用するため
#include <tf/tf.h>                 // tf::Quaternion, tf::Matrix3x3, tf::quaternionMsgToTF を含む
#include <tf/transform_datatypes.h>    // ヨー・ピッチ・ロールからクォータニオンへの変換

#include "const.h"

class VCAMERA
{
public:
    VCAMERA()
    {
	p_x = p_y = p_z = 0.0;
	q_x = q_y = q_z = q_w = 0.0;
	o_roll = o_pitch = o_yaw = 0.0;
    }
    ~VCAMERA(){}

    void setvcampos(void)
    {
	setsrv.request.position.x = 3.0;
	setsrv.request.position.y = 3.0;
	setsrv.request..position.z = 2.0;

	// --- 設定するカメラの新しい向き（クォータニオン） ---
	// 例: 対象物（原点付近）を見るように設定
	// ここでは、ヨー・ピッチ・ロールからクォータニオンを作成
	// ロール=0, ピッチ= -30度 (下を向く), ヨー= -135度 (斜めから見る)
	double roll = 0.0;
	double pitch = -M_PI / 6.0;  // -30度をラジアンに変換
	double yaw = -3 * M_PI / 4.0; // -135度をラジアンに変換

	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	setsrv.request.orientation.x = q.x();
	setsrv.request.orientation.y = q.y();
	setsrv.request.orientation.z = q.z();
	setsrv.request.orientation.w = q.w();

// サービス呼び出しを試みる
	if (setc.call(setsrv)) {
	    if (setsrv.response.success) {
		ROS_INFO("Set VCAMERA position to x=%.2f, y=%.2f, z=%.2f",
			 setsrv.request.position.x,
			 setsrv.request.position.y,
			 setsrv.request.position.z);
	    }
	    else {
		ROS_ERROR("Failed to set VXAMERA position");
	    }
	}
    }
/*    
    void getvcampos(void)
    {
	if (getc.call(getsrv)) {
	    if (getsrv.response.success) {
		position = getsrv.response.pose.position;
		orientation = getsrv.response.pose.orientation;
// 物体の世界座標（位置）と向きを取得
		p_x = position.x;
		p_y = position.y;
		p_z = position.z - 0.05; // 木材を格納するための左右の壁の分、下げる
		q_x = orientation.x;
		q_y = orientation.y;
		q_z = orientation.z;
		q_w = orientation.w;
// 取得した座標を表示
		ROS_INFO("Model: %s", target_model_name.c_str()); 
		ROS_INFO("Position: x=%f, y=%f, z=%f", p_x, p_y, p_z);
		ROS_INFO("Quaternion: x=%f, y=%f, z=%f, w=%f", q_x, q_y, q_z, q_w);
	    }
	    else {
		ROS_WARN("Failed to get state for model '%s': %s",
			 target_model_name.c_str(), getsrv.response.status_message.c_str());
	    }
// geometry_msgs::Quaternion を tf::Quaternion に変換
	    tf::Quaternion q_tf;
	    tf::quaternionMsgToTF(orientation, q_tf);
	    q_tf.normalize(); // 正規化の警告が出たらこの行を実行
// tf::Quaternion から tf::Matrix3x3 を作成（オプションだが推奨される中間ステップ）
	    tf::Matrix3x3 m(q_tf);
// tf::Matrix3x3 からロール、ピッチ、ヨーを取得
	    double roll_rad, pitch_rad, yaw_rad;
	    m.getRPY(roll_rad, pitch_rad, yaw_rad); // ロール、ピッチ、ヨーは radian
	    o_roll = roll_rad * 180.0 / M_PI;
	    o_pitch = pitch_rad * 180.0 / M_PI;
	    o_yaw = yaw_rad * 180.0 / M_PI;
	    ROS_INFO("Euler Angles (Roll, Pitch, Yaw): (%.3f deg, %.3f deg, %.3f deg)",
		     o_roll, o_pitch, o_yaw);
	}
    }
*/
    float p_x, p_y, p_z;    
    float q_x, q_y, q_z, q_w;
    float o_roll, o_pitch, o_yaw;
private:
// /gazebo/get_model_state サービスのためのクライアントを作成
    ros::NodeHandle nh;
    ros::ServiceClient setc = nh.serviceClient<gazebo_msgs::SetCameraPose>("/gazebo/set_camera_pose");
    gazebo_msgs::SetCameraPose setsrv;
    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;
};
