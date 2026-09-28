// mstatesclass.h   Model States, Pose(姿勢データと速度)の読み出し

// see prj4/src/unused/modelstatessample.cpp
// データの名前や型を確認する
/*
$ rostopic echo -n 1 /gazebo/model_states
name: 
pose: 
    position: x, y, z
    orientation: x, y, z, w
twist: 
    linear: x, y, z
    angular: x, y, z
*/

#ifndef	HEADER_MSCLASS
#define	HEADER_MSCLASS

// #include <geometry_msgs/Pose.h> // 関係するがなくても大丈夫そう
// #include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include "const.h"

/* 以下は関係するヘッダがある場所
$ cd /opt/ros/noetic/include/geometry_msgs
$ cd /opt/ros/noetic/include/gazebo_msgs
*/

// 名前、位置(座標)、向き、並進速度、回転速度をまとめて記録する構造体
struct Mspose {
    std::string name;
    float position[XYZ], orientation[XYZW], rpy[RPY]; // 位置（座標）と向き
    float linear[XYZ], angular[XYZ];                  // 速度、並進と回転
};

float R2D(float rad);
float D2R(float degree);

class CMSTATES
{
private:
    ros::NodeHandle nh_; // この２行はここ、コンストラクタには置けない
    ros::Subscriber ms_subsc_; // model states
    int Msnumobj = 0; // Gazebo に登場するモデル(物体)の個数
    struct Mspose *Msptr; // 必要な個数分の配列を動的に確保する
    geometry_msgs::Quaternion gquat;
    
public:
    CMSTATES()
//  CMSTATES(const char tname[]) // 引数を省略すると実行されないので注意
    {
        ms_subsc_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",
					    100, &CMSTATES::MsCallback, this);
    }

    ~CMSTATES(){delete[] Msptr;}

private:
    void MsCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
	static int initflag = 0;
	if (0 == initflag) { // 最初の１回だけ実行、配列の確保
	    initflag = 1;
	    Msnumobj = msg -> name.size(); // 物体の数の確認
	    Msptr = new(std::nothrow) struct Mspose[Msnumobj]; // 個数分の構造体の確保
	}
	for (int i = 0; i < Msnumobj; i++) {
	    Msptr[i]. name = msg -> name[i];
	    Msptr[i]. position[X] = msg -> pose[i].position.x;
	    Msptr[i]. position[Y] = msg -> pose[i].position.y;
	    Msptr[i]. position[Z] = msg -> pose[i].position.z;

	    Msptr[i]. orientation[X] = msg -> pose[i].orientation.x;
	    Msptr[i]. orientation[Y] = msg -> pose[i].orientation.y;
	    Msptr[i]. orientation[Z] = msg -> pose[i].orientation.z;
	    Msptr[i]. orientation[W] = msg -> pose[i].orientation.w;
/*
	    Msptr[i]. rpy[R]   = msg -> twist[i].rpy.r; // これ rpy[] は存在しないので、
	    Msptr[i]. rpy[P]   = msg -> twist[i].rpy.p; // MsGetRPY() が呼ばれる度に
	    Msptr[i]. rpy[YAW] = msg -> twist[i].rpy.yaw; // xyzw から計算で求める
*/
	    Msptr[i]. linear[X] = msg -> twist[i].linear.x;
	    Msptr[i]. linear[Y] = msg -> twist[i].linear.y;
	    Msptr[i]. linear[Z] = msg -> twist[i].linear.z;

	    Msptr[i]. angular[X] = msg -> twist[i].angular.x;
	    Msptr[i]. angular[Y] = msg -> twist[i].angular.y;
	    Msptr[i]. angular[Z] = msg -> twist[i].angular.z;
	}
// ロボットや物体の座標と回転を表示してみる
	static int count = 0;
	if (0 == (count++ % 2000)) {
//	    MsPrint(MsName2Idx("prj4")); // ロボット
//	    for (int i = 0; i < Msnumobj; i++) MsPrint(i); // 全物体
	}
/**/
// 最初の実装
/*	std::string name = msg -> name[5];
        posex = msg -> pose[5].position.x;
        posey = msg -> pose[5].position.y;
        posez = msg -> pose[5].position.z;
*/
    }

public:
    void MsGetPos(float p[XYZ], const std::string name)
    {
	int idx = MsName2Idx(name);
	p[X] = Msptr[idx]. position[X];
	p[Y] = Msptr[idx]. position[Y];
	p[Z] = Msptr[idx]. position[Z];
    }

    void MsGetRPY(float p[RPY], const std::string name)
    {
	int idx = MsName2Idx(name);

	geometry_msgs::Quaternion gquat;
	gquat. x = Msptr[idx]. orientation[X];
	gquat. y = Msptr[idx]. orientation[Y];
	gquat. z = Msptr[idx]. orientation[Z];
	gquat. w = Msptr[idx]. orientation[W];
	double roll, pitch, yaw;
	MsGQuat2Rpy(roll, pitch, yaw, gquat);
	p[R] = roll;
	p[P] = pitch;
	p[YAW] = yaw;
/*	p[R]   = Msptr[idx]. rpy[R]; // rpy[] は存在しない
	p[P]   = Msptr[idx]. rpy[P];
	p[YAW] = Msptr[idx]. rpy[YAW]; */
    }

    void MsSetPos(const float p[XYZ], const std::string name)
    {
	int idx = MsName2Idx(name);
	Msptr[idx]. position[X] = p[X];
	Msptr[idx]. position[Y] = p[Y];
	Msptr[idx]. position[Z] = p[Z];
// Gazebo に設定する
	MsSetState(name, p, NULL, NULL, NULL);
    }

    void MsSetRPY(const float p[RPY], const std::string name)
    {
	int idx = MsName2Idx(name);

	geometry_msgs::Quaternion gquat;
	gquat. x = Msptr[idx]. orientation[X];
	gquat. y = Msptr[idx]. orientation[Y];
	gquat. z = Msptr[idx]. orientation[Z];
	gquat. w = Msptr[idx]. orientation[W];
	double roll, pitch, yaw;
	MsGQuat2Rpy(roll, pitch, yaw, gquat);
// 設定するのは xyzw なので rpy から xyzw に変換する必要あり
// なので上記の処理は逆でダメ
// Gazebo に設定する
	MsSetState(name, NULL, p, NULL, NULL);
    }

    void MsSetSpeedLinear(const float p[XYZ], const std::string name)
    {
	int idx = MsName2Idx(name);
	Msptr[idx]. linear[X] = p[X];
	Msptr[idx]. linear[Y] = p[Y];
	Msptr[idx]. linear[Z] = p[Z];
// Gazebo に設定する
	MsSetState(name, NULL, NULL, p, NULL);
    }

    void MsSetSpeedAngular(const float p[XYZ], const std::string name)
    {
	int idx = MsName2Idx(name);
	Msptr[idx]. angular[X] = p[X];
	Msptr[idx]. angular[Y] = p[Y];
	Msptr[idx]. angular[Z] = p[Z];
// Gazebo に設定する
	MsSetState(name, NULL, NULL, NULL, p);
    }

    void MsSetState(const std::string name, const float xyz[XYZ],const float rpy[RPY],
		    const float linear[XYZ], const float angular[XYZ])
    {
	int idx = MsName2Idx(name);

	geometry_msgs::Pose model_pose;
	if (NULL == xyz) { // 現状維持
	    model_pose.position.x = Msptr[idx]. position[X];
	    model_pose.position.y = Msptr[idx]. position[Y];
	    model_pose.position.z = Msptr[idx]. position[Z];
	}
	else { // 変更
	    model_pose.position.x = xyz[X];
	    model_pose.position.y = xyz[Y];
	    model_pose.position.z = xyz[Z];
	}
	if (NULL == rpy) {
	    model_pose.orientation.x = Msptr[idx]. orientation[X];
	    model_pose.orientation.y = Msptr[idx]. orientation[Y];
	    model_pose.orientation.z = Msptr[idx]. orientation[Z];
	    model_pose.orientation.w = Msptr[idx]. orientation[W];
	}
	else {
	    model_pose.orientation.x = 0.0; // rpy[] から求める
	    model_pose.orientation.y = 0.0;
	    model_pose.orientation.z = 0.0;
	    model_pose.orientation.w = 0.0;
	}
	
	geometry_msgs::Twist model_twist;
	model_twist.linear.x = Msptr[idx]. linear[X];
	model_twist.linear.y = Msptr[idx]. linear[Y];
	model_twist.linear.z = Msptr[idx]. linear[Z];
	model_twist.angular.x = Msptr[idx]. angular[X];
	model_twist.angular.y = Msptr[idx]. angular[Y];
	model_twist.angular.z = Msptr[idx]. angular[Z];
    }
    
///////////////////////////////////////////////////////////////////

    void MsGetRobotPos(float p[XYZ])
    {
	MsGetPos(p, "prj4");
    }

    void MsGetRobotRPY(float p[RPY])
    {
	MsGetRPY(p, "prj4");
    }

    void MsSetRobotPos(const float p[XYZ])
    {
	MsSetPos(p, "prj4");
    }

    void MsSetRobotRPY(const float p[RPY])
    {
	MsSetRPY(p, "prj4");
    }

///////////////////////////////////////////////////////////////////
    
    void MsGQuat2Rpy(double& roll, double& pitch, double& yaw,
		     const geometry_msgs::Quaternion gQuat)
    {
	tf::Quaternion tQuat;
	quaternionMsgToTF(gQuat, tQuat);
	tf::Matrix3x3(tQuat).getRPY(roll, pitch, yaw);  // rpy are Passed by Reference
    }

    void MsGetNameList(std::string names[])
    {
	for (int i = 0; i < Msnumobj; i++) names[i] = Msptr[i]. name;
    }

    int MsName2Idx(const std::string searchname)
    {
	for (int i = 0; i < Msnumobj; i++) {
	    if (searchname == Msptr[i]. name) return i;
	}
	std::cout << "No such name in namelist, MsName2Idx()" << std::endl;
	return -100;
    }

    void MsPrintAll(void)
    {
	for (int i = 0; i < Msnumobj; i++) MsPrint(i);
    }
    
    void MsPrint(const int i)
    {
	std::cout << Msptr[i]. name << " ";
	std::cout << std::fixed << std::setprecision(2) << Msptr[i]. position[X] << " ";
	std::cout << std::fixed << std::setprecision(2) << Msptr[i]. position[Y] << " ";
	std::cout << std::fixed << std::setprecision(2) << Msptr[i]. position[Z] << " ";
	MsGetRobotRPY(Msptr[i]. rpy);
	std::cout << std::fixed << std::setprecision(2) << R2D(Msptr[i]. rpy[R]) << " ";
	std::cout << std::fixed << std::setprecision(2) << R2D(Msptr[i]. rpy[P]) << " ";
	std::cout << std::fixed << std::setprecision(2) << R2D(Msptr[i]. rpy[YAW])
		  << " " << std::endl;
// radian による表示は以下
/*	std::cout << std::fixed << std::setprecision(2) << Msptr[i]. rpy[R] << " ";
	std::cout << std::fixed << std::setprecision(2) << Msptr[i]. rpy[P] << " ";
	std::cout << std::fixed << std::setprecision(2) << Msptr[i]. rpy[YAW]
		  << " " << std::endl;
*/
    }
};

#endif // HEADER_MSCLASS
