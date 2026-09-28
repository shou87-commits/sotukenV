// prg4.cpp

// 重要：プログラムが間違っていなくても時々不思議なエラーで動かない
//       ことがあるので、問題なさそうな時は２回以上起動してみる。

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/WrenchStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "const.h"

#include "user.h"

#include "robotclass.h"
#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h"
#include "lidarclass.h"
#include "ftclass.h"
#include "mstatesclass.h"
#include "armclass.h"
// #include "viewcameraclass.h" // 今は使えない、SetCameraPose.hがないため

// 次のポインタは、いくつかのデータを、オブジェクト外の AI の関数 
// intelligence() やシェルの関数からまとめてアクセスできるようにするため
C1 *gMotor;
LIDAR *gLIDAR;
RGBCam *gDepthRGB;
DepthCam *gDepthD;
FTSENSOR *gFT;
ARM *gArm;
ROBOT *gRobot;

// 関数のプロトタイプ宣言
void monitorJointState(const sensor_msgs::JointState::ConstPtr& js);
void intelligence(const ros::TimerEvent&); // タイマにより AIITVL [sec] 毎に呼び出される
void stoparmft(const ros::TimerEvent&);
template <typename TYPE>
void setparam(TYPE dst[], TYPE src[], int n);

//void UserMotion(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[]); // intelligence() から呼び出される
void openshell(const ros::TimerEvent&);
void flasksubCallback(const std_msgs::Float32MultiArray& array);
int  dispatch(float cmdary[], int &lastact);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_controller");

// モータの角度を取得する subscriber の設定
// モータ角度の publisher の設定は、config/prg4.yaml の先頭にある
    ros::NodeHandle nh;
    ros::Subscriber p_sub = nh.subscribe("/prj4/joint_states", 128, monitorJointState);

// Web, flask から届く指示を受け取る
    ros::Subscriber flasksub = nh.subscribe("/prj4/flask", 10, flasksubCallback);
    
// 次の行動を決める AI をタイマーにより周期的に呼び出す
    ros::Timer timer = nh.createTimer(ros::Duration(AIITVL), intelligence);

// 対話的に指示できるシェルの起動関数をタイマーにより周期的に呼び出す
    ros::Timer timer2 = nh.createTimer(ros::Duration(AIITVL), openshell);

// カメラ画像を取り込むクラスのオブジェクト、中でコールバック関数が動く
// カメラ１台にインスタンスが１個必要   
// 深度カメラを伴わない ic1, ic2 は今は使っていない
//  RGBCam ic1(IDHEADCAM, "/prj4/camera1/image_raw", "/prj4/image_topic1",
//		       "Head Camera"); // head cam
//  RGBCam ic2(IDHANDCAM, "/prj4/camera2/image_raw", "/prj4/image_topic2",
//		       "Hand Camera"); // hand cam
    RGBCam rgbcam[NUMRGBCAMS] = {
	RGBCam(IDDEPTHCAMCOLOR, "/depthcamera/color/image_raw", "/prj4/depthimage_topic1",
	       "Depth Camera Color"),
	RGBCam(IDDEPTHCAMCOLORSL, "/depthcameraSL/color/image_raw", "/prj4/depthimageSL_topic1",
	       "Depth CameraSL Color"),
	RGBCam(IDDEPTHCAMCOLORSR, "/depthcameraSR/color/image_raw", "/prj4/depthimageSR_topic1",
	       "Depth CameraSR Color"),
    };

// 1 台ずつ個別のインスタンスを生成する以下の起動方法は使わなくなった
/*  RGBCam icF(IDDEPTHCAMCOLOR, "/depthcamera/color/image_raw", "/prj4/depthimage_topic1",
		       "Depth Camera Color");
    RGBCam icSL(IDDEPTHCAMCOLORSL, "/depthcameraSL/color/image_raw", "/prj4/depthimageSL_topic1",
		       "Depth CameraSL Color");
    RGBCam icSR(IDDEPTHCAMCOLORSR, "/depthcameraSR/color/image_raw", "/prj4/depthimageSR_topic1",
		       "Depth CameraSR Color");
*/

// デプスカメラは別の処理
    DepthCam depthcam[NUMDEPTHCAMS] = {
        DepthCam(IDDEPTHCAMDEPTH, "/depthcamera/depth/image_raw", "/prj4/depthimage_topic2",
		 "Depth Camera Depth"),
	DepthCam(IDDEPTHCAMDEPTHSL, "/depthcameraSL/depth/image_raw", "/prj4/depthimageSL_topic2",
		 "Depth CameraSL Depth"),
	DepthCam(IDDEPTHCAMDEPTHSR, "/depthcameraSR/depth/image_raw", "/prj4/depthimageSR_topic2",
		 "Depth CameraSR Depth"),
    };
// 1 台ずつ個別のインスタンスを生成する以下の起動方法は使わなくなった
/*  DepthCam dicF(IDDEPTHCAMDEPTH, "/depthcamera/depth/image_raw",
			     "/prj4/depthimage_topic2", "Depth Camera Depth");
    DepthCam dicSL(IDDEPTHCAMDEPTHSL, "/depthcameraSL/depth/image_raw",
			     "/prj4/depthimageSL_topic2", "Depth CameraSL Depth");
    DepthCam dicSR(IDDEPTHCAMDEPTHSR, "/depthcameraSR/depth/image_raw",
			     "/prj4/depthimageSR_topic2", "Depth CameraSR Depth");
*/

// LIDAR, レーザレンジファインダ、測距
//  LIDAR lidar1("/prj4/laser/scan"); // 名前は urdf 中の topicName と同じにする必要あり

// FT(force and torque) Sensor, 力覚とトルクセンサ
    FTSENSOR ftsensor[NUMFTSENSORS] = {
	FTSENSOR(FTA1J1, "ftsensor_lf1/raw"), // 左前脚の指１内側
	FTSENSOR(FTA1J2, "ftsensor_lf2/raw"), // 左前脚の指２内側
	FTSENSOR(FTA4J1, "ftsensor_rf1/raw"), // 右前脚の指１内側
	FTSENSOR(FTA4J2, "ftsensor_rf2/raw"), // 右前脚の指２内側
	FTSENSOR(FT5,    "ftsensor_5/raw"),   // 背中
	FTSENSOR(FTA1S1, "ftsensor_a1s1/raw"), // 左前脚の指外側 
	FTSENSOR(FTA2S1, "ftsensor_a2s1/raw"), // 左中脚の指外側
	FTSENSOR(FTA3S1, "ftsensor_a3s1/raw"), // 左後脚の指外側
	FTSENSOR(FTA4S1, "ftsensor_a4s1/raw"), // 右前脚の指外側
	FTSENSOR(FTA5S1, "ftsensor_a5s1/raw"), // 右中脚の指外側
	FTSENSOR(FTA6S1, "ftsensor_a6s1/raw"), // 右後脚の指外側
    };

// arm, 脚を１本ずつ表現するオブジェクト
    ARM arm[NUMARMS] = {
	ARM(LF), // Left Front
	ARM(LM),
	ARM(LB),
	ARM(RF),
	ARM(RM),
	ARM(RB),
    };
    
// FTセンサの値により脚の動きを止めるチェックをタイマーにより周期的に呼び出す
    ros::Timer timerarm = nh.createTimer(ros::Duration(AIITVL), stoparmft);

// ロボットの世界座標用

    ROBOT robot;
    
// CMSTATESは今は使っていない
// CMSTATES, Class Model States, ワールド座標の読み出し
//  CMSTATES p1("/gazebo/model_states"); // 今は引数を省略してはいけない
//    CMSTATES p1; // その後これでよくなった
    
// この後、各関節に１個ずつ、コントローラのインスタンスを順次生成する

// 各関節の角度を Gazebo から受信する monitorJointState() のデータの並びは
// 次の C1 motor[] のインスタンス生成順ではないので注意する。正しくは URDF
// ファイルで各ジョイントに付けた名前 jointNN?? を辞書順に並べた順番に
// なる。よってインスタンス生成順と一致させたいときは名前の決め方に注意する。
// 関連するファイルは URDF と config/yaml ファイル

    float param[NUMPARAM];  // モータコントローラのパラメータ設定用配列

//  param[0] = MODENONE;    // 制御の仕方、シンプルな PID 制御のみ
    param[0] = MODEPROFILE; // 制御の仕方、プロファイルにより細かく目標角度を決める
    param[1] = 4;           // 動作を終える目標時間、動きを切り替える周期の初期値
    param[2] = 0;           // 目標角度の初期値

    C1 motor[NUMALLJOINTS] = {
	C1((char*)"/prj4/a1j1_pos_con/command", A1J1, param), // ArmLeft1 shoulder
	C1((char*)"/prj4/a1j2_pos_con/command", A1J2, param), // ArmLeft1 elbow1
	C1((char*)"/prj4/a1j3_pos_con/command", A1J3, param), // ArmLeft1 elbow2
	C1((char*)"/prj4/a1j4_pos_con/command", A1J4, param), // ArmLeft1 wrist1
	C1((char*)"/prj4/a1j5_pos_con/command", A1J5, param), // ArmLeft1 wrist2
	C1((char*)"/prj4/a1j6_pos_con/command", A1J6, param), // ArmLeft1 wrist3
	C1((char*)"/prj4/a1j7_pos_con/command", A1J7, param), // ArmLeft1 Hand Finger1
	C1((char*)"/prj4/a1j8_pos_con/command", A1J8, param), // ArmLeft1 Hnad Finger2

	C1((char*)"/prj4/a2j1_pos_con/command", A2J1, param), // ArmLeft2 shoulder
	C1((char*)"/prj4/a2j2_pos_con/command", A2J2, param), // ArmLeft2 elbow1
	C1((char*)"/prj4/a2j3_pos_con/command", A2J3, param), // ArmLeft2 elbow2
	C1((char*)"/prj4/a2j4_pos_con/command", A2J4, param), // ArmLeft2 wrist1
	C1((char*)"/prj4/a2j5_pos_con/command", A2J5, param), // ArmLeft2 wrist2
	C1((char*)"/prj4/a2j6_pos_con/command", A2J6, param), // ArmLeft2 wrist3
	C1((char*)"/prj4/a2j7_pos_con/command", A2J7, param), // ArmLeft2 Hand Finger1
	C1((char*)"/prj4/a2j8_pos_con/command", A2J8, param), // ArmLeft2 Hnad Finger2

	C1((char*)"/prj4/a3j1_pos_con/command", A3J1, param), // ArmLeft3 shoulder
	C1((char*)"/prj4/a3j2_pos_con/command", A3J2, param), // ArmLeft3 elbow1
	C1((char*)"/prj4/a3j3_pos_con/command", A3J3, param), // ArmLeft3 elbow2
	C1((char*)"/prj4/a3j4_pos_con/command", A3J4, param), // ArmLeft3 wrist1
	C1((char*)"/prj4/a3j5_pos_con/command", A3J5, param), // ArmLeft3 wrist2
	C1((char*)"/prj4/a3j6_pos_con/command", A3J6, param), // ArmLeft3 wrist3
	C1((char*)"/prj4/a3j7_pos_con/command", A3J7, param), // ArmLeft3 Hand Finger1
	C1((char*)"/prj4/a3j8_pos_con/command", A3J8, param), // ArmLeft3 Hnad Finger2

	C1((char*)"/prj4/a4j1_pos_con/command", A4J1, param), // ArmRight1 shoulder
	C1((char*)"/prj4/a4j2_pos_con/command", A4J2, param), // ArmRight1 elbow1
	C1((char*)"/prj4/a4j3_pos_con/command", A4J3, param), // ArmRight1 elbow2
	C1((char*)"/prj4/a4j4_pos_con/command", A4J4, param), // ArmRight1 wrist1
	C1((char*)"/prj4/a4j5_pos_con/command", A4J5, param), // ArmRight1 wrist2
	C1((char*)"/prj4/a4j6_pos_con/command", A4J6, param), // ArmRight1 wrist3
	C1((char*)"/prj4/a4j7_pos_con/command", A4J7, param), // ArmRight1 Hand Finger1
	C1((char*)"/prj4/a4j8_pos_con/command", A4J8, param), // ArmRight1 Hnad Finger2

	C1((char*)"/prj4/a5j1_pos_con/command", A5J1, param), // ArmRight2 shoulder
	C1((char*)"/prj4/a5j2_pos_con/command", A5J2, param), // ArmRight2 elbow1
	C1((char*)"/prj4/a5j3_pos_con/command", A5J3, param), // ArmRight2 elbow2
	C1((char*)"/prj4/a5j4_pos_con/command", A5J4, param), // ArmRight2 wrist1
	C1((char*)"/prj4/a5j5_pos_con/command", A5J5, param), // ArmRight2 wrist2
	C1((char*)"/prj4/a5j6_pos_con/command", A5J6, param), // ArmRight2 wrist3
	C1((char*)"/prj4/a5j7_pos_con/command", A5J7, param), // ArmRight2 Hand Finger1
	C1((char*)"/prj4/a5j8_pos_con/command", A5J8, param), // ArmRight2 Hnad Finger2

	C1((char*)"/prj4/a6j1_pos_con/command", A6J1, param), // ArmRight3 shoulder
	C1((char*)"/prj4/a6j2_pos_con/command", A6J2, param), // ArmRight3 elbow1
	C1((char*)"/prj4/a6j3_pos_con/command", A6J3, param), // ArmRight3 elbow2
	C1((char*)"/prj4/a6j4_pos_con/command", A6J4, param), // ArmRight3 wrist1
	C1((char*)"/prj4/a6j5_pos_con/command", A6J5, param), // ArmRight3 wrist2
	C1((char*)"/prj4/a6j6_pos_con/command", A6J6, param), // ArmRight3 wrist3
	C1((char*)"/prj4/a6j7_pos_con/command", A6J7, param), // ArmRight3 Hand Finger1
	C1((char*)"/prj4/a6j8_pos_con/command", A6J8, param), // ArmRight3 Hnad Finger2

	C1((char*)"/prj4/a7j1_pos_con/command", A7J1, param), // Neck 1
	C1((char*)"/prj4/a7j2_pos_con/command", A7J2, param), // Neck 2

	C1((char*)"/prj4/a8j1_pos_con/command", A8J1, param), // Hatch 1
	C1((char*)"/prj4/a8j2_pos_con/command", A8J2, param), // Hatch 2
    };
// 初期姿勢の設定は一連の動作の最初の動作として扱うように変更したので以下は不要になった
//  SetInitialPose(motor); // 最初の姿勢を指定する、歩行用
//  SetInitialPose2(motor); // 最初の姿勢の別バージョン、アームを周期的に動かす
//  IKInitialPose(motor); // IK を利用して最初の姿勢を指定する
//  UserInitialPose(motor); // ユーザ定義、最初の姿勢を指定する

// グローバルにアクセスできるようにするためポインタにアドレスを入れておく
    gMotor = motor;
//  gLIDAR = &lidar1;
//  gDepthRGB = &ic3; // old
//  gDepthD = &dic1;  // old
    gDepthRGB = rgbcam;
    gDepthD = depthcam;
    gFT = ftsensor;
    gArm = arm;
    gRobot = &robot;
    
// FT センサで脚、関節の動きを止める指示の例    
//  gMotor[A1J1]. setftflag(ON);
//  gArm[LF]. setftflag(OFF); // 止めない
    
//  intelligence() はタイマにより定期的に実行されるので main() には現れない
    
    ros::spin();
// シングルスレッドはやめて、以下の 2 行のマルチスレッドに変更することもできる
// ただしマルチスレッドにするとRGB画像と深度画像を両方同時に表示できない
// 逆に RGB 画像と深度画像を imshow() で両方同時に表示しなければマルチスレッド可
//  ros::AsyncSpinner spinner(6); // マルチスレッドのイベントハンドラ
//  spinner.start();

    ros::waitForShutdown();

    return 0;
}

// 角度データが届く度に呼び出されるこの関数で、各関節の現在の角度を記録する。
void monitorJointState(const sensor_msgs::JointState::ConstPtr& js)
{
// 各関節の角度を Gazebo から受信する monitorJointState() のデータの並びは
// main() 中の C1 motor[] のインスタンス生成順ではないので注意する。正しくは URDF
// ファイルで各ジョイントに付けた名前 jointNN?? を辞書順に並べた順番になる。
// よってインスタンス生成順と一致させたいときは名前の決め方に注意する。
// 関連するファイルは URDF と config/yaml ファイル

    for (int i = 0; i < NUMALLJOINTS; i++) {
        gMotor[i].anglenow.data = js -> position[i]; // 配列に格納する、Nextage は注意
    }

/*
    static int t = 0;
    float xyzrpy[6] = {0};
    if (0 == t % 100) {
	gRobot -> getrobotpos(xyzrpy);
    }
    if (0 == t++ % 1000) {
	gRobot -> setrobotpos(xyzrpy);
    }

    if (0 == t++ % 500) {
	for (int i = 0; i < NUMALLJOINTS; i++) {
	    std::cout << js -> name[i] << ' ' << std::fixed << std::setprecision(2) <<
		js -> position[i] << ' ';
	}
	std::cout << std::endl;
    }
*/

#ifdef MODEL_NEXTAGE
// 以下は Nextage 専用の処理、Nextage は腕が２本のため、右前脚の角度データは
// 左前脚の直後に現れる。これ以降のプログラムを６脚と Nextage で共通にするため、
// Nextage 右前脚の角度データを６脚の右前脚と同じ位置にコピーする。首も同様。
    for (int i = 8; i < 8+8; i++) { // Nextage の右腕のデータを６脚の右腕の位置に
        gMotor[i+16].anglenow.data = gMotor[i].anglenow.data; 
    }
    gMotor[A7J1].anglenow.data = gMotor[16].anglenow.data; // 首、水平回転軸
    gMotor[A7J2].anglenow.data = gMotor[17].anglenow.data; // 首、垂直回転軸
#endif

// 前方首に付いているカメラは３次元座標の計算に首の回転角度が必要なのでここで代入しておく    
    gDepthD[IDDEPTHCAMDEPTH].ha = gMotor[A7J1].anglenow.data;
    gDepthD[IDDEPTHCAMDEPTH].va = gMotor[A7J2].anglenow.data;

    return; // 表示が不要なのでここで戻る
/*
    static int timer = 0;
    if (0 == timer % 100) { // １秒に１回表示する
        for (int i = 0; i < NUMALLJOINTS; i++) { // 全ての関節
	    for (int i = 0; i < 0+3; i++) { // 左前脚、肘まで
		fprintf(stderr, "%.2f ", gMotor[i].anglenow.data);
	    }
	    for (int i = 24; i < 24+3; i++) { // 右前脚、肘まで、左右の比較のため
		fprintf(stderr, "%.2f ", gMotor[i].anglenow.data);
	    }
	    fprintf(stderr, "\n");
	}
    }
    timer++;
*/
}

// 周期的に呼び出されるこの関数で行動計画を決めて、各関節の目標角度を指定する。
// ここで指定した目標角度は各関節のコントローラからモータに送られる。
void intelligence(const ros::TimerEvent&)
{
//  printf("i");
// この関数は AICYCLE 回／秒、呼び出される。その都度、関節の目標角度を指定可能

    UserMotion(gMotor, gDepthRGB, gDepthD, gFT); // ユーザが作った関数を呼び出す

// やっていることが分かりにくければ、以下の数行のように角度を変えたい関節の目標角度を
// 直接指定してもよい。角度を変える必要がなければ、指示不要、それまでの角度が維持される。

//  gMotor[A1J3].ctrlmode = MODEPROFILE;        // 制御のモード選択、これはゆっくり
//  gMotor[A1J3].duration = duration;           // 動作時間
//  gMotor[A1J3].targetangle.data = M_PI / 2.0; // 左前脚の肘を９０度上に
}

void stoparmft(const ros::TimerEvent&)
{
// この関数は AICYCLE 回／秒、呼び出される。その都度、脚を止めるかチェックする

    for (int i = 0; i < NUMARMS; i++) {
	gArm[i]. ftstop();
    }
}

template <typename TYPE>
void setparam(TYPE dst[], TYPE src[], int n)
{
    for (int i = 0; i < n; i++) dst[i] = src[i];
}
