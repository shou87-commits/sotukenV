// prg4.cpp

// 重要：プログラムが間違っていなくても時々不思議なエラーで動かない
//       ことがあるので、問題なさそうな時は２回以上起動してみる。

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/WrenchStamped.h>

#include "const.h"
#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h"
#include "lidarclass.h"

#include "ftclass.h"


// 次のポインタは、いくつかのデータを、オブジェクト外の AI の関数 
// intelligence() やシェルの関数からまとめてアクセスできるようにするため
C1 *gMotor;
LIDAR *gLIDAR;
RGBCam *gDepthRGB;
DepthCam *gDepthD;
FTSENSOR *gFT;

// 関数のプロトタイプ宣言
void monitorJointState(const sensor_msgs::JointState::ConstPtr& js);
void intelligence(const ros::TimerEvent&); // タイマにより 1/100[sec] 毎に呼び出される
void UserMotion(C1 m[], RGBCam *c, DepthCam *d); // intelligence() からそのまま呼び出される
void openshell(const ros::TimerEvent&);

int main(int argc, char **argv)
{
    float param[NUMPARAM];

    ros::init(argc, argv, "joint_controller");

// モータの角度を取得する subscriber の設定
// モータ角度の publisher の設定は、config/prg4.yaml の先頭にある
    ros::NodeHandle nh;
    ros::Subscriber p_sub = nh.subscribe("/prj4/joint_states", 128, monitorJointState);

// 次の行動を決める AI をタイマーにより周期的に呼び出す
    ros::Timer timer = nh.createTimer(ros::Duration(AIITVL), intelligence);

// 対話的に指示をできるシェルの起動関数をタイマーにより周期的に呼び出す
    ros::Timer timer2 = nh.createTimer(ros::Duration(AIITVL), openshell);

// カメラ画像を取り込むクラスのオブジェクト、中でコールバック関数が動く
// カメラ１台にインスタンスが１個必要   
    RGBCam ic1(IDHEADCAM, "/prj4/camera1/image_raw", "/prj4/image_topic1",
		       "Head Camera"); // head cam
    RGBCam ic2(IDHANDCAM, "/prj4/camera2/image_raw", "/prj4/image_topic2",
		       "Hand Camera"); // hand cam
    RGBCam ic3(IDDEPTHCAMCOLOR, "/depthcamera/color/image_raw", "/prj4/depthimage_topic1",
		       "Depth Camera Color");
// デプスカメラは別の処理
    DepthCam dic1(IDDEPTHCAMDEPTH, "/depthcamera/depth/image_raw",
			     "/prj4/depthimage_topic2", "Depth Camera Depth");

// LIDAR, レーザレンジファインダ、測距
    LIDAR lidar1("/prj4/laser/scan"); // 名前は urdf 中の topicName と同じにする必要あり

//Ftsensor
    FTSENSOR ftsensor[NUMFTSENSORS]= {
	FTSENSOR(FTA1J1, "ftsensor_lf1/raw"),
	FTSENSOR(FTA1J2, "ftsensor_lf2/raw"),
	FTSENSOR(FTA1J1, "ftsensor_rf1/raw"),
	FTSENSOR(FTA1J2, "ftsensor_rf2/raw"),
    };
// この後、各関節に１個ずつ、コントローラのインスタンスを順次生成する

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
    };
// 初期姿勢の設定は一連の動作の最初の動作として扱うように変更したので以下は不要になった
//  SetInitialPose(motor); // 最初の姿勢を指定する、歩行用
//  SetInitialPose2(motor); // 最初の姿勢の別バージョン、アームを周期的に動かす
//  IKInitialPose(motor); // IK を利用して最初の姿勢を指定する
//  UserInitialPose(motor); // ユーザ定義、最初の姿勢を指定する

// グローバルにアクセスできるようにするためポインタにアドレスを入れておく
    gMotor = motor;
    gLIDAR = &lidar1;
    gDepthRGB = &ic3;
    gDepthD = &dic1;
	gFT = ftsensor;

//ft確認用
	//gFT[A1J1].Ftkoredake(0);
	//printf("prg4 id=%p tail=%p f=%p \n",&(gFT[FTA1J1].id), &(gFT[FTA1J1].tail), gFT[FTA1J1].f);
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
    for (int i = 0; i < NUMALLJOINTS; i++) {
        gMotor[i].anglenow.data = js -> position[i]; // 配列に格納する、Nextage は注意
    }
/*
    static int t = 0;
    if (0 == t++ % 300) {
      for (int i = 0; i < NUMALLJOINTS; i++) {
        printf("%.2f ", js -> position[i]);
      }
      printf("\n");
    }
*/    
#ifdef MODEL_NEXTAGE
// 以下は Nextage 専用の処理、Nextage は腕が２本のため、右前脚の角度データは
// 左前脚の直後に現れる。これ以降のプログラムを６脚と Nextage で共通にするため、
// Nextage 右前脚の角度データを６脚の右前脚と同じ位置にコピーする。首も同様。
    for (int i = 8; i < 8+8; i++) { // Nextage の右腕のデータを６脚の右腕の位置に
        gMotor[i+16].anglenow.data = gMotor[i].anglenow.data; 
    }
    gMotor[48].anglenow.data = gMotor[16].anglenow.data; // 首、水平回転軸
    gMotor[49].anglenow.data = gMotor[17].anglenow.data; // 首、垂直回転軸
#endif
    return; // 表示が不要なのでここで戻る

    static int timer = 0;
    if (0 == timer % 100) { // １秒に１回表示する
        for (int i = 0; i < NUMALLJOINTS; i++) { // 全ての関節
//          for (int i = 0; i < 0+3; i++) { // 左前脚、肘まで
	    fprintf(stderr, "%.2f ", gMotor[i].anglenow.data);
	}
/*
        for (int i = 24; i < 24+3; i++) { // 右前脚、肘まで、左右の比較のため
	    fprintf(stderr, "%.2f ", gMotor[i].anglenow.data);
	}
*/
	fprintf(stderr, "\n");
    }
    timer++;
}

// 周期的に呼び出されるこの関数で行動計画を決めて、各関節の目標角度を指定する。
// ここで指定した目標角度は各関節のコントローラからモータに送られる。
void intelligence(const ros::TimerEvent&)
{
// この関数は１００回／秒、呼び出される。その都度、関節の目標角度を指定可能

    UserMotion(gMotor, gDepthRGB, gDepthD); // ユーザが作った関数を呼び出す

// やっていることが分かりにくければ、以下の数行のように角度を変えたい関節の目標角度を
// 直接指定してもよい。角度を変える必要がなければ、指示不要、それまでの角度が維持される。

//  gMotor[A1J3].ctrlmode = MODEPROFILE;        // 制御のモード選択、これはゆっくり
//  gMotor[A1J3].duration = duration;           // 動作時間
//  gMotor[A1J3].targetangle.data = M_PI / 2.0; // 左前脚の肘を９０度上に
}
