// action.cpp
// 起動、歩行、旋回、首振りなど、よく使う基本的な動きの関数群
// 練習用の関数は action2.cpp に移した

#include <cmath>
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

#include "const.h"
#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h"
#include "lidarclass.h"
#include "ftclass.h"
#include "image.h"

#include "user.h" // user*.cpp に含まれる関数のプロトタイプ宣言

// 定数の設定は const.h に移動した

// LIDAR 測距センサのデータにアクセスするためのポインタ
extern LIDAR *gLIDAR;

// gDepthRGB を使いたい時の書き方
extern RGBCam *gDepthRGB;
// extern RGBCam gDepthRGB[]; // この書き方ではダメ

int    Name2Num(char name[]);
float  R2D(float rad);
float  D2R(float deg);

// 初期姿勢の設定、UserInitialPose() と同じ内容
int ActInitialPose(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL];
    int cycletotal = (int)(duration*100); // 目標角度に達するまでの関数の呼び出し回数

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "start ActInitialPose()\n");
#endif
//	ikparam[0] = duration * 0.5; // 2 段階の動きに分けた、記述は下部に移した
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;

// 起動直後の脚の動きをゆっくりにしようと、以下を試したができなかった。
// 最初の動きはプログラムの制御下にないようだ
/*	
// ０番目の動き、脱力
	for (int i = 0; i < NUMALLJOINTS; i++) {
            m[i].ctrlmode = MODEPROFILE;
            m[i].duration = duration * 0.5; // 全体の動作時間の50%、他と足して1.0にする
            m[i].targetangle.data = 0.0;
        }
// ０番目の動き、逆運動学で脱力
	float initxyz[NUMARMS][XYZ] = {{ 0.15, 1,0}, { 0.0, 1,0}, {-0.15, 1,0},
				       { 0.15,-1,0}, { 0.0,-1,0}, {-0.15,-1,0}};
	ikparam[0] = duration * 0.5; // 全体の動作時間の50%、他と足して1.0にする
	IKMotions(m, LF, initxyz[0], ikparam);
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);
*/
    }    
// １番目の動き、脚先の位置を決める
    if (0 == (timer % cycletotal)) {
//  if ((int)(cycletotal * 0.5) == (timer % cycletotal)) { // 上記０番目の動き用
//	for (int i = 0; i < NUMJOINTS; i++) m[i].targetangle.data = 0; // 不要
	float initxyz[NUMARMS][XYZ] = {{ 0.5, 0.5,-0.1}, { 0.2, 0.5,-0.1}, {-0.5, 0.5,-0.1},
				       { 0.5,-0.5,-0.1}, { 0.2,-0.5,-0.1}, {-0.5,-0.5,-0.1}};

	ikparam[0] = duration * 0.5; // 2 段階の動きに分けたため
#ifdef MODEL_NEXTAGE
	initxyz[0][2] = 0.2; initxyz[3][2] = 0.2; // テーブルにぶつからないように下に下げない
#endif
	IKMotions(m, LF, initxyz[0], ikparam); // 必要なら返り値を調べて計算の問題の有無がわかる
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);
    }
// ２番目の動き、立ち上がる
    if ((int)(cycletotal * 0.5) == (timer % cycletotal)) {
	float initxyz[NUMARMS][XYZ] = {{ 0.5, 0.5,-0.3}, { 0.2, 0.5,-0.3}, {-0.5, 0.5,-0.3},
				       { 0.5,-0.5,-0.3}, { 0.2,-0.5,-0.3}, {-0.5,-0.5,-0.3}};
	ikparam[0] = duration * 0.5; // 2 段階の動きに分けたため
#ifdef MODEL_NEXTAGE
	initxyz[0][2] = 0.2; initxyz[3][2] = 0.2; // テーブルにぶつからないように下に下げない
#endif
	IKMotions(m, LF, initxyz[0], ikparam); // 必要なら返り値を調べて計算の問題の有無がわかる
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);
    }

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "end ActInitialPose()\n");
#endif
	return ACT_END;
    }
}

// ひっくり返ったときに向きを直す動き
int ActStandup(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL];
    int cycletotal = (int)(duration*100); // 目標角度に達するまでの関数の呼び出し回数

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "start ActStandup()\n");
#endif
	ikparam[0] = duration * 0.5; // ２段階の動きに分けたため
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
    }    
// １番目の動き
    if (0 == (timer % cycletotal)) {
//	for (int i = 0; i < NUMJOINTS; i++) m[i].targetangle.data = 0; // 不要
	float initxyz[NUMARMS][XYZ] = {{ 0.4, 0.2,-0.6}, { 0.1, 0.5,0.6}, {-0.1, 0.9,0.0},
				       { 0.4,-0.2,-0.6}, { 0.1,-0.5,0.6}, {-0.1,-0.9,0.0}};

// Nextage のテーブルにぶつからない動きは消した

//	IKMotions(m, LF, initxyz[0], ikparam); // 前脚は IK ではなく直接角度指定にした
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
//	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);

	char jn[][5] = {"a1j1", "a1j2", "a1j3", "a4j1", "a4j2", "a4j3"}; // joint name
	m[Name2Num(jn[0])].ctrlmode = MODEPROFILE;
	m[Name2Num(jn[0])].duration = duration * 0.5;
	m[Name2Num(jn[0])].targetangle.data = D2R(-90);
	m[Name2Num(jn[1])].ctrlmode = MODENONE;
	m[Name2Num(jn[1])].targetangle.data = D2R(-60);
	m[Name2Num(jn[2])].ctrlmode = MODENONE;
	m[Name2Num(jn[2])].targetangle.data = D2R(-30);
	m[Name2Num(jn[3])].ctrlmode = MODEPROFILE;
	m[Name2Num(jn[3])].duration = duration * 0.5;
	m[Name2Num(jn[3])].targetangle.data = D2R(90);
	m[Name2Num(jn[4])].ctrlmode = MODENONE;
	m[Name2Num(jn[4])].targetangle.data = D2R(60);
	m[Name2Num(jn[5])].ctrlmode = MODENONE;
	m[Name2Num(jn[5])].targetangle.data = D2R(30);
    }
// ２番目の動き
    if (cycletotal / 2 == (timer % cycletotal)) {
	float initxyz[NUMARMS][XYZ] = {{ 0.2, 0.7,-0.2}, { 0.0, 0.7,-0.2}, {-0.2, 0.7,-0.2},
				       { 0.2,-0.7,-0.2}, { 0.0,-0.7,-0.2}, {-0.2,-0.7,-0.2}};
	ikparam[0] = duration * 0.5; // ２段階の動きに分けたため
	IKMotions(m, LF, initxyz[0], ikparam); // 必要なら返り値を調べて計算の問題の有無がわかる
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);
    }

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "end ActStandup()\n");
#endif
	return ACT_END;
    }
}

// ひっくり返ったときに向きを直す動きの２
int ActStandup2(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL];
    int cycletotal = (int)(duration*100); // 目標角度に達するまでの関数の呼び出し回数

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "start ActStandup2()\n");
#endif
	ikparam[0] = duration * 0.5;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
    }    
// １番目の動き
    if (0 == (timer % cycletotal)) {
	float initxyz[NUMARMS][XYZ] = {{ 0.4, 0.2,-0.6}, { 0.1, 0.5,0.6}, {-0.1, 0.9,0.0},
				       { 0.4,-0.2,-0.6}, { 0.1,-0.5,0.6}, {-0.1,-0.9,0.0}};
/*
//	IKMotions(m, LF, initxyz[0], ikparam); // 前脚は IK ではなく直接角度指定にした
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
//	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);
*/
	char jn[][5] = {"a1j1", "a1j2", "a1j3", "a4j1", "a4j2", "a4j3"}; // joint name
	m[Name2Num(jn[0])].ctrlmode = MODEPROFILE;
	m[Name2Num(jn[0])].duration = duration * 0.5;
	m[Name2Num(jn[0])].targetangle.data = D2R(-90);
	m[Name2Num(jn[1])].ctrlmode = MODENONE;
	m[Name2Num(jn[1])].targetangle.data = D2R(60);
	m[Name2Num(jn[2])].ctrlmode = MODENONE;
	m[Name2Num(jn[2])].targetangle.data = D2R(-30);
	m[Name2Num(jn[3])].ctrlmode = MODEPROFILE;
	m[Name2Num(jn[3])].duration = duration * 0.5;
	m[Name2Num(jn[3])].targetangle.data = D2R(90);
	m[Name2Num(jn[4])].ctrlmode = MODENONE;
	m[Name2Num(jn[4])].targetangle.data = D2R(-60);
	m[Name2Num(jn[5])].ctrlmode = MODENONE;
	m[Name2Num(jn[5])].targetangle.data = D2R(30);
    }
    // ２番目の動き
    if (cycletotal / 2 == (timer % cycletotal)) {
	float initxyz[NUMARMS][XYZ] = {{ 0.2, 0.7,-0.2}, { 0.0, 0.7,-0.2}, {-0.2, 0.7,-0.2},
				       { 0.2,-0.7,-0.2}, { 0.0,-0.7,-0.2}, {-0.2,-0.7,-0.2}};
	ikparam[0] = duration * 0.5; // ２段階の動きに分けたため
	IKMotions(m, LF, initxyz[0], ikparam); // 必要なら返り値を調べて計算の問題の有無がわかる
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);

    }
    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "end ActStandup2()\n");
#endif
	return ACT_END;
    }
}

// ジャンプ、PIDのゲインPを300、<limit effort=200 velocity=10.14> でも跳ねなかったので開発中断 
int ActJump(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL];
    int cycletotal = (int)(duration*100); // 目標角度に達するまでの関数の呼び出し回数

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "start ActJump()\n");
#endif
	ikparam[0] = duration * 0.8;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODENONE;
        timer = 0;
    }    
// １番目の動き
    if (0 == (timer % cycletotal)) {
	float initxyz[NUMARMS][XYZ] = {{ 0.25, 0.3,-0.3}, { 0.0, 0.3,-0.3}, {-0.25, 0.3,-0.3},
				       { 0.25,-0.3,-0.3}, { 0.0,-0.3,-0.3}, {-0.25,-0.3,-0.3}};
	IKMotions(m, LF, initxyz[0], ikparam);
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);
    }
    // ２番目の動き
    if (cycletotal / 2 == (timer % cycletotal)) {
/*	float initxyz[NUMARMS][XYZ] = {{ 0.15, 0.2,-0.9}, { 0.0, 0.2,-0.9}, {-0.15, 0.2,-0.9},
				       { 0.15,-0.2,-0.9}, { 0.0,-0.2,-0.9}, {-0.15,-0.2,-0.9}};
	ikparam[0] = duration * 0.2; // ２段階の動きに分けたため
	ikparam[3] = (float)MODENONE;
	IKMotions(m, LF, initxyz[0], ikparam); // 必要なら返り値を調べて計算の問題の有無がわかる
	IKMotions(m, LM, initxyz[1], ikparam);
	IKMotions(m, LB, initxyz[2], ikparam);
	IKMotions(m, RF, initxyz[3], ikparam);
	IKMotions(m, RM, initxyz[4], ikparam);
	IKMotions(m, RB, initxyz[5], ikparam);
*/
	char jn[][5] = {"a1j2", "a1j3", "a2j2", "a2j3", "a3j2", "a3j3",
			"a4j2", "a4j3", "a5j2", "a5j3", "a6j2", "a6j3"}; // joint name
	m[Name2Num(jn[0])].ctrlmode = MODENONE;
	m[Name2Num(jn[0])].targetangle.data = D2R(-90);
	m[Name2Num(jn[1])].ctrlmode = MODENONE;
	m[Name2Num(jn[1])].targetangle.data = D2R(0);
	m[Name2Num(jn[2])].ctrlmode = MODENONE;
	m[Name2Num(jn[2])].targetangle.data = D2R(-90);
	m[Name2Num(jn[3])].ctrlmode = MODENONE;
	m[Name2Num(jn[3])].targetangle.data = D2R(0);
	m[Name2Num(jn[4])].ctrlmode = MODENONE;
	m[Name2Num(jn[4])].targetangle.data = D2R(-90);
	m[Name2Num(jn[5])].ctrlmode = MODENONE;
	m[Name2Num(jn[5])].targetangle.data = D2R(0);
	m[Name2Num(jn[6])].ctrlmode = MODENONE;
	m[Name2Num(jn[6])].targetangle.data = D2R(90);
	m[Name2Num(jn[7])].ctrlmode = MODENONE;
	m[Name2Num(jn[7])].targetangle.data = D2R(-0);
	m[Name2Num(jn[8])].ctrlmode = MODENONE;
	m[Name2Num(jn[8])].targetangle.data = D2R(90);
	m[Name2Num(jn[9])].ctrlmode = MODENONE;
	m[Name2Num(jn[9])].targetangle.data = D2R(-0);
	m[Name2Num(jn[10])].ctrlmode = MODENONE;
	m[Name2Num(jn[10])].targetangle.data = D2R(90);
	m[Name2Num(jn[11])].ctrlmode = MODENONE;
	m[Name2Num(jn[11])].targetangle.data = D2R(-0);
    }
    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "end ActJump()\n");
#endif
	return ACT_END;
    }
}

// 脚先を固定して本体を移動、平行移動
int ActBodyShift(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL];
    int cycletotal = (int)(duration*100); // 目標角度に達するまでの関数の呼び出し回数

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "start ActBodyShift()\n");
#endif
	ikparam[0] = duration;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
    }    
// １番目の動き
    if (0 == (timer % cycletotal)) {
	int flag[NUMARMS];
	decode_leg((int)p[AP_BSHIFTFLAG], flag);
	float target[XYZ] = {0};
	target[0] = p[AP_BSHIFTX];
	target[1] = p[AP_BSHIFTY];
	target[2] = p[AP_BSHIFTZ];
        if (ON == flag[0]) IKIncrementMotions(m, 1, target, ikparam); // 必要なら返り値を調べる
	if (ON == flag[1]) IKIncrementMotions(m, 2, target, ikparam); 
	if (ON == flag[2]) IKIncrementMotions(m, 3, target, ikparam); 
	if (ON == flag[3]) IKIncrementMotions(m, 4, target, ikparam);
	if (ON == flag[4]) IKIncrementMotions(m, 5, target, ikparam); 
	if (ON == flag[5]) IKIncrementMotions(m, 6, target, ikparam); 
    }

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "end ActBodyShift()\n");
#endif
	return ACT_END;
    }
}

// 脚先を固定して本体を移動、回転
int ActTilt(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL];
    int cycletotal = (int)(duration*100); // 目標角度に達するまでの関数の呼び出し回数

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "start ActTilt()\n");
#endif
	ikparam[0] = duration;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
    }    
// １番目の動き
    if (0 == (timer % cycletotal)) {
	int flag[NUMARMS];
	decode_leg((int)p[AP_BSHIFTFLAG], flag);
	float target[6][XYZ] = {{0}};
	if (0.0 != p[AP_BSHIFTX]) { // ピッチ、Y軸周りの回転
	    target[0][2] += p[AP_BSHIFTX];
	    target[2][2] -= p[AP_BSHIFTX];
	    target[3][2] += p[AP_BSHIFTX];
	    target[5][2] -= p[AP_BSHIFTX];
	}
	if (0.0 != p[AP_BSHIFTY]) { // ロール、X軸周りの回転
	    target[0][2] += p[AP_BSHIFTY];
	    target[1][2] += p[AP_BSHIFTY];
	    target[2][2] += p[AP_BSHIFTY];
	    target[3][2] -= p[AP_BSHIFTY];
	    target[4][2] -= p[AP_BSHIFTY];
	    target[5][2] -= p[AP_BSHIFTY];
	}
	if (0.0 != p[AP_BSHIFTZ]) { // ヨー、Z軸周りの回転
	    target[0][1] += p[AP_BSHIFTZ];
	    target[2][1] -= p[AP_BSHIFTZ];
	    target[3][1] += p[AP_BSHIFTZ];
	    target[5][1] -= p[AP_BSHIFTZ];
	}
        if (ON == flag[0]) IKIncrementMotions(m, 1, target[0], ikparam); // 必要なら返り値を調べる
	if (ON == flag[1]) IKIncrementMotions(m, 2, target[1], ikparam); 
	if (ON == flag[2]) IKIncrementMotions(m, 3, target[2], ikparam); 
	if (ON == flag[3]) IKIncrementMotions(m, 4, target[3], ikparam);
	if (ON == flag[4]) IKIncrementMotions(m, 5, target[4], ikparam); 
	if (ON == flag[5]) IKIncrementMotions(m, 6, target[5], ikparam); 
    }

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "end ActTilt()\n");
#endif
	return ACT_END;
    }
}

// 動きを止めて静止
int ActStay(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL];
    int cycletotal = (int)(duration*100); // 目標角度に達するまでの関数の呼び出し回数

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "start ActStay()\n");
#endif
	ikparam[0] = duration;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
    }    
// １番目の動き
    if (0 == (timer % cycletotal)) {
// nothing
    }

    timer++;
    p[AP_ELAPSEDTIME] = timer; // 追加機能の試験、1/100sec単位で経過時間がわかる
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "end ActStay()\n");
#endif
	return ACT_END;
    }
}

// 脱力、各関節の角度を0度にする
int ActRelax(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL];
    int cycletotal = (int)(duration*100); // 目標角度に達するまでの関数の呼び出し回数

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "start ActRelax()\n");
#endif
	ikparam[0] = duration;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
	for (int i = 0; i < NUMALLJOINTS; i++) {
	    m[i].ctrlmode = MODEPROFILE;
	    m[i].duration = 3.0;
	    m[i].targetangle.data = 0.0;
	}
    }    
// １番目の動き
    if (0 == (timer % cycletotal)) {
// nothing
    }

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC // 次行の表示が不要なら const.h で ACTEXEC をコメントアウト
	fprintf(stderr, "end ActRelax()\n");
#endif
	return ACT_END;
    }
}

// 歩行
int ActWalk(C1 m[], float p[])
{
    static int timer  = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
    float time1 = p[AP_TIME1]; // 動き１の時間、秒
    float time2 = p[AP_TIME2]; // 最後の着地の時間、秒
    int iter = p[AP_ITER]; // 繰り返し回数
    int cycletotal = (int)(duration*100); // 動作終了までに関数が呼び出される回数
    int cycle1 = (int)(time1*100); // 動き１の動作終了までに関数が呼び出される回数
    int phase = p[AP_PHASE]; // 動きの種類
    int dir = p[AP_WALK_DIR]; // 前進か後退か
    float HOHABAX = p[AP_HOHABAX]; // 脚のX座標 歩幅(m) 前後の脚が衝突しない限界 +-0.2m

// 本体重心からの各脚先の着地点、標準の値
    static float XF =  0.5, XM =  0.0, XB = -0.5;  // 脚のX座標 基準 Front, Middle, Back
    static float YF =  0.4, YM =  0.5, YB =  0.4;  // 脚のY座標 基準
    static float ZF = -0.3, ZM = -0.3, ZB = -0.3;  // 脚のZ座標 基準 
// 歩行時の４位相の標準着地点からの距離    
    float XF1 =  HOHABAX, XM1 =  HOHABAX, XB1 =  HOHABAX;  // 脚のX座標 歩幅前 
    float XF2 = -HOHABAX, XM2 = -HOHABAX, XB2 = -HOHABAX;  // 脚のX座標 歩幅後
    float YF1 =  0.0, YM1 =  0.0, YB1 =  0.0;  // 脚のY座標 横歩幅調整

    float ZU = 0.1,  ZD = -0.1;  // 脚の上下のZ座標 高低 Up, Down
    
//  static float UPZ = 0.1, DNZ = -0.3;  // 以前の脚の上下のZ座標 高低

    float target[NUMARMS][4][XYZ] = { // 直線歩行パターン
     {{XF+XF1, YF+YF1, ZF+ZU}, {XF+XF1, YF+YF1, ZF+ZD}, {XF+XF2, YF+YF1, ZF+ZD}, {XF+XF2, YF+YF1, ZF+ZU}},
     {{XM+XF2, YM+YM1, ZM+ZD}, {XM+XF2, YM+YM1, ZM+ZU}, {XM+XF1, YM+YM1, ZM+ZU}, {XM+XF1, YM+YM1, ZM+ZD}},
     {{XB+XF1, YB+YB1, ZB+ZU}, {XB+XF1, YB+YB1, ZB+ZD}, {XB+XF2, YB+YB1, ZB+ZD}, {XB+XF2, YB+YB1, ZB+ZU}},
     {{XF+XF2, -(YF+YF1), ZF+ZD}, {XF+XF2, -(YF+YF1), ZF+ZU}, {XF+XF1, -(YF+YF1), ZF+ZU}, {XF+XF1, -(YF+YF1), ZF+ZD}},
     {{XM+XF1, -(YM+YM1), ZM+ZU}, {XM+XF1, -(YM+YM1), ZM+ZD}, {XM+XF2, -(YM+YM1), ZM+ZD}, {XM+XF2, -(YM+YM1), ZM+ZU}},
     {{XB+XF2, -(YB+YB1), ZB+ZD}, {XB+XF2, -(YB+YB1), ZB+ZU}, {XB+XF1, -(YB+YB1), ZB+ZU}, {XB+XF1, -(YB+YB1), ZB+ZD}}
/* パラメータ化する前の脚の動き４位相ｘ６本
        {{ 0.7, 0.4,UPZ}, { 0.7, 0.4,DNZ}, { 0.3, 0.4,DNZ}, { 0.3, 0.4,UPZ}},
        {{-0.2, 0.5,DNZ}, {-0.2, 0.5,UPZ}, { 0.2, 0.5,UPZ}, { 0.2, 0.5,DNZ}},
	{{-0.3, 0.4,UPZ}, {-0.3, 0.4,DNZ}, {-0.7, 0.4,DNZ}, {-0.7, 0.4,UPZ}},
	{{ 0.3,-0.4,DNZ}, { 0.3,-0.4,UPZ}, { 0.7,-0.4,UPZ}, { 0.7,-0.4,DNZ}},
	{{ 0.2,-0.5,UPZ}, { 0.2,-0.5,DNZ}, {-0.2,-0.5,DNZ}, {-0.2,-0.5,UPZ}},
	{{-0.7,-0.4,DNZ}, {-0.7,-0.4,UPZ}, {-0.3,-0.4,UPZ}, {-0.3,-0.4,DNZ}}
*/
    }; 

//  fprintf(stderr, "(%d %d %d %d) ", timer, cycle1, phase, iter);
    
// 動作の指示はここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	ikparam[0] = time1;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = p[AP_CTRL];
        timer = 0;
    }
    if (0 == (timer % cycle1) && timer < (cycle1 * phase * iter)) {
	// それぞれの動きの時間が同じなら (timer % cycle1) の書き方ができる
      if (FORWARD == dir) {
	IKMotions(m, LF, target[0][timer/cycle1 % phase], ikparam);  // 必要なら返り値を調べる
	IKMotions(m, LM, target[1][timer/cycle1 % phase], ikparam);
	IKMotions(m, LB, target[2][timer/cycle1 % phase], ikparam);
	IKMotions(m, RF, target[3][timer/cycle1 % phase], ikparam);
	IKMotions(m, RM, target[4][timer/cycle1 % phase], ikparam);
	IKMotions(m, RB, target[5][timer/cycle1 % phase], ikparam);
      }
      else if (BACKWARD == dir) {
	IKMotions(m, LF, target[0][phase-1 - (timer/cycle1 % phase)], ikparam); 
	IKMotions(m, LM, target[1][phase-1 - (timer/cycle1 % phase)], ikparam);
	IKMotions(m, LB, target[2][phase-1 - (timer/cycle1 % phase)], ikparam);
	IKMotions(m, RF, target[3][phase-1 - (timer/cycle1 % phase)], ikparam);
	IKMotions(m, RM, target[4][phase-1 - (timer/cycle1 % phase)], ikparam);
	IKMotions(m, RB, target[5][phase-1 - (timer/cycle1 % phase)], ikparam);
      }
    }

    if (timer == (cycle1 * phase * iter)) { // 最後に浮いている脚を着地させる
	ikparam[0] = time2;
	int idx;
	if (FORWARD == dir)       idx = 3;
	else if (BACKWARD == dir) idx = 0;
	float tmpxyz[XYZ];
	tmpxyz[0] = target[0][idx][0]; tmpxyz[1] = target[0][idx][1]; tmpxyz[2] = ZF+ZD;
	IKMotions(m, LF, tmpxyz, ikparam);
	tmpxyz[0] = target[2][idx][0]; tmpxyz[1] = target[2][idx][1]; tmpxyz[2] = ZM+ZD;
	IKMotions(m, LB, tmpxyz, ikparam);
	tmpxyz[0] = target[4][idx][0]; tmpxyz[1] = target[4][idx][1]; tmpxyz[2] = ZB+ZD;
	IKMotions(m, RM, tmpxyz, ikparam);
    }
// ここまでの間に書く //////////////////////////////////

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
	return ACT_END;
    }
}

// 任意の脚を任意の座標に任意の位相数で動かす、任意回数繰返し可（現在、MAXPHASE=4）
int ActLegs(C1 m[], float p[], float target[NUMARMS][MAXPHASE][XYZ], int onflag[NUMARMS])
{
    static int timer  = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
    float time1 = p[AP_TIME1]; // 動き１の時間、秒、後の位相もすべて time1 秒を前提
    int iter = p[AP_ITER]; // 繰り返し回数
    int cycletotal = (int)(duration*100); // 動作終了までに関数が呼び出される回数
    int cycle1 = (int)(time1*100); // 動き１の動作終了までに関数が呼び出される回数
    int phase = p[AP_PHASE]; // 動きの種類

// 動作の指示はここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	ikparam[0] = time1;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = p[AP_CTRL];
        timer = 0;
    }
    if (0 == (timer % cycle1) && timer < (cycle1 * phase * iter)) { // 次の位相へ、かつ全体の実行時間未満
// それぞれの動きの時間が同じなら (timer % cycle1) の書き方ができる, LF=1
	if (ON == onflag[LF-1]) IKMotions(m, LF, target[LF-1][timer/cycle1 % phase], ikparam);
	if (ON == onflag[LM-1]) IKMotions(m, LM, target[LM-1][timer/cycle1 % phase], ikparam);
	if (ON == onflag[LB-1]) IKMotions(m, LB, target[LB-1][timer/cycle1 % phase], ikparam);
	if (ON == onflag[RF-1]) IKMotions(m, RF, target[RF-1][timer/cycle1 % phase], ikparam);
	if (ON == onflag[RM-1]) IKMotions(m, RM, target[RM-1][timer/cycle1 % phase], ikparam);
	if (ON == onflag[RB-1]) IKMotions(m, RB, target[RM-1][timer/cycle1 % phase], ikparam);
    }
// ここまでの間に書く //////////////////////////////////

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
	return ACT_END;
    }
}

// その場で旋回する
int ActTurn(C1 m[], float p[])
{
    static int timer  = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
    float time1 = p[AP_TIME1]; // 行動１の時間、秒
    int cycletotal = (int)(duration*100); // 動作終了までに関数が呼び出される回数
    int cycle1 = (int)(time1*100); // 動き１の動作終了までに関数が呼び出される回数
    int phase = p[AP_PHASE]; // 動きの種類、位相は４を想定
    static float target[NUMARMS][4][XYZ] = {0.0};

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	float org[NUMARMS][2] = {{ 0.5, 0.5}, { 0.0, 0.5}, {-0.5, 0.5},
				 { 0.5,-0.5}, { 0.0,-0.5}, {-0.5,-0.5}};
	float rot[NUMARMS][2] = {0.0};
	float theta = M_PI / 180 * p[AP_TURNDEG]; // 角度の計算
	float sign = p[AP_TURN_DIR];
    
	for (int i = 0; i < NUMARMS; i++) {
	    rot[i][0] = org[i][0] * cos(theta)        + org[i][1] * sin(theta) * sign * -1;
	    rot[i][1] = org[i][0] * sin(theta) * sign + org[i][1] * cos(theta);
	}
	for (int i = 0; i < NUMARMS; i++) {
	    for (int j = 0; j < 4; j++) { // 位相は４を想定
		if (0==i%2 && j<(4/2)) {
		    target[i][j][0] = rot[i][0];
		    target[i][j][1] = rot[i][1];
		    target[i][j][2] = 0==j%2?-0.2:-0.4;
		}
		if (1==i%2 && j<(4/2)) {
		    target[i][j][0] = org[i][0];
		    target[i][j][1] = org[i][1];
		    target[i][j][2] = 0==j%2?-0.4:-0.2;
		}
		if (0==i%2 && (4/2)<=j) {
		    target[i][j][0] = org[i][0];
		    target[i][j][1] = org[i][1];
		    target[i][j][2] = 0==j%2?-0.4:-0.2;
		}
		if (1==i%2 && (4/2)<=j) {
		    target[i][j][0] = rot[i][0];
		    target[i][j][1] = rot[i][1];
		    target[i][j][2] = 0==j%2?-0.2:-0.4;
		}
	    }
	}
	ikparam[0] = time1;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = p[AP_CTRL];
        timer = 0;
    }
    if (0 == (timer % cycle1)) { // それぞれの動きの時間が同じを想定、この書き方ができる
	IKMotions(m, LF, target[0][timer/cycle1 % phase], ikparam);  // 必要なら返り値を調べる
	IKMotions(m, LM, target[1][timer/cycle1 % phase], ikparam);
	IKMotions(m, LB, target[2][timer/cycle1 % phase], ikparam);
	IKMotions(m, RF, target[3][timer/cycle1 % phase], ikparam);
	IKMotions(m, RM, target[4][timer/cycle1 % phase], ikparam);
	IKMotions(m, RB, target[5][timer/cycle1 % phase], ikparam);
    }

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
	return ACT_END;
    }
}

// 首を動かす
int ActNeck(C1 m[], float p[])
{
    static int timer = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC
	fprintf(stderr, "start ActNeck()\n");
#endif
	m[A7J1].ctrlmode = MODENONE;
        m[A7J1].duration = duration;
	m[A7J2].ctrlmode = MODENONE;
        m[A7J2].duration = duration;
//	NeckDeg(m, p[AP_NECK_HA], p[AP_NECK_VA]); // degree 指定
	Neck(m, p[AP_NECK_HA], p[AP_NECK_VA]); // radian 指定

//	fprintf(stderr, "----- ActNeck() start %.2f %.2f %.2f %.2f\n", m[A7J1].targetangle.data,
//		m[A7J2].targetangle.data, m[A7J1].anglenow.data, m[A7J2].anglenow.data);

	timer = 0;
    }    

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC
	fprintf(stderr, "end ActNeck()\n");
#endif
//	fprintf(stderr, "----- ActNeck() end  %.2f %.2f %.2f %.2f\n", m[A7J1].targetangle.data,
//		m[A7J2].targetangle.data, m[A7J1].anglenow.data, m[A7J2].anglenow.data);
	return ACT_END;
    }
}

// 腹下のハッチを開閉する
int ActHatch(C1 m[], float p[])
{
    static int timer = 0;
    static float f = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC
	fprintf(stderr, "start ActHatch()\n");
#endif
	f = (0.0 == f) ? 1.57 : 0.0;
	
	m[A8J1].ctrlmode = MODENONE;
        m[A8J1].duration = duration;
	m[A8J1].targetangle.data = p[AP_HATCH_LEFT];
//	m[A8J1].targetangle.data = f;
	m[A8J2].ctrlmode = MODENONE;
        m[A8J2].duration = duration;
	m[A8J2].targetangle.data = p[AP_HATCH_RIGHT];
//	m[A8J2].targetangle.data = f * -1;
	timer = 0;
    }    
    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC
	fprintf(stderr, "end ActHatch()\n");
#endif
	return ACT_END;
    }
}

// ハンドを開く、閉じる
int ActHand(C1 m[], float p[])
{
    static int timer = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

    // void Hand(C1 m[], int place, int move, float rad);
    
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC
	fprintf(stderr, "start ActHand()\n");
#endif
//	HandDeg(m, p[AP_HANDNUM], p[AP_HANDSTATE], R2D(p[AP_HANDRAD])); // degree 指定
	Hand(m, p[AP_HANDNUM], p[AP_HANDSTATE], p[AP_HANDRAD]); // radian 指定
        timer = 0;
    }    

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC
	fprintf(stderr, "end ActHand()\n");
#endif
	return ACT_END;
    }
}

// LiDAR センサで前方を計測する
int ActLidar(C1 m[], float p[])
{
    static int timer = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
//  float hangle = p[AP_NECK_HA] * -1??? / 180.0 * M_PI; // 首振り左右目標角度(degree -> radian)
    float vangle = p[AP_NECK_VA] * -1 / 180.0 * M_PI; // 首振り上下目標角度(degree -> radian)
    int cycletotal = (int)(duration*100);

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) {
#ifdef ACTEXEC
	fprintf(stderr, "start ActLidar()\n");
#endif
	m[A7J1].ctrlmode = MODEPROFILE; 
	m[A7J1].duration = duration;
	m[A7J1].targetangle.data = 0; // hangle; // 首を左右に振る関節
	m[A7J2].ctrlmode = MODEPROFILE; 
	m[A7J2].duration = duration;
	m[A7J2].targetangle.data = vangle; // 首を上下に振る関節
        timer = 0;
    }

    timer++;
    if (timer < cycletotal) { // 首振り継続中
	p[AP_STATE] = ACT_MOVING; // 動作中
	return ACT_MOVING;
    }
    else if (timer == cycletotal) { // 最後に LIDAR で得たデータを処理してローカル座標を求める
	float x, y, z;
	for (int i = 0; i < gLIDAR -> numdata; i++) {
	    if (0.0 < gLIDAR -> distance[i] && gLIDAR -> distance[i] < 3.0) {
		gLIDAR -> Lidar2Xyz(gLIDAR -> distance[i], i, vangle, &x, &y, &z); // 座標変換
		printf("Dist:%.2f H:%d V:%.1f x:%.2f y:%.2f z:%.2f\n",
		       gLIDAR -> distance[i], i-60, p[AP_NECK_VA], x, y, z);
	    }
	}
	printf("\n");
	return ACT_MOVING;        // 動作中
    }
    else {
	timer = 0;   // タイマをクリアして
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC
	fprintf(stderr, "end ActLidar()\n");
#endif
	return ACT_END; // 動作終了
    }
}

// 垂直方向のスキャンを行うえる、Lidar センサで距離を測る
int ActLidarVScan(C1 m[], float p[])
{
    static int timer = 0;
    static float vangledeg = 0; // 首振り上下開始角度(degree)
//  float vangle = vangledeg / 180.0 * M_PI * -1; // 首振り上下目標角度(degree -> radian)
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);
    static int scany = 0;
    
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) {
/*
	m[A7J1].ctrlmode = MODENONE; 
	m[A7J1].duration = duration;
	m[A7J1].targetangle.data = 0; // 首を左右に振る関節
	m[A7J2].ctrlmode = MODENONE; 
	m[A7J2].duration = duration;
	m[A7J2].targetangle.data = vangle; // 首を上下に振る関節
*/
	for (int i = 0; i < LIDAR_BUFY; i++) {
	    for (int j = 0; j < LIDAR_BUFX; j++) {
		gLIDAR -> dist2D[i][j] = 0.0;
	    }
	}

	vangledeg = p[AP_LIDAR_VUP];
	scany = 0;
        timer = 0;
    }

    timer++;
    if (timer < cycletotal) { // SCAN 継続中

	static int timer2 = 0;
	static int substate = 0;               // 動作の状態を表す、継続中か、終了か
	static float sub_p[AP_PARAMS] = {0.0}; // パラメータの受け渡し用配列
	float subduration = p[AP_TIME1];
	int subtotal = (int)(subduration*100); 

	float vangle = vangledeg / 180.0 * M_PI * -1; // 首振り上下目標角度(degree -> radian)
	
	if (ACT_START == (int)sub_p[AP_STATE] || 0 == timer2) {
	    m[A7J1].ctrlmode = MODENONE; 
	    m[A7J1].duration = subduration;
	    m[A7J1].targetangle.data = 0; // 首を左右に振る関節
	    m[A7J2].ctrlmode = MODENONE; 
	    m[A7J2].duration = subduration;
	    m[A7J2].targetangle.data = vangle; // 首を上下に振る関節
	    timer2 = 0;
	}
	if (0 == timer2) {
//	if (0 == (timer2 % subtotal)) {
	    sub_p[AP_STATE] = ACT_START;
	    sub_p[AP_TIMETOTAL] = p[AP_TIME1]; // 動作時間 (sec)
	    sub_p[AP_NECK_HA] = +0.0;  // 左右首振り目標角度 (degree)
	    sub_p[AP_NECK_VA] = vangle; // 上下首振り目標角度 (degree)
	    substate = ActLidar(m, sub_p);
	}
	timer2++;
	if (timer2 < subtotal) { // 首振り継続中
	    sub_p[AP_STATE] = ACT_MOVING;
//	    return ACT_MOVING;
	}
	if (timer2 == subtotal) { // 計測データの取り出し
	    for (int x = 0; x < gLIDAR -> numdata; x++) {
		gLIDAR -> dist2D[scany][x] = gLIDAR -> distance[x];
	    }
	    sub_p[AP_STATE] = ACT_MOVING;
//	    return ACT_MOVING;
	}
        else { // 1回分のSCAN終了
	    vangledeg -= 1.0; // SCAN の角度を１度下げる
	    scany++;
	    timer2 = 0;   // タイマをクリア
	    sub_p[AP_STATE] = ACT_END;
//          return ACT_END;
	}

	p[AP_STATE] = ACT_MOVING; // 動作中
	return ACT_MOVING;
    }
    else {
	FILE *fp;
	if ((FILE *)NULL == (fp = fopen("/tmp/lidar.pgm", "w"))) {
	    fprintf(stderr, "Error: [%s] can not open\n", "/tmp/lidar.pgm");
	    exit(1);
	}
	fprintf(fp, "P5\n%d %d\n255\n", gLIDAR -> numdata, scany);
	for (int i = 0; i < scany; i++) {
	    for (int j = 0; j < gLIDAR -> numdata; j++) {
		unsigned char d;
		if (2.0 < gLIDAR -> dist2D[i][j]) d = 0;
		else d = (1.0 - gLIDAR -> dist2D[i][j] / 2.0) * 255;
		fwrite(&d, sizeof(unsigned char), 1, fp);
	    }
	}
	fclose(fp);

	timer = 0;   // タイマをクリアして
	p[AP_STATE] = ACT_END;
	return ACT_END; // 動作終了
    }
}

// カラー画像、深度画像を計測するセンサの画像から色や輪郭で物体を検出、ローカル座標を求める
int ActVision(C1 m[], RGBCam *c, DepthCam *d, float p[])
{
    static int timer = 0;
    static int count = 0;
//  static float ikparam[IKNUMPARAM]; // ここでは不使用になった
    char fname[64];
    float duration = p[AP_TIMETOTAL]; 
    int cycletotal = (int)(duration*100);

// 動作の指示は ★★★ ここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC
	fprintf(stderr, "start ActVision()\n");
#endif
        timer = 0;
    }
    if (0 == (timer % cycletotal)) {

//	fprintf(stderr, "ActVision() start %.2f %.2f %.2f %.2f\n", m[A7J1].targetangle.data,
//              m[A7J2].targetangle.data, m[A7J1].anglenow.data, m[A7J2].anglenow.data);
	
	float x, y, z;            // 計算後のローカル座標を記憶する変数
	float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要
	float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要
//	float ha = m[A7J1].targetangle.data; // 目標角度
//	float va = m[A7J2].targetangle.data; // 目標角度
//	float ha = p[AP_NECK_HA]; // 首の水平回転角度(radian)、正しくセットされてない可能性がある
//	float va = p[AP_NECK_VA]; // 首の垂直回転角度(radian)、正しくセットされてない可能性がある

//////////// 画像処理の例 /////////////////////////////////
// 画像処理をして、必要なら更に自分で画像を調べて、注目座標を ix, iy にセットする

//      cv_bridge::CvImagePtr p = c -> cv_ptr;  // 原画像の構造体へのポインタ
//      cv::Mat in = p -> image;
//	cv::Mat in_c = c -> cv_ptr -> image; // 上記2行はこの1行にまとめた
//	cv::Mat in_d = d -> cv_ptr -> image; // 距離画像の場合はこちら
	int filter = (int)p[AP_FILTER]; // LAPLACIAN など
	int imgORcol = (int)p[AP_IMGORCOL]; // DEPTH など
	cv::Mat in;
	if (DEPTH == imgORcol) in = d -> cv_ptr -> image; // DEPTH
	else                   in = c -> cv_ptr -> image; // COLOR(RGB), RED, GREEN, ...
	int objs;                            // 見つけた領域、物体の個数
// 次の2個の配列はカメラオブジェクトに持たせるため c -> fareacog, c -> fareacorners に移した
//	int cog[MAXOBJS][XY];                // 重心、Center of Gravity
//	int corners[MAXOBJS][LURL][XY];      // 外接長方形の左上と右下角の座標

// 次の処理で画像処理を行い、条件に合う領域を検出する
// 求めた個々の領域に対して重心と外接長方形を求める		
	objs = imFindObjs(&in, filter, imgORcol, SHOW, "imFindObjs",
			  c -> fareacog, c -> fareacorners);
/*
// 他の画像処理の例、書き方が古いので直前の行の書き方を真似する
//
// (1) 指定した色の領域(物体)を抽出する
	objs = imFindObjs(&in_c, FOC, RED, SHOW, "ColorObjsResult", cog, corners);
// (2) カラー画像から Sobel フィルタで輪郭を求める
	objs = imFindObjs(&in_c, SOBEL, COLOR, SHOW, "SobelColorResult");
// (3) 距離画像から Sobel フィルタで輪郭を求める
	objs = imFindObjs(&in_d, SOBEL, DEPTH, SHOW, "SobelDepthResult");
// (4) カラー画像から Laplacian フィルタで輪郭を求める、個々の領域に対して重心と外接長方形を求める
	objs = imFindObjs(&in_c, LAPLACIAN, COLOR, SHOW, "LaplacianColorResult", cog, corners);
// (5) 距離画像から Laplacian フィルタで輪郭を求める、個々の領域に対して重心と外接長方形を求める
	objs = imFindObjs(&in_d, LAPLACIAN, DEPTH, SHOW, "LaplacianDepthResult", cog, corners);
*/

	c -> fareanum = objs; // 検出した領域の個数
	d -> fobjnum  = objs; // 検出した物体の個数、両者は同じ値になる

	if (0 < objs) {        // もし注目領域、物体があれば
	    int a[MAXOBJS*2];  // x, y なので2倍の要素数が必要
	    int n = 0;
	    for (int i = 0; i < objs; i++) {
		a[n++] = c -> fareacog[i][0]; // x 座標、画像中に + マークを表示するため
		a[n++] = c -> fareacog[i][1]; // y 座標
// カメラ画像の window で注目する画素の位置（重心）に + マークを描画する、不要なら省略可
		c -> SetMark(objs, a);
// 重要、次の処理で画像中の座標から3次元ローカル座標を求める
		d -> LocalXyz(c -> fareacog[i][0], c -> fareacog[i][1], &x, &y, &z, ha, va);
// 検出した個々の物体の3次元ローカル座標を配列に記憶する
		d -> fobjxyz[i][0] = x;
		d -> fobjxyz[i][1] = y; // depthclass.h 中の宣言で float fobjxyz[MAXOBJS][XYZ];
		d -> fobjxyz[i][2] = z; // となっていることを要確認、もし最後が [XY] なら要修正
	    }
//	    c -> ResetMark(); // 登録したマークを削除して描画をやめるとき
	    p[AP_OBJFIND] = objs; // 物体発見のフラグ、見つけた個数
	}
	else {
	    p[AP_OBJFIND] = 0;  // 物体を見つけなかった
	}
// 以下を実行すると画像をファイルに書き出す、不要なら実行しなくて可
//	int a[] = {319, 239, 319, 357, 198, 239, 198, 357, 190, 116}; // 例、５個の座標
//	sprintf(fname, "rgb%d.ppm", count++); // カラー画像、ファイル名の生成
//	c -> RGBsavemark(fname, 5, a); // マーク付きでRGB画像を保存
//	sprintf(fname, "d%d.pgm", count++); // 距離画像、ファイル名の生成
//	d -> Dsave(fname); // depth画像を保存
    }
// ここ ★ ★ ★ までの間に書く //////////////////////////////////

    timer++;
    if (timer < cycletotal) { // 動作継続中の処理、次行のように書いてはいけない
//  if (timer <= cycletotal) { // ★ 重要、これは間違い、１回多く実行され致命的なことも
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // 動作終了、タイマをクリアして ACT_END を返り値に
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC
	fprintf(stderr, "end ActVision()\n");
#endif
	return ACT_END;
    }
}

// 物体を掴む、持ち上げる、落とす
int ActGrabRelease(C1 m[], DepthCam *d, float p[])
{
    static int timer  = 0; // 関数が呼び出された回数をカウントする
    static float ikparam[IKNUMPARAM];
    static float x, y, z; // 掴む対象となる物体の座標

    float duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
    float time1 = p[AP_TIME1]; // 動き１の時間、秒
    float time2 = p[AP_TIME2]; // 動き２の時間、秒
    float time3 = p[AP_TIME3]; // 動き３の時間、秒
    float time4 = p[AP_TIME4];
    float time5 = p[AP_TIME5];
    int cycletotal = (int)(duration*100); // 動作終了までに関数が呼び出される回数
    int cycle1 = (int)(time1*100); // 動き１の動作終了までに関数が呼び出される回数
    int cycle2 = (int)(time2*100); // 動き２の動作終了までに関数が呼び出される回数
    int cycle3 = (int)(time3*100); // 動き３の動作終了までに関数が呼び出される回数
    int cycle4 = (int)(time4*100);
    int cycle5 = (int)(time5*100);

// 動作の指示はここから ////////////////////////////////

// 初期設定は最初に１回
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { 
#ifdef ACTEXEC
	fprintf(stderr, "start ActGrabRelease()\n");
#endif
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
	m[A1J7].ctrlmode = m[A1J8].ctrlmode = MODENONE; // 左前脚
	m[A1J7].duration = m[A1J8].duration = time1;
	m[A4J7].ctrlmode = m[A4J8].ctrlmode = MODENONE; // 右前脚
	m[A4J7].duration = m[A4J8].duration = time1;
        timer = 0;
// 掴むものの座標の設定
	x = d -> fobjxyz[0][0]; // 配列の先頭のものを「仮に」掴む
	y = d -> fobjxyz[0][1];
	z = d -> fobjxyz[0][2];
/*	x = d -> fobjxyz[d -> fobjnum - 1][0]; // 配列の末尾のものを掴む書き方
	y = d -> fobjxyz[d -> fobjnum - 1][1];
	z = d -> fobjxyz[d -> fobjnum - 1][2];
*/
	fprintf(stderr, "ActGrabRelease(), target x:%.2f y:%.2f z:%.2f\n", x, y, z);
/* 古い処理
	x = p[AP_LOCALX]; // 掴む対象となる物体の座標、以前はp[]経由で受け取った
	y = p[AP_LOCALY];
	z = p[AP_LOCALZ];
*/

// もしここでRGB画像を参照したい時の書き方の例
//          RGBCam *c = &(gDepthRGB[0]); // これでよいかは要確認
/*          RGBCam *c = gDepthRGB;
            int i = 0; // 配列中の物体の番号
            fprintf(stderr, "(i=%d ", i);
            fprintf(stderr, "%d ", c -> fareacog[i][X]); // 重心
            fprintf(stderr, "%d ", c -> fareacog[i][Y]); 
*/
    }
// １番目の動き、物体の上にハンドを移動させる
    if (0 == (timer % cycletotal)) {
	float zz = z + 0.25, target1[XYZ] = {x, y, zz}; // アーム、物体の 250mm 上に
	ikparam[0] = time1;
// 次の5行はその後のコメントアウトされている処理と同等、三項演算子を使ってまとめた
	IKMotions(m, 0.0<y?LF:RF, target1, ikparam); // 左右のアームの選択
	Hand(m, 0.0<y?LF:RF, HANDOPEN, 1.57); // 物体に近い方のハンドを開く
	float targetrest[XYZ] = {0.6, 0.2, 0.3}; // 使わないアーム、待機座標
	targetrest[1] *= 0.0<y?-1:1; // 使わないアーム、左右の判定
	IKMotions(m, 0.0<y?RF:LF, targetrest, ikparam); // 使わないアーム、おやすみ
/*
	if (0.0 < y) { // 物体が左右どちらにあるかで動かすアームを決める
	    IKMotions(m, LF, target1, ikparam); // 左アーム、物体の 250mm 上に
	    Hand(m, LF, HANDOPEN, 1.57); // 左ハンドを開く
	    float targetrest[XYZ] = {0.6, -0.2, 0.3};
	    IKMotions(m, RF, targetrest, ikparam); // 右アーム、おやすみ
	}
	else {
	    IKMotions(m, RF, target1, ikparam);	// 右アーム
	    Hand(m, RF, HANDOPEN, 1.57); // 右ハンドを開く
	    float targetrest[XYZ] = {0.6, 0.2, 0.3};
	    IKMotions(m, LF, targetrest, ikparam); // 左アーム、おやすみ
	}
*/
    }
// ２番目の動き、腕を下ろす
    if (cycle1 == (timer % cycletotal)) { // cycle1 回呼び出されたら次の動きに移る
	ikparam[0] = time2; // 動きによって時間が異なる場合
// 次行、xx, yy, zz は高確率で掴むための目標位置の微調整
	float xx = x + 0.075, yy = y * 1.1, zz = z - 0.075, target1[XYZ] = {xx, yy, zz};
// 物体が左右どちらにあるかで動かす腕を変える
	IKMotions(m, 0.0<y?LF:RF, target1, ikparam);
/*
	if (0.0 < y) IKMotions(m, LF, target1, ikparam); // 物体の 50mm 上
	else         IKMotions(m, RF, target1, ikparam);
*/
    }
// ３番目の動き、ハンドを閉じる
    if ((cycle1 + cycle2) == (timer % cycletotal)) { // 次の動きに移る
	ikparam[0] = time3; // 動きによって時間が異なる場合
	Hand(m, 0.0<y?LF:RF, HANDOPEN, 0.4); // ハンドを閉じる
/*
	if (0.0 < y) Hand(m, LF, HANDOPEN, 0.4); // 左ハンドを閉じる
	else         Hand(m, RF, HANDOPEN, 0.4); // 右ハンドを閉じる
	}
*/
    }
// ４番目の動き、持ち上げる
    if ((cycle1 + cycle2 + cycle3) == (timer % cycletotal)) { // 次の動きに移る
	ikparam[0] = time4; // 動きによって時間が異なる場合
	float zz = z + 0.4, target1[XYZ] = {0.5, 0, 0};
	target1[1] = 0.0<y?-0.1:0.1;
	IKMotions(m, 0.0<y?LF:RF, target1, ikparam);
    }
// ５番目の動き、ハンドを開いて落とす
    if ((cycle1 + cycle2 + cycle3 + cycle4) == (timer % cycletotal)) { // 次の動きに移る
	ikparam[0] = time5; // 動きによって時間が異なる場合
	Hand(m, 0.0<y?LF:RF, HANDOPEN, 1.57); // ハンドを開く
	float target1[XYZ] = {0.5, 0, 0.5};
	target1[1] = 0.0<y?0.2:-0.2;
	ikparam[0] = 3.0; // 動きによって時間が異なる場合
	IKMotions(m, 0.0<y?LF:RF, target1, ikparam);
    }
// ここまでの間に書く //////////////////////////////////

    timer++;
    if (timer < cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC
	fprintf(stderr, "end ActGrabRelease()\n");
#endif
	return ACT_END;
    }
}

// BodyShift で動かす脚と動かさない脚の組み合わせをコード化する
int encode_leg(int flag[])
{
    return flag[0]*1 + flag[1]*2 +flag[2]*4 +flag[3]*8 +flag[4]*16 +flag[5]*32;
}

// コード化された値を分離する
void decode_leg(int val, int flag[])
{
    flag[0] = val & 1 ? 1 : 0;
    flag[1] = val & 2 ? 1 : 0;
    flag[2] = val & 4 ? 1 : 0;
    flag[3] = val & 8 ? 1 : 0;
    flag[4] = val & 16 ? 1 : 0;
    flag[5] = val & 32 ? 1 : 0;
}
