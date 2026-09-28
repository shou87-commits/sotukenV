// motorclass.cpp

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "const.h"
#include "motorclass.h"
#include "ftclass.h"

const float JC_MARGIN = 6.0; // Joint Collision Margin(deg), 隣の脚に衝突するまでのマージン
const float PU_THA    = 0.5; // Profile Update Threshold Angle(deg), profile を更新するかの閾値

extern C1 *gMotor;
extern FTSENSOR *gFT;

float  R2D(float rad);
float  D2R(float deg);
template <typename TYPE>
void setparam(TYPE dst[], TYPE src[], int n);

// 各関節のコントローラのインスタンス生成と初期化
C1::C1(char *name, int idnum, float param[NUMPARAM])
{
    c_pub = nh.advertise<std_msgs::Float64>(name, 1000); // 角度をモータに publish するため

    // timerCallback は周期的に呼び出され、モータの目標角度を決める
    timer = nh.createTimer(ros::Duration(MotorCtrlITVL), &C1::timerCallback, this);

    id = idnum; // 渡された id 値をオブジェクトの変数 id にセットする
    ctrlmode   = param[0]; // 制御の種類を決める
    duration   = param[1]; // この時間をかけてモータは回転する sec
    lasttarget = param[2]; // 目標角度の記憶用変数、初期値は意味のない仮の値

// FTセンサを使わない設定
//  ftstopflag = OFF;

// FTセンサで停止させるための設定    
    ftstopflag = OFF;
    int ary[NUMMOTORFT]; // 設定準備用の配列
    for (int i = 0; i < NUMMOTORFT; i++) ary[i] = EOLV; // 最初にすべてEndOfListVal で初期化
// 必要な関節のみ停止判定用 FT センサの番号を設定する
// 左右前脚のハンドは内側２個のFTセンサのどちらかが反応したら止める
    if      (A1J7 == id) {ary[0] = FTA1J1; ary[1] = FTA1J2;}
    else if (A1J8 == id) {ary[0] = FTA1J1; ary[1] = FTA1J2;}
    else if (A4J7 == id) {ary[0] = FTA4J1; ary[1] = FTA4J2;}
    else if (A4J8 == id) {ary[0] = FTA4J1; ary[1] = FTA4J2;}
// 以下は設定の例、背中に荷重があれば右後脚の３関節の動きを止めるデモ
//  else if (A6J1 == id) {ary[0] = FT5;}
//  else if (A6J2 == id) {ary[0] = FT5;}
//  else if (A6J3 == id) {ary[0] = FT5;}

// 最後にセンサ番号を登録する
    setparam(ft_idx, ary, NUMMOTORFT);
    
    count = 0;        // プロファイルをどこまで実行したか覚えておく
    for (int i = 0; i < PROFSIZE; i++) profile[i] = 0;
}

void C1::Stop()
{
    targetangle.data = anglenow.data;
}

// 動作中はこの関数が周期的に呼び出され、関節の目標角度を送信する
void C1::timerCallback(const ros::TimerEvent&)
{
//  if (0 == id) printf("m"); // 呼び出された回数のカウント、m, j, i, c, d がある

    int i = 0;
    if (ON == ftstopflag) {
        while (EOLV != ft_idx[i]) {
            if (ft_thrd < gFT[ft_idx[i]]. Ftlastave()) Stop(); // ftセンサが一個でも反応したら停止
            i++;
        }
    }

    if (fabs(targetangle.data - anglenow.data) < 0.1 && MOVING == state) {
        state = DONE; // 目標角度に到達したら、完了のフラグをセットする
//      fprintf(stderr, "!%d(%d) ", id, count);
    }
    
    std_msgs::Float64 val;     // モータに送る角度を入れる
    
    if (MODENONE == ctrlmode) { // 単純な PID 制御
        if (D2R(PU_THA) < fabs(targetangle.data - lasttarget)) {  // 目標角度が変更されたか？
	    CheckCollision(id);
	    state = MOVING; // 目標角度が変更されたら、動作中のフラグをセットする
	    lasttarget = targetangle.data;
	    count = 0;
	}
        val.data = targetangle.data;
	count++;
    }

    if (MODEPROFILE == ctrlmode) { // プロファイルによって目標を調整する
        if (D2R(PU_THA) < fabs(targetangle.data - lasttarget)) { // 目標角度変更→ 新プロファイルを計算
	    CheckCollision(id);
	    state = MOVING; // 目標角度が変更されたら、動作中のフラグをセットする
	    lasttarget = targetangle.data;
	    width = targetangle.data - anglenow.data;
	    goal = duration / MotorCtrlITVL;
	    count = 0;

	    int i; // ここからプロファイルの生成
	    float start = anglenow.data; // 開始角度
	    for (i = 0; i < goal * AttackTime; i++) { // 立ち上がりのプロファイル
	        profile[i] = start + (width * Attack * (i+1) / (goal * AttackTime));
	    }
	    start = profile[i-1];
	    for (i = goal * AttackTime; i < goal * (AttackTime+SustainTime); i++) { // 中間部のプロファイル
	        profile[i] = start + (width * Sustain * (i-(goal * AttackTime)+1) / (goal * SustainTime));
	    }
	    start = profile[i-1];
	    for (i = goal * (AttackTime+SustainTime); i < goal; i++) { // 立ち下がりのプロファイル
	        profile[i] = start + (width * Release * (i-(goal * (AttackTime+SustainTime))+1) / (goal * ReleaseTime));
	    }
	    for (i = goal; i < PROFSIZE; i++) profile[i] = targetangle.data;
	}
	if (PROFSIZE - 1 < count) count = PROFSIZE - 1; // 長時間同じ姿勢が続く場合は配列の末尾を超えないように

	val.data = profile[count]; // 計算したプロファイルに沿った値を順番に送る
	count++;
    }

    if (MODEPROFILE2 == ctrlmode) { // プロファイル２によって目標を調整する
        if (D2R(PU_THA) < fabs(targetangle.data - lasttarget)) { // 目標角度変更→ 新プロファイルを計算
	    CheckCollision(id);
	    state = MOVING; // 目標角度が変更されたら、動作中のフラグをセットする
	    lasttarget = targetangle.data;
	    width = targetangle.data - anglenow.data;
	    goal = duration / MotorCtrlITVL / 2;
	    count = 0;

	    int i; // ここからプロファイルの生成
	    for (i = 0; i < goal; i++) { // 立ち上がりのプロファイル
	      profile[i] = anglenow.data + width * (i+1) / goal;
	    }
	    for (i = goal; i < PROFSIZE; i++) profile[i] = targetangle.data;
	}
	if (PROFSIZE - 1 < count) count = PROFSIZE - 1; // 長時間同じ姿勢が続く場合は配列の末尾を超えないように

	val.data = profile[count]; // 計算したプロファイルに沿った値を順番に送る
	count++;
    }

    c_pub.publish(val); // ここでモータに送られる
}

void C1::CheckCollision(int id)
{
    int rcode;

    if (A1J1 == id) {
//	fprintf(stderr, "CheckCollision() %d:%.3f %d:%.3f\n", A1J1,
//		R2D(targetangle.data),A2J1, R2D(gMotor[A2J1].targetangle.data));
	rcode = CheckCollisionCCW(A1J1, A2J1);
    }
    if (A2J1 == id) {
	rcode =	CheckCollisionCCW(A2J1, A3J1);
	rcode =	CheckCollisionCW(A2J1, A1J1);
    }
    if (A3J1 == id) {
	rcode =	CheckCollisionCW(A3J1, A2J1);
    }
    if (A4J1 == id) {
	rcode =	CheckCollisionCW(A4J1, A5J1);
    }
    if (A5J1 == id) {
	rcode =	CheckCollisionCW(A5J1, A6J1);
	rcode =	CheckCollisionCCW(A5J1, A4J1);
    }
    if (A6J1 == id) {
	rcode =	CheckCollisionCCW(A6J1, A5J1);
    }
}

int C1::CheckCollisionCCW(int from, int to)
{
    if (gMotor[to].targetangle.data + D2R(JC_MARGIN) < gMotor[from].targetangle.data) {
	fprintf(stderr, "Warning: Collision CCW J%d to J%d : %.3f -> %.3f\n", from, to,
		R2D(gMotor[from].targetangle.data),
		R2D(gMotor[to].targetangle.data + D2R(JC_MARGIN)));
	gMotor[from].targetangle.data = gMotor[to].targetangle.data + D2R(JC_MARGIN);
	return NO;
    }
    return OK;
}

int C1::CheckCollisionCW(int from, int to)
{
    if (gMotor[to].targetangle.data - D2R(JC_MARGIN) > gMotor[from].targetangle.data) {
	fprintf(stderr, "Warning: Collision CW J%d to J%d : %.3f -> %.3f\n", from, to,
		R2D(gMotor[from].targetangle.data),
		R2D(gMotor[to].targetangle.data - D2R(JC_MARGIN)));
	gMotor[from].targetangle.data = gMotor[to].targetangle.data - D2R(JC_MARGIN);
	return NO;
    }
    return OK;
}

// もし異なる動作のコントローラを使いたい場合はこのように別のオブジェクトにする
C2::C2(char *name, int idnum, float param[NUMPARAM])
{
    c_pub = nh.advertise<std_msgs::Float64>(name, 1000);

    timer = nh.createTimer(ros::Duration(0.01), &C2::timerCallback, this);

    id = idnum; // 渡された値を変数id にセットする
}

// 別のコントローラ
void C2::timerCallback(const ros::TimerEvent&)
{
    std_msgs::Float64 pos;
    
    pos.data = M_PI / 2.0;

    c_pub.publish(pos);
}
