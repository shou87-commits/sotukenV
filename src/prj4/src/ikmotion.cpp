// ikmotion.cpp // 逆運動学を計算して動きを生成する

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <unistd.h>

#include "const.h"
#include "motorclass.h"
#include "armclass.h"

extern ARM *gArm;

int  IKIncrementMotions(C1 m[], int arm, float inc[], float ikparam[]);
int  IKMotions(C1 m[], int arm, float target[], float ikparam[]);
int  IKCalc(float target[], float org[], float shoulder[], float ikparam[], float angles[]);

void IKInitialPose(C1 m[]);
void IKSimpleMotion(C1 m[]);
void IKWalkMotion(C1 m[]);
void InitialPoseOld(C1 m[]); // 従来の初期姿勢の決め方

int IKIncrementMotions(C1 m[], int arm, float inc[], float ikparam[])
{
    float target[XYZ];
    
    gArm[arm - 1]. get_xyz(target);
    target[0] += inc[0];
    target[1] += inc[1];
    target[2] += inc[2];
    return IKMotions(m, arm, target, ikparam);
}

int IKMotions(C1 m[], int arm, float target[], float ikparam[])
// arm : 計算対象のアームの番号 1 ~ 6
{
    float org[XYZ] = {0.0, 0.0, 0.0};     // ロボット原点は本体の重心、ローカル座標ではすべて 0
// 以下はロボット原点から見た肩関節の座標
    float shoulder[NUMARMS][XYZ] = {{0.15, 0.15, 0.0}, {0.0, 0.15, 0.0}, {-0.15, 0.15, 0.0},
				    {0.15,-0.15, 0.0}, {0.0,-0.15, 0.0}, {-0.15,-0.15, 0.0}};
    float angles[NUMJOINTS] = {0.0}; // Arm の各 joint の目標角度(rad)
    int joints = NUMJOINTS + NUMHANDJOINTS;
    int rcode = NORMAL;
    
// ikarm[] に目標座標を記録しておく
    gArm[arm - 1]. set_xyz(target);
    
// 逆運動学の計算対象は、現在は肩から最初の3個の関節「のみ」、４個以上は計算が難しいため
    m[(arm-1) * joints + 0].duration = ikparam[0];
    m[(arm-1) * joints + 0].ctrlmode = (int)ikparam[3];
    m[(arm-1) * joints + 1].duration = ikparam[0];
    m[(arm-1) * joints + 1].ctrlmode = (int)ikparam[3];
    m[(arm-1) * joints + 2].duration = ikparam[0];
    m[(arm-1) * joints + 2].ctrlmode = (int)ikparam[3];
    
    rcode = IKCalc(target, org, shoulder[arm-1], ikparam, angles); // Arm 1 本分の逆運動学の計算
    if (ERROR == rcode) fprintf(stderr, "ikerrA%d\n", arm); // 解がない場合
    else {
	m[(arm-1) * joints + 0].targetangle.data = angles[0]; // 解が求められていれば
	m[(arm-1) * joints + 1].targetangle.data = angles[1]; // 計算結果を目標角度に設定する
	m[(arm-1) * joints + 2].targetangle.data = angles[2];
    }
    return rcode;
}

int IKCalc(float target[], float org[], float shoulder[], float ikparam[], float angles[])
{
    static float len[NUMLINKS] = {0.05, 0.3, 0.3, 0.05, 0.05, 0.05}; // 各 link の長さ(m)
    float scale; // 目標座標が遠いときにどの程度近付けるかの係数
    int rcode = NORMAL; // 計算が問題なくできたかを表す返り値、NORMAL or ERROR
    
// 計算対象を手首にするか、ハンド先端にするか
    if (IKWRIST == ikparam[1]) len[2] = 0.3;
    else                       len[2] = 0.3+0.05+0.05+0.05+0.10; // 0.10 はハンドの長さ

// XY 平面の回転、これは簡単に求められる
    angles[0] = atan2(target[1] - (org[1] + shoulder[1]), target[0] - (org[0] + shoulder[0]));

    float org2[XYZ]; // angles[0] rad 回転後の肩のリンクの先端座標、以後の計算の基準点
    org2[0] = org[0] + shoulder[0] + (cos(angles[0]) * len[0]);
    org2[1] = org[1] + shoulder[1] + (sin(angles[0]) * len[0]);
    org2[2] = org[2] + shoulder[2];
    float alpha = sqrt(((target[0] - org2[0]) * (target[0] - org2[0])) +
		       ((target[1] - org2[1]) * (target[1] - org2[1])));
    float beta = target[2] - org2[2];
    float g = (alpha*alpha + beta*beta - len[1]*len[1] - len[2]*len[2]) / (2 * len[1] * len[2]);

// 1.0 < g は目標座標が遠くて届かないので、なんらかの対処が必要
// 以下では届く位置まで目標座標を近づけてみる

    while (1.0 < g) { // 遠くて届かない場合 (1.0 < g) は、目標座標を再帰的に近くに移動させてみる
	rcode = ERROR;
	float oldg = g;
//	fprintf(stderr, "Warning: target scaled %.1f ", g);
	usleep(10000);
	if (5.0 < g) scale = 0.667; // 遠すぎるときは一気に近付ける
	else         scale = 0.95;  // ある程度近いときは少しずつ近付ける
//	fprintf(stderr, "(%.1f, %.1f, %.1f) moved to ", target[0], target[1], target[2]);
	target[0] *= scale; target[1] *= scale; target[2] *= scale;	
//	fprintf(stderr, "(%.1f, %.1f, %.1f)\n", target[0], target[1], target[2]);
	org2[0] = org[0] + shoulder[0] + (cos(angles[0]) * len[0]);
	org2[1] = org[1] + shoulder[1] + (sin(angles[0]) * len[0]);
	alpha = sqrt(((target[0] - org2[0]) * (target[0] - org2[0])) +
                       ((target[1] - org2[1]) * (target[1] - org2[1])));
//	beta = target[2] - org2[2]; 変化なし、再計算不要
	g = (alpha*alpha + beta*beta - len[1]*len[1] - len[2]*len[2]) / (2 * len[1] * len[2]);

// 左前脚に対して (0.7, 0.3, 0.5) は逆運動学が問題なく解け、(0.7, 0.3, 0.8)
// は目標座標の移動で対応でき、(0.7, 0.3, 0.9) は無限ループ、エラーとなる。
// 逆運動学の動作の確認に使える。
/*	
	if (oldg < g) { // g が小さくならない場合は、目標座標を乱数でずらす、ほぼ効果なし
	    fprintf(stderr, "(%.1f, %.1f, %.1f) moved to ", target[0], target[1], target[2]);
	    target[0] += (1==(rand()%2)) ? 0.1 : -0.1;
	    target[1] += (1==(rand()%2)) ? 0.1 : -0.1;
	    target[2] += (1==(rand()%2)) ? 0.1 : -0.1;
	    fprintf(stderr, "(%.1f, %.1f, %.1f)\n", target[0], target[1], target[2]);
	}
*/
// g が小さくならない、または近過ぎる場合は、今は計算を諦めて戻る
	if (oldg < g || (fabs(target[0])+fabs(target[1])+fabs(target[2])) < 0.75) { 
	    rcode = ERROR;
//	    fprintf(stderr, "Give up\n");
// 諦める場合はなるべく他のアームとぶつからない角度にしておく
// その後、呼び出し元では解がない場合は元の角度から変更しない運用に変更した
	    if      ((0.0 < shoulder[0]) && (0.0 < shoulder[1])) angles[0] = -RAD45; // LF
	    else if ((0.0 ==shoulder[0]) && (0.0 < shoulder[1])) angles[0] = 0.0; // LM
	    else if ((shoulder[0] < 0,0) && (0.0 < shoulder[1])) angles[0] = RAD45; // LB
	    else if ((0.0 < shoulder[0]) && (shoulder[1] < 0.0)) angles[0] = RAD45; // RF
	    else if ((0.0 ==shoulder[0]) && (shoulder[1] < 0.0)) angles[0] = 0.0; // RM
	    else if ((shoulder[0] < 0.0) && (shoulder[1] < 0.0)) angles[0] = -RAD45; // RB
	    angles[1] = 0.0;
	    angles[2] = 0.0;
	    return rcode;
	}
    }

// 以降は肘の上下の向きの調整、脚が付いている左右の側面による調整
    
    float gamma = acos(g);
    float eta = atan2(len[2]*sin(gamma), len[1]+len[2]*cos(gamma));

    if (IKELBOWUP == ikparam[2]) { // 肘を上向きにする
	angles[1] = (asin(beta / sqrt(alpha*alpha + beta*beta)) - eta) * -1 + atan2(beta, alpha) * 2;
	angles[2] = gamma * -1;
    }
    else if (IKELBOWDOWN == ikparam[2]) { // 肘を下向きにする
	angles[1] = (asin(beta / sqrt(alpha*alpha + beta*beta)) - eta);
        angles[2] = gamma;
    }

    // 本体の左右の脚で向きや角度の調整が必要、shoulder[1] の正負で左右を判断している
    if (shoulder[1] < 0.0) { // 右側に脚がついていたら
	angles[0] += RAD90;
	angles[1] *= -1;
	angles[2] *= -1;
    }
    else { // 左側に脚がついていたら
	angles[0] -= RAD90;
    }
// ここから下は手首、未対応、当面は各関節角度を個別に指定する
//  angles[3] = 0; 
//  angles[4] = 0;
//  angles[5] = 0;

    return rcode;
}

// ここから下は動きの例 --------------------------------------------------------

void IKInitialPose(C1 m[]) // 初期姿勢の設定
{
// 逆運動学を利用する初期姿勢の設定、各脚の脚先の座標を指定する
// 角度一定の関節はここに初期値を書いて、その後放置で構わない

    float initxyz[NUMARMS][XYZ] = {{ 0.5, 0.5,-0.3}, { 0.0, 0.5,-0.3}, {-0.5, 0.5,-0.3},
				   { 0.5,-0.5,-0.3}, { 0.0,-0.5,-0.3}, {-0.5,-0.5,-0.3}};
    float duration = 1.0;

    float ikparam[IKNUMPARAM]; // IK 計算時のパラメータ設定用
    ikparam[0] = duration;  // 動作の時間、秒
    ikparam[1] = IKHAND;    // IK 計算時の目標、IKWRIST(手首)、IKHAND(ハンド先端)
    ikparam[2] = IKELBOWUP; // 肘を上下どちらに曲げるか、IKELBOWUP, IKELBOWDOWN
    ikparam[3] = (float)MODEPROFILE; // 制御のモード、duration ちょうどで動くように

    for (int i = 0; i < NUMJOINTS; i++) m[i].targetangle.data = 0; // 全関節を一旦 0 [rad] に初期化

    IKMotions(m, LF, initxyz[0], ikparam);
    IKMotions(m, LM, initxyz[1], ikparam);
    IKMotions(m, LB, initxyz[2], ikparam);
    IKMotions(m, RF, initxyz[3], ikparam);
    IKMotions(m, RM, initxyz[4], ikparam);
    IKMotions(m, RB, initxyz[5], ikparam);
}

void    IKSimpleMotion(C1 m[])
{
    static int timer = 0;
    static int timer2= 0;
    float duration = 2.0;
    static int cycle = (int)(duration*100); // 目標角度を変える周期、cycle*0.01 秒
    
    float ikparam[IKNUMPARAM]; // IK 計算時のパラメータ設定用
    ikparam[0] = duration;  // 動作の時間、秒
    ikparam[1] = IKHAND;    // IK 計算時の目標、IKWRIST(手首)、IKHAND(ハンド先端)
    ikparam[2] = IKELBOWUP; // 肘を上下どちらに曲げるか、IKELBOWUP, IKELBOWDOWN
    ikparam[3] = (float)MODEPROFILE; // 制御のモード、duration ちょうどで動くように

    float target[NUMARMS][4][XYZ] = {
	     {{ 0.5, 0.9,-0.2}, { 0.5, 0.5,-0.3}, { 0.5, 0.3,-0.4}, { 0.5, 0.5,-0.3}},
	     {{ 0.0 , 0.9,-0.2}, { 0.0 , 0.5,-0.3}, { 0.0 , 0.3,-0.4}, { 0.0 , 0.5,-0.3}},
	     {{-0.5, 0.9,-0.2}, {-0.5, 0.5,-0.3}, {-0.5, 0.3,-0.4}, {-0.5, 0.5,-0.3}},
	     {{ 0.5,-0.3,-0.4}, { 0.5,-0.5,-0.3}, { 0.5,-0.9,-0.2}, { 0.5,-0.5,-0.3}},
	     {{ 0.0 ,-0.3,-0.4}, { 0.0 ,-0.5,-0.3}, { 0.0 ,-0.9,-0.2}, { 0.0 ,-0.5,-0.3}},
	     {{-0.5,-0.3,-0.4}, {-0.5,-0.5,-0.3}, {-0.5,-0.9,-0.2}, {-0.5,-0.5,-0.3}}
    };

    if (cycle == timer) { // cycle*0.01 秒ごとに目標角度を変更する
        timer = 0;
	if (0 == timer2 % 4) {
	    IKMotions(m, LF, target[0][0], ikparam);
	    IKMotions(m, LM, target[1][0], ikparam);
	    IKMotions(m, LB, target[2][0], ikparam);
	    IKMotions(m, RF, target[3][0], ikparam);
	    IKMotions(m, RM, target[4][0], ikparam);
	    IKMotions(m, RB, target[5][0], ikparam);
	}
	if (1 == timer2 % 4) {
	    IKMotions(m, LF, target[0][1], ikparam);
	    IKMotions(m, LM, target[1][1], ikparam);
	    IKMotions(m, LB, target[2][1], ikparam);
	    IKMotions(m, RF, target[3][1], ikparam);
	    IKMotions(m, RM, target[4][1], ikparam);
	    IKMotions(m, RB, target[5][1], ikparam);
	}
	if (2 == timer2 % 4) {
	    IKMotions(m, LF, target[0][2], ikparam);
	    IKMotions(m, LM, target[1][2], ikparam);
	    IKMotions(m, LB, target[2][2], ikparam);
	    IKMotions(m, RF, target[3][2], ikparam);
	    IKMotions(m, RM, target[4][2], ikparam);
	    IKMotions(m, RB, target[5][2], ikparam);
	}
	if (3 == timer2 % 4) {
	    IKMotions(m, LF, target[0][3], ikparam);
	    IKMotions(m, LM, target[1][3], ikparam);
	    IKMotions(m, LB, target[2][3], ikparam);
	    IKMotions(m, RF, target[3][3], ikparam);
	    IKMotions(m, RM, target[4][3], ikparam);
	    IKMotions(m, RB, target[5][3], ikparam);
	}
	timer2++;
    }
    timer++;
}

void    IKWalkMotion(C1 m[])
{
    static int timer = 0;
    static int timer2= 0;
    float duration = 3.0;
    static int cycle = (int)(duration*100); // 目標角度を変える周期、cycle*0.01 秒
    
    float ikparam[IKNUMPARAM]; // IK 計算時のパラメータ設定用
    ikparam[0] = duration;  // 動作の時間、秒
    ikparam[1] = IKHAND;    // IK 計算時の目標、IKWRIST(手首)、IKHAND(ハンド先端)
    ikparam[2] = IKELBOWUP; // 肘を上下どちらに曲げるか、IKELBOWUP, IKELBOWDOWN
    ikparam[3] = (float)MODEPROFILE; // 制御のモード、duration ちょうどで動くように

    float target[NUMARMS][4][XYZ] = { // 歩行パターン
       {{ 0.7, 0.4,-0.18}, { 0.7, 0.4,-0.38}, { 0.3, 0.4,-0.38}, { 0.3, 0.4,-0.18}},
       {{-0.2, 0.5,-0.38}, {-0.2, 0.5,-0.18}, { 0.2, 0.5,-0.18}, { 0.2, 0.5,-0.38}},
       {{-0.3, 0.4,-0.18}, {-0.3, 0.4,-0.38}, {-0.7, 0.4,-0.38}, {-0.7, 0.4,-0.18}},
       {{ 0.3,-0.4,-0.38}, { 0.3,-0.4,-0.18}, { 0.7,-0.4,-0.18}, { 0.7,-0.4,-0.38}},
       {{ 0.2,-0.5,-0.18}, { 0.2,-0.5,-0.38}, {-0.2,-0.5,-0.38}, {-0.2,-0.5,-0.18}},
       {{-0.7,-0.4,-0.38}, {-0.7,-0.4,-0.18}, {-0.3,-0.4,-0.18}, {-0.3,-0.4,-0.38}}
    }; 
    
    if (cycle == timer) { // cycle*0.01 秒ごとに目標角度を変更する
        timer = 0;
	if (0 == timer2 % 4) {
	    IKMotions(m, LF, target[0][0], ikparam);
	    IKMotions(m, LM, target[1][0], ikparam);
	    IKMotions(m, LB, target[2][0], ikparam);
	    IKMotions(m, RF, target[3][0], ikparam);
	    IKMotions(m, RM, target[4][0], ikparam);
	    IKMotions(m, RB, target[5][0], ikparam);
	}
	if (1 == timer2 % 4) {
	    IKMotions(m, LF, target[0][1], ikparam);
	    IKMotions(m, LM, target[1][1], ikparam);
	    IKMotions(m, LB, target[2][1], ikparam);
	    IKMotions(m, RF, target[3][1], ikparam);
	    IKMotions(m, RM, target[4][1], ikparam);
	    IKMotions(m, RB, target[5][1], ikparam);
	}
	if (2 == timer2 % 4) {
	    IKMotions(m, LF, target[0][2], ikparam);
	    IKMotions(m, LM, target[1][2], ikparam);
	    IKMotions(m, LB, target[2][2], ikparam);
	    IKMotions(m, RF, target[3][2], ikparam);
	    IKMotions(m, RM, target[4][2], ikparam);
	    IKMotions(m, RB, target[5][2], ikparam);
	}
	if (3 == timer2 % 4) {
	    IKMotions(m, LF, target[0][3], ikparam);
	    IKMotions(m, LM, target[1][3], ikparam);
	    IKMotions(m, LB, target[2][3], ikparam);
	    IKMotions(m, RF, target[3][3], ikparam);
	    IKMotions(m, RM, target[4][3], ikparam);
	    IKMotions(m, RB, target[5][3], ikparam);
	}
	timer2++;
    }
    timer++;
}

// 初期姿勢の設定、従来の個々の関節角度を指定する方法
void InitialPoseOld(C1 m[])
{
    for (int i = 0; i < NUMJOINTS; i++) m[i].targetangle.data = 0; // 全関節を一旦 0 [rad] に初期化
    
    m[A1J1].targetangle.data =-RAD90; // 左前脚を前に曲げる
//  m[A1J2].targetangle.data = RAD90; // 肘を 90 度曲げておく、今は動いてから上げることにする
    m[A1J3].targetangle.data =-RAD90; // 肘を 90 度曲げておく

    m[A2J1].targetangle.data = 0;     // 左中脚
    m[A2J3].targetangle.data =-RAD90; // 肘を 90 度曲げておく
    
    m[A3J1].targetangle.data = RAD45; // 左後脚を後ろに 45 度曲げておく
    m[A3J3].targetangle.data =-RAD90; // 肘を 90 度曲げておく

    m[A4J1].targetangle.data = RAD90; // 右前脚を前に 90 度曲げておく 
    m[A4J3].targetangle.data = RAD90; // 肘を 90 度曲げておく

    m[A5J1].targetangle.data = 0;     // 右中脚
    m[A5J3].targetangle.data = RAD90; // 肘を 90 度曲げておく

    m[A6J1].targetangle.data =-RAD45; // 右後脚を後ろに 45 度曲げておく
    m[A6J3].targetangle.data = RAD90; // 肘を 90 度曲げておく
}

/* 左右の関節角度の違い
    m[A3J1].targetangle.data = M_PI / 3.0; // 左後脚を後ろに度曲げておく
    m[A3J2].targetangle.data =-M_PI / 6.0; // 肘を曲げておく
    m[A3J3].targetangle.data =-M_PI / 6.0; // 肘を曲げておく
    m[A3J4].targetangle.data = M_PI / 1.5; // 手首を上に曲げておく

    m[A6J1].targetangle.data =-M_PI / 3.0; // 右後脚を後ろに度曲げておく
    m[A6J2].targetangle.data = M_PI / 6.0; // 肘を曲げておく
    m[A6J3].targetangle.data = M_PI / 6.0; // 肘を曲げておく
    m[A6J4].targetangle.data =-M_PI / 1.5; // 手首を上に曲げておく
*/
