// motion.cpp

#include <iostream>
#include <vector>
//#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "const.h"
#include "motorclass.h"

// 次の関数は目標座標に対して逆運動学を計算して、結果に従い脚を動かす
void IKMotions(C1 m[], int arm, float target[], float ikparam[]);

// IKMotions() の IK は逆運動学、指定した座標に脚先が行くように関節角度を計算する。
    
// ここでの座標系はローカル座標系で、ロボットがどんな姿勢でも、常に、
// ロボット正面前方がX軸正値、左手方向がY軸正値、ボディ上面の法線方向が
// Z軸正値となる。例として、(0.7, 0.3, 0.5)はロボット正面から左斜め上になる。

// 原点はボディの重心で(0, 0, 0)、左前脚の付け根は(0.15, 0.15, 0)になる。
// ロボットの各リンク長や座標は URDF ファイルに記述されている

// 左前脚に対して (0.7, 0.3, 0.5) は逆運動学が問題なく解け、(0.7, 0.3, 0.8)
// は目標座標の移動で対応でき、(0.7, 0.3, 0.9) は無限ループ、エラーとなる。
// 逆運動学の動作の確認に使える。

void HandDeg(C1 m[], int place, int move, float deg);
void Hand(C1 m[], int place, int move, float rad);
void NeckDeg(C1 m[], float hdeg, float vdeg);
void Neck(C1 m[], float hrad, float vrad);
void Dance(C1 m[]);
float R2D(float rad);
float D2R(float deg);

void HandDeg(C1 m[], int place, int move, float deg)
{
    Hand(m, place, move, D2R(deg));
}

void Hand(C1 m[], int place, int move, float rad)
{
    int j1, j2;
    
    switch (place) {
    case LF: j1 = A1J7; j2 = A1J8; break;
    case LM: j1 = A2J7; j2 = A2J8; break;
    case LB: j1 = A3J7; j2 = A3J8; break;
    case RF: j1 = A4J7; j2 = A4J8; break;
    case RM: j1 = A5J7; j2 = A5J8; break;
    case RB: j1 = A6J7; j2 = A6J8; break;
    default: fprintf(stderr, "Error: Hand()\n"); return;
    }
    if (HANDOPEN == move) {
	if (place <= LB) {
	    m[j1].targetangle.data = rad;
	    m[j2].targetangle.data = rad*-1;
	}
	else {
	    m[j1].targetangle.data = rad*-1;
	    m[j2].targetangle.data = rad;
	}
    }
    else if (HANDCLOSE == move) {
	m[j1].targetangle.data = 0.0;
	m[j2].targetangle.data = 0.0;
    }
}

void NeckDeg(C1 m[], float hdeg, float vdeg)
{
    Neck(m, D2R(hdeg), D2R(vdeg));
}

void Neck(C1 m[], float hrad, float vrad)
{
    if (ANGLEKEEP != hrad) m[A7J1].targetangle.data = hrad * -1;
    if (ANGLEKEEP != vrad) m[A7J2].targetangle.data = vrad * -1; 
}

void Dance(C1 m[])
{
    float duration = 0.15; // 秒、この時間内に動き終わる、
    static int cycle = (int)(duration*100); // 目標角度を変える間隔、cycle*0.01 秒毎
    static int timer = cycle; // 最初に呼び出されたときに目標角度をセットするため
    static int timer2= 0;
    
    float ikparam[IKNUMPARAM]; // IK 計算時のパラメータ設定用
    ikparam[0] = duration;  // 動作の時間、秒
    ikparam[1] = IKHAND;    // IK 計算時の先端、IKWRIST(手首)、IKHAND(ハンド先端)
    ikparam[2] = IKELBOWUP; // 肘を上下どちらに曲げるか、IKELBOWUP, IKELBOWDOWN
    ikparam[3] = (float)MODENONE; // 制御のモード、シンプル
//  ikparam[3] = (float)MODEPROFILE; // 制御のモード、duration ちょうどで動くように

#include "danceall.txt"

// 以下の timer を使った if 文の外（前後両方）に関節を動かす指示を置いてはいけない。
// 理由は if 文の外ではこの関数が呼び出される度にその指示が実行されてしまうからである。
// if 文の外では 100 回 / 秒 実行されるのに対し、if 文の中では３秒に１回程度になるので
// if 文の中の指示はほとんど動きに反映されなくなる。
// 上記の説明を理解した上で敢えて指示を置くのは構わない。

// timer, timer2 が途中でリセットされないため、この関数を複数回呼び出すと２回目以降は
// 前回の動きの続きになる
    
    int repeat = 2; // １サイクルの動きを繰り返す回数
    
    if (cycle == timer) {
        timer = 0;
	if (NUMDANCESTATES * repeat * 0 <= timer2 && timer2 < NUMDANCESTATES * repeat * 1) {
	    IKMotions(m, LF, target1[0][timer2 % NUMDANCESTATES], ikparam); 
	    IKMotions(m, LM, target1[1][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, LB, target1[2][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RF, target1[3][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RM, target1[4][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RB, target1[5][timer2 % NUMDANCESTATES], ikparam);
	}
	if (NUMDANCESTATES * repeat * 1 <= timer2 && timer2 < NUMDANCESTATES * repeat * 2) {
	    IKMotions(m, LF, target2[0][timer2 % NUMDANCESTATES], ikparam); 
	    IKMotions(m, LM, target2[1][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, LB, target2[2][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RF, target2[3][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RM, target2[4][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RB, target2[5][timer2 % NUMDANCESTATES], ikparam);
	}
	if (NUMDANCESTATES * repeat * 2 <= timer2 && timer2 < NUMDANCESTATES * repeat * 3) {
	    IKMotions(m, LF, target3[0][timer2 % NUMDANCESTATES], ikparam); 
	    IKMotions(m, LM, target3[1][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, LB, target3[2][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RF, target3[3][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RM, target3[4][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RB, target3[5][timer2 % NUMDANCESTATES], ikparam);
	}
	if (NUMDANCESTATES * repeat * 3 <= timer2 && timer2 < NUMDANCESTATES * repeat * 4) {
	    IKMotions(m, LF, target4[0][timer2 % NUMDANCESTATES], ikparam); 
	    IKMotions(m, LM, target4[1][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, LB, target4[2][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RF, target4[3][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RM, target4[4][timer2 % NUMDANCESTATES], ikparam);
	    IKMotions(m, RB, target4[5][timer2 % NUMDANCESTATES], ikparam);
	}
	if (NUMDANCESTATES * repeat * 4 == timer2) { // 最初の動きに戻す
	    timer2 = 0;
	}
	timer2++;
    }
    timer++;
}

float R2D(float rad)
{
    return(rad / M_PI * 180);
}

float D2R(float deg)
{
    return(deg / 180 * M_PI);
}
