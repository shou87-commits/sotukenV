// oldmotion.cpp

#include <iostream>
#include <vector>
//#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "const.h"
#include "motorclass.h"

//typedef unsigned char Uchar;

// 次の関数は目標座標に対して逆運動学を計算して、結果に従い脚を動かす
void IKMotions(C1 m[], int arm, float target[], float ikparam[]);

// 初期姿勢の設定、歩行用
void SetInitialPose(C1 m[])
{
    for (int i = 0; i < NUMALLJOINTS; i++) m[i].targetangle.data = 0; // 全関節を一旦 0 [rad] に初期化

    m[A1J1].targetangle.data =-M_PI / 3.0; // 左前脚を前に　度曲げる、角度一定の関節はここに書いて構わない
    m[A1J2].targetangle.data =-M_PI / 6.0; // 肘を曲げておく
    m[A1J3].targetangle.data =-M_PI / 6.0; // 肘を曲げておく
    m[A1J4].targetangle.data = M_PI / 1.5; // 手首を上に曲げておく

    m[A2J1].targetangle.data = 0;          // 左中脚
    m[A2J2].targetangle.data =-M_PI / 6.0; // 肘を曲げておく
    m[A2J3].targetangle.data =-M_PI / 6.0; // 肘を曲げておく
    m[A2J4].targetangle.data = M_PI / 1.5; // 手首を上に曲げておく
    
    m[A3J1].targetangle.data = M_PI / 3.0; // 左後脚を後ろに度曲げておく
    m[A3J2].targetangle.data =-M_PI / 6.0; // 肘を曲げておく
    m[A3J3].targetangle.data =-M_PI / 6.0; // 肘を曲げておく
    m[A3J4].targetangle.data = M_PI / 1.5; // 手首を上に曲げておく

    m[A4J1].targetangle.data = M_PI / 3.0; // 右前脚を前に　度曲げておく、左右で角度の指定の符号が変わる
    m[A4J2].targetangle.data = M_PI / 6.0; // 肘を曲げておく
    m[A4J3].targetangle.data = M_PI / 6.0; // 肘を曲げておく
    m[A4J4].targetangle.data =-M_PI / 1.5; // 手首を上に曲げておく

    m[A5J1].targetangle.data = 0;          // 右中脚
    m[A5J2].targetangle.data = M_PI / 6.0; // 肘を曲げておく
    m[A5J3].targetangle.data = M_PI / 6.0; // 肘を曲げておく
    m[A5J4].targetangle.data =-M_PI / 1.5; // 手首を上に曲げておく

    m[A6J1].targetangle.data =-M_PI / 3.0; // 右後脚を後ろに度曲げておく
    m[A6J2].targetangle.data = M_PI / 6.0; // 肘を曲げておく
    m[A6J3].targetangle.data = M_PI / 6.0; // 肘を曲げておく
    m[A6J4].targetangle.data =-M_PI / 1.5; // 手首を上に曲げておく
}
    
// 初期姿勢の設定、左前足を周期的に動かす場合
void SetInitialPose2(C1 m[])
{
    for (int i = 0; i < NUMALLJOINTS; i++) m[i].targetangle.data = 0; // 全関節を一旦 0 [rad] に初期化

    m[A1J1].targetangle.data =-M_PI / 2.0; // 左前脚を前に　度曲げる、角度一定の関節はここに書いて構わない
    m[A1J2].targetangle.data = M_PI / 2.5; // 肘を　度曲げておく
    m[A1J3].targetangle.data =-M_PI / 1.25; // 肘を　度曲げておく
    m[A1J4].targetangle.data = M_PI / 2.0;
    m[A1J7].targetangle.data = M_PI / 4.0; // 指　を少し開いておく、関節を動かす場合はここに初期値は書けない
    m[A1J8].targetangle.data =-M_PI / 4.0; // 指　を少し開いておく、同上

    m[A2J1].targetangle.data =-M_PI / 4.0; // 左中脚
    m[A2J2].targetangle.data = M_PI / 2.5; // 肘を　度曲げておく
    m[A2J3].targetangle.data =-M_PI / 1.25; // 肘を　度曲げておく
    
    m[A3J1].targetangle.data = M_PI / 4.0; // 左後脚を後ろに度曲げておく
    m[A3J2].targetangle.data = M_PI / 2.5; // 肘を　度曲げておく
    m[A3J3].targetangle.data =-M_PI / 1.25; // 肘を　度曲げておく

    m[A4J1].targetangle.data = M_PI / 2.0; // 右前脚を前に　度曲げておく 
    m[A4J2].targetangle.data =-M_PI / 2.5; // 肘を　度曲げておく
    m[A4J3].targetangle.data = M_PI / 1.25; // 肘を　度曲げておく
    m[A4J4].targetangle.data =-M_PI / 2.0;
    m[A4J7].targetangle.data =-M_PI / 4.0; // 指　を少し開いておく、関節を動かす場合はここに初期値は書けない
    m[A4J8].targetangle.data = M_PI / 4.0; // 指　を少し開いておく、同上

    m[A5J1].targetangle.data = M_PI / 4.0;
    m[A5J2].targetangle.data =-M_PI / 2.5; // 肘を　度曲げておく
    m[A5J3].targetangle.data = M_PI / 1.25; // 肘を　度曲げておく

    m[A6J1].targetangle.data =-M_PI / 4.0; // 後脚を後ろに度曲げておく
    m[A6J2].targetangle.data =-M_PI / 2.5; // 肘を　度曲げておく
    m[A6J3].targetangle.data = M_PI / 1.25; // 肘を　度曲げておく
}

//  SimpleMotion(gMotor);    // 脚の上げ下げ
void    SimpleMotion(C1 motors[])
{
    static int timer = 0;
    static int timer2= 0;
    static int cycle = 100; // 目標角度を変える周期、cycle*0.01 秒

    if (cycle == timer) { // cycle*0.01 秒ごとに目標角度を変更する
        timer = 0;
	if (0 == timer2 % 2) {
	    motors[A1J2].targetangle.data = 0;
	    motors[A2J2].targetangle.data =-M_PI / 6.0;
	    motors[A3J2].targetangle.data = 0;
	    motors[A4J2].targetangle.data = M_PI / 6.0;
	    motors[A5J2].targetangle.data = 0;
	    motors[A6J2].targetangle.data = M_PI / 6.0;
	}
	if (1 == timer2 % 2) {
	    motors[A1J2].targetangle.data =-M_PI / 6.0;
	    motors[A2J2].targetangle.data = 0;
	    motors[A3J2].targetangle.data =-M_PI / 6.0;
	    motors[A4J2].targetangle.data = 0;
	    motors[A5J2].targetangle.data = M_PI / 6.0;
	    motors[A6J2].targetangle.data = 0;
	}
	timer2++;
    }
    timer++;
}

//  WalkMotion(gMotor);      // 歩行
void    WalkMotion(C1 motors[])
{
    static int timer = 0;
    static int timer2= 0;
    static int cycle = 100; // 目標角度を変える周期、cycle*0.01 秒

    if (cycle == timer) { // cycle*0.01 秒ごとに目標角度を変更する
        timer = 0;
	if (0 == timer2 % 4) {
	    motors[A1J2].targetangle.data = 0;
	    motors[A2J2].targetangle.data =-M_PI / 6.0;
	    motors[A3J2].targetangle.data = 0;
	    motors[A4J2].targetangle.data = M_PI / 6.0;
	    motors[A5J2].targetangle.data = 0;
	    motors[A6J2].targetangle.data = M_PI / 6.0;
	}
	if (1 == timer2 % 4) {
	    motors[A1J1].targetangle.data += M_PI / 8.0;
	    motors[A2J1].targetangle.data +=-M_PI / 8.0;
	    motors[A3J1].targetangle.data += M_PI / 8.0;
	    motors[A4J1].targetangle.data += M_PI / 8.0;
	    motors[A5J1].targetangle.data +=-M_PI / 8.0;
	    motors[A6J1].targetangle.data += M_PI / 8.0;
	}
	if (2 == timer2 % 4) {
	    motors[A1J2].targetangle.data =-M_PI / 6.0;
	    motors[A2J2].targetangle.data = 0;
	    motors[A3J2].targetangle.data =-M_PI / 6.0;
	    motors[A4J2].targetangle.data = 0;
	    motors[A5J2].targetangle.data = M_PI / 6.0;
	    motors[A6J2].targetangle.data = 0;
	}
	if (3 == timer2 % 4) {
	    motors[A1J1].targetangle.data +=-M_PI / 8.0; 
	    motors[A2J1].targetangle.data += M_PI / 8.0;
	    motors[A3J1].targetangle.data +=-M_PI / 8.0;
	    motors[A4J1].targetangle.data +=-M_PI / 8.0;
	    motors[A5J1].targetangle.data += M_PI / 8.0;
	    motors[A6J1].targetangle.data +=-M_PI / 8.0;
	}
	timer2++;
    }
    timer++;
}

//  SampleArmMotion(gMotor); // アームを動かす
void    SampleArmMotion(C1 motors[])
// 周期的に腕を動かす処理の例
// 今は周期的に各関節の目標角度を変えているだけ、それ以外はしていない。
// カメラや距離センサの情報をもとにして、次の動きを決めて関節角度を指定するのが本来の処理
{   
    static int timer = 0;
    static int timer2= 0;
    static int cycle = 400; // 目標角度を変える周期、cycle*0.01 秒
    static int mode = 2;

    if (cycle == timer) { // cycle*0.01 秒ごとに目標角度を変更する
        timer = 0;
	motors[A1J2].targetangle.data = 0;
        if (0.0 == motors[A1J3].targetangle.data) motors[A1J3].targetangle.data = M_PI / 2.0; // 肘
	else                                      motors[A1J3].targetangle.data = 0.0;
        if (0.0 == motors[A1J4].targetangle.data) motors[A1J4].targetangle.data = M_PI / 2.0; // 手首
	else                                      motors[A1J4].targetangle.data = 0.0;
        if (0.0 == motors[A1J5].targetangle.data) motors[A1J5].targetangle.data = M_PI / 2.0; // 手首
	else                                      motors[A1J5].targetangle.data = 0.0;
        if (0.0 == motors[A1J6].targetangle.data) motors[A1J6].targetangle.data = M_PI / 2.0; // 手首
	else                                      motors[A1J6].targetangle.data = 0.0;

	if (0.0 == motors[A1J7].targetangle.data) motors[A1J7].targetangle.data = M_PI / 3.0; // 指
	else                                      motors[A1J7].targetangle.data = 0.0;
	if (0.0 == motors[A1J8].targetangle.data) motors[A1J8].targetangle.data =-M_PI / 3.0; // 指
	else                                      motors[A1J8].targetangle.data = 0.0;

	if (0.0 == motors[A2J7].targetangle.data) motors[A2J7].targetangle.data = M_PI / 4.0; // 指
	else                                      motors[A2J7].targetangle.data = 0.0;
	if (0.0 == motors[A2J8].targetangle.data) motors[A2J8].targetangle.data =-M_PI / 4.0; // 指
	else                                      motors[A2J8].targetangle.data = 0.0;

	if (0.0 == motors[A4J7].targetangle.data) motors[A4J7].targetangle.data =-M_PI / 4.0; // 指
	else                                      motors[A4J7].targetangle.data = 0.0;
	if (0.0 == motors[A4J8].targetangle.data) motors[A4J8].targetangle.data = M_PI / 4.0; // 指
	else                                      motors[A4J8].targetangle.data = 0.0;

        if (M_PI / 1.5 == motors[A7J1].targetangle.data) motors[A7J1].targetangle.data =-M_PI / 1.5; // 首　横
	else                                             motors[A7J1].targetangle.data = M_PI / 1.5;
        if (M_PI / 20.0 == motors[A7J2].targetangle.data)motors[A7J2].targetangle.data =-M_PI / 20.0; // 首　縦
	else                                             motors[A7J2].targetangle.data = M_PI / 20.0;
    }
    timer++;

    if (cycle * 2 == timer2) { // cucle*0.01 *2 秒ごとに制御のモードを切り替え
        timer2 = 0;
	printf("ctrlmode %d\n", mode % 3);
	motors[A1J3].ctrlmode = mode % 3;
	motors[A1J3].duration = 4; // これはなくてもよいのでは？
	motors[A1J4].ctrlmode = mode % 3;
	motors[A1J4].duration = 4;
	motors[A1J5].ctrlmode = mode % 3;
	motors[A1J5].duration = 4;
	motors[A1J6].ctrlmode = mode % 3;
	motors[A1J6].duration = 4;
	motors[A1J7].ctrlmode = mode % 3;
	motors[A1J7].duration = 4;
	motors[A1J8].ctrlmode = mode % 3;
	motors[A1J8].duration = 4;
	mode++;
    }
    timer2++;
}

void Dance(C1 m[])
{
    static int timer = 0;
    static int timer2= 0;
    float duration = 0.15; // 秒、この時間内に動き終わる、
    static int cycle = (int)(duration*100); // 目標角度を変える周期、cycle*0.01 秒
    
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
    
    if (cycle == timer) {
        timer = 0;
	if (360/3*0 < timer2 && timer2 <= 360/3*1) {
	    IKMotions(m, LF, target1[0][timer2 % NUMSTATES], ikparam); 
	    IKMotions(m, LM, target1[1][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, LB, target1[2][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RF, target1[3][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RM, target1[4][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RB, target1[5][timer2 % NUMSTATES], ikparam);
	}
	if (360/3*1 < timer2 && timer2 <= 360/3*2) {
	    IKMotions(m, LF, target2[0][timer2 % NUMSTATES], ikparam); 
	    IKMotions(m, LM, target2[1][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, LB, target2[2][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RF, target2[3][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RM, target2[4][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RB, target2[5][timer2 % NUMSTATES], ikparam);
	}
	if (360/3*2 < timer2 && timer2 <= 360/3*3) {
	    IKMotions(m, LF, target3[0][timer2 % NUMSTATES], ikparam); 
	    IKMotions(m, LM, target3[1][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, LB, target3[2][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RF, target3[3][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RM, target3[4][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RB, target3[5][timer2 % NUMSTATES], ikparam);
	}
	if (360/3*3 < timer2) {
	    IKMotions(m, LF, target4[0][timer2 % NUMSTATES], ikparam); 
	    IKMotions(m, LM, target4[1][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, LB, target4[2][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RF, target4[3][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RM, target4[4][timer2 % NUMSTATES], ikparam);
	    IKMotions(m, RB, target4[5][timer2 % NUMSTATES], ikparam);
	}
	timer2++;
    }
    timer++;
}

// 初期姿勢の設定
void UserInitialPose(C1 m[])
{
// 逆運動学を利用する初期姿勢の設定、各脚の脚先の座標を指定する
// 角度一定の関節はここに初期値を書いて、その後放置で構わない

    float initxyz[NUMARMS][XYZ] = {{ 0.5, 0.5,-0.3}, { 0.2, 0.5,-0.3}, {-0.5, 0.5,-0.3},
				   { 0.5,-0.5,-0.3}, { 0.2,-0.5,-0.3}, {-0.5,-0.5,-0.3}};
    float duration = 1.0; // 秒、この時間内に動き終わる、cycle/100

    float ikparam[IKNUMPARAM]; // IK 計算時のパラメータ設定用、IKMotions()仕用時は必須
    ikparam[0] = duration; // 動作の時間、秒
    ikparam[1] = IKHAND;  // IK 計算時の先端、IKWRIST(手首)、IKHAND(ハンド先端)
    ikparam[2] = IKELBOWUP; // 肘を上下どちらに曲げるか、IKELBOWUP, IKELBOWDOWN
    ikparam[3] = (float)MODEPROFILE; // 制御のモード、duration ちょうどで動くように
    
    for (int i = 0; i < NUMJOINTS; i++) m[i].targetangle.data = 0; // 全関節を一旦 0 [rad] に初期化

    IKMotions(m, LF, initxyz[0], ikparam); // LF は Left Front, 左前脚を表す。const.h を見る。
    IKMotions(m, LM, initxyz[1], ikparam);
    IKMotions(m, LB, initxyz[2], ikparam);
    IKMotions(m, RF, initxyz[3], ikparam);
    IKMotions(m, RM, initxyz[4], ikparam);
    IKMotions(m, RB, initxyz[5], ikparam);
}

// 基本的な使い方、プログラムの書き方を示すための例、この関数の内容はすべて理解する必要あり
void    UserMotion1(C1 m[])
{
    static int timer = 0;
    static int timer2= 0;
    float duration = 3.0; // 秒、この時間内に動き終わる、cycle/100
    static int cycle = (int)(duration*100) + 300; // 目標角度を変える周期、cycle*0.01 秒
                                                  // +300 は静止した姿勢を見やすくするため
    
// 動きを早くしたいときは 0.5 のように duration を小さな値にする        

    float ikparam[IKNUMPARAM]; // IK 計算時のパラメータ設定用
    ikparam[0] = duration;  // 動作の時間、秒
    ikparam[1] = IKHAND;    // IK 計算時の先端、IKWRIST(手首)、IKHAND(ハンド先端)
    ikparam[2] = IKELBOWUP; // 肘を上下どちらに曲げるか、IKELBOWUP, IKELBOWDOWN
    ikparam[3] = (float)MODEPROFILE; // 制御のモード、duration ちょうどで動くように
    
    float target1[XYZ] = {0.7, 0.3,  0.9}; // 脚先の目標座標１, XYZ は３次元の「３」
    float target2[XYZ] = {0.3, 0.7, -0.1}; // 脚先の目標座標２

// IKMotions() の IK は逆運動学、指定した座標に脚先が行くように関節角度を計算する。
    
// ここでの座標系はローカル座標系で、ロボットがどんな姿勢でも、常に、
// ロボット正面前方がX軸正値、左手方向がY軸正値、ボディ上面の法線方向が
// Z軸正値となる。例として、(0.7, 0.3, 0.5)はロボット正面から左斜め上になる。

// 原点はボディの重心で(0, 0, 0)、左前脚の付け根は(0.15, 0.15, 0)になる。
// ロボットの各リンク長や座標は URDF ファイルに記述されている

// 以下の timer を使った if 文の外（前後両方）に関節を動かす指示を置いてはいけない。
// 理由は if 文の外ではこの関数が呼び出される度にその指示が実行されてしまうからである。
// if 文の外では 100 回 / 秒 実行されるのに対し、if 文の中では３秒に１回程度になるので
// if 文の中の指示はほとんど動きに反映されなくなる。
// 上記の説明を理解した上で敢えて背くのは構わない。

// 左前脚に対して (0.7, 0.3, 0.5) は逆運動学が問題なく解け、(0.7, 0.3, 0.8)
// は目標座標の移動で対応でき、(0.7, 0.3, 0.9) は無限ループ、エラーとなる。
// 逆運動学の動作の確認に使える。

    if (cycle == timer) { // cycle*0.01 秒ごとに目標角度を変更する
        timer = 0;
	if (0 == timer2 % 4) {
	    ikparam[1] = IKHAND; // 計算時の脚先端としてハンドを指定
	    ikparam[2] = IKELBOWUP; // 肘を上に曲げる
	    IKMotions(m, LF, target1, ikparam); // LF は Left Front, const.h を見る
	    Hand(m, LF, HANDOPEN, RAD30); // RAD30 は30度を表す radian 値
//	    Hand(m, LM, HANDOPEN, RAD45); // RAD45 は45度, const.h を見る
//	    Hand(m, LB, HANDOPEN, RAD60);
        }
	if (1 == timer2 % 4) {
	    ikparam[1] = IKHAND; // ハンドを指定
	    ikparam[2] = IKELBOWUP; // 肘を上に曲げる
	    IKMotions(m, LF, target2, ikparam);
	    Hand(m, LF, HANDCLOSE, 0); // ハンドを閉じるときは常に 0 rad で
	}
	if (2 == timer2 % 4) { // IKWRIST, IKHAND の違いを示す例
	    float target3[XYZ] = {0.6, 0.15, 0.0}; // ２つの座標は横から見ると同じ位置になる
	    float target4[XYZ] = {0.6,-0.15, 0.0};
	    ikparam[1] = IKHAND; // ハンドを指定
	    ikparam[2] = IKELBOWUP; // 肘を上に曲げる
	    IKMotions(m, LF, target3, ikparam);
	    ikparam[1] = IKWRIST; // 手首を指定
	    ikparam[2] = IKELBOWUP; // 肘を上に曲げる
	    IKMotions(m, RF, target4, ikparam);
	    m[A4J4].targetangle.data =-RAD90; // ※ 従来の個別の関節角度指定も IK と併用可能
	}
	if (3 == timer2 % 4) { // IKELBOWUP, IKELBOWDOWN の違い、肘の曲がり方が逆になる
	    float target5[XYZ] = {0.65, 0.25, 0.3}; // ２つの座標は横から見ると同じ位置になる
	    float target6[XYZ] = {0.65,-0.25, 0.3};
	    m[A4J4].targetangle.data = 0; // 手首を真っ直ぐに戻す
	    ikparam[1] = IKHAND;
	    ikparam[2] = IKELBOWUP; // 肘を上に曲げる
	    IKMotions(m, LF, target5, ikparam);
	    ikparam[1] = IKHAND;
	    ikparam[2] = IKELBOWDOWN; // 肘を下に曲げる、地面との衝突に注意
	    IKMotions(m, RF, target6, ikparam);
	}
	timer2++;
    }
    timer++;
}

// 6本の脚を連動させる動き
void    UserMotion2(C1 m[])
{
    static int timer = 0;
    static int timer2= 0;
    float duration = 2.0; // 秒、この時間内に動き終わる、cycle/100  
    static int cycle = (int)(duration*100); // 目標角度を変える周期、cycle*0.01 秒
    
    float ikparam[IKNUMPARAM]; // IK 計算時のパラメータ設定用
    ikparam[0] = duration;  // 動作の時間、秒
    ikparam[1] = IKHAND;    // IK 計算時の先端、IKWRIST(手首)、IKHAND(ハンド先端)
    ikparam[2] = IKELBOWUP; // 肘を上下どちらに曲げるか、IKELBOWUP, IKELBOWDOWN
    ikparam[3] = (float)MODEPROFILE; // 制御のモード、duration ちょうどで動くように
    
    float target[NUMARMS][4][XYZ] = { // NUMARMS は「6」本脚、「４」は状脚先の座標が４種類
	     {{ 0.5, 0.9,-0.2}, { 0.5, 0.5,-0.3}, { 0.5, 0.3,-0.4}, { 0.5, 0.5,-0.3}},
	     {{ 0.0 , 0.9,-0.2}, { 0.0 , 0.5,-0.3}, { 0.0 , 0.3,-0.4}, { 0.0 , 0.5,-0.3}},
	     {{-0.5, 0.9,-0.2}, {-0.5, 0.5,-0.3}, {-0.5, 0.3,-0.4}, {-0.5, 0.5,-0.3}},
	     {{ 0.5,-0.3,-0.4}, { 0.5,-0.5,-0.3}, { 0.5,-0.9,-0.2}, { 0.5,-0.5,-0.3}},
	     {{ 0.0 ,-0.3,-0.4}, { 0.0 ,-0.5,-0.3}, { 0.0 ,-0.9,-0.2}, { 0.0 ,-0.5,-0.3}},
	     {{-0.5,-0.3,-0.4}, {-0.5,-0.5,-0.3}, {-0.5,-0.9,-0.2}, {-0.5,-0.5,-0.3}}
    };

// 目標座標をプログラム実行中に自動で生成できるようになれば、事前に人が作成する必要がなくなる
    
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

// 歩行の例、右脚の歩幅を小さくして円を描くように歩く
void    UserWalkMotion(C1 m[])
{
    static int timer = 0;
    static int timer2= 0;
    float duration = 0.5; // 秒、この時間内に動き終わる、
    static int cycle = (int)(duration*100); // 目標角度を変える周期、cycle*0.01 秒
    
    float ikparam[IKNUMPARAM]; // IK 計算時のパラメータ設定用
    ikparam[0] = duration;  // 動作の時間、秒
    ikparam[1] = IKHAND;    // IK 計算時の先端、IKWRIST(手首)、IKHAND(ハンド先端)
    ikparam[2] = IKELBOWUP; // 肘を上下どちらに曲げるか、IKELBOWUP, IKELBOWDOWN
    ikparam[3] = (float)MODEPROFILE; // 制御のモード、duration ちょうどで動くように
    
    float target[NUMARMS][4][XYZ] = { // 歩行パターン、６本の脚は４種類の座標を順に動く
       {{ 0.7, 0.4,-0.18}, { 0.7, 0.4,-0.38}, { 0.3, 0.4,-0.38}, { 0.3, 0.4,-0.18}},
       {{-0.2, 0.5,-0.38}, {-0.2, 0.5,-0.18}, { 0.2, 0.5,-0.18}, { 0.2, 0.5,-0.38}},
       {{-0.3, 0.4,-0.18}, {-0.3, 0.4,-0.38}, {-0.7, 0.4,-0.38}, {-0.7, 0.4,-0.18}},
       {{ 0.4,-0.4,-0.38}, { 0.4,-0.4,-0.18}, { 0.5,-0.4,-0.18}, { 0.5,-0.4,-0.38}},
       {{ 0.1,-0.5,-0.18}, { 0.1,-0.5,-0.38}, {-0.1,-0.5,-0.38}, {-0.1,-0.5,-0.18}},
       {{-0.4,-0.4,-0.38}, {-0.4,-0.4,-0.18}, {-0.3,-0.4,-0.18}, {-0.3,-0.4,-0.38}}
    }; 

// 目標座標をプログラム実行中に自動で生成できるようになれば、事前に人が作成する必要がなくなる
    
/*
    float target[NUMARMS][4][XYZ] = { // 直線歩行パターンの例
       {{ 0.7, 0.4,-0.18}, { 0.7, 0.4,-0.38}, { 0.3, 0.4,-0.38}, { 0.3, 0.4,-0.18}},
       {{-0.2, 0.5,-0.38}, {-0.2, 0.5,-0.18}, { 0.2, 0.5,-0.18}, { 0.2, 0.5,-0.38}},
       {{-0.3, 0.4,-0.18}, {-0.3, 0.4,-0.38}, {-0.7, 0.4,-0.38}, {-0.7, 0.4,-0.18}},
       {{ 0.3,-0.4,-0.38}, { 0.3,-0.4,-0.18}, { 0.7,-0.4,-0.18}, { 0.7,-0.4,-0.38}},
       {{ 0.2,-0.5,-0.18}, { 0.2,-0.5,-0.38}, {-0.2,-0.5,-0.38}, {-0.2,-0.5,-0.18}},
       {{-0.7,-0.4,-0.38}, {-0.7,-0.4,-0.18}, {-0.3,-0.4,-0.18}, {-0.3,-0.4,-0.38}}
    }; 
*/
    if (cycle == timer) { // cycle*0.01 秒ごとに目標角度を変更する
        timer = 0;
// 以下は UserMotion2() と同じ意味、これで４種類の動きを順番に実行できる
	IKMotions(m, LF, target[0][timer2 % 4], ikparam); 
	IKMotions(m, LM, target[1][timer2 % 4], ikparam);
	IKMotions(m, LB, target[2][timer2 % 4], ikparam);
	IKMotions(m, RF, target[3][timer2 % 4], ikparam);
	IKMotions(m, RM, target[4][timer2 % 4], ikparam);
	IKMotions(m, RB, target[5][timer2 % 4], ikparam);

	timer2++;
    }
    timer++;
}
