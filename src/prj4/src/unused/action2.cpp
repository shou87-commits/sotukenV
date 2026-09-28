// action2.cpp
// 練習用の動作の関数を user.cpp からこちらに移した。
// プログラムの構造の理解や書き方の練習には役に立つが実用性はあまりない。

#include <math.h>
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

#include "user.h"

// LIDAR 測距センサのデータにアクセスするためのポインタ
// LIDAR を使わなければ消せる
extern LIDAR *gLIDAR;

// 以下の変数の実体は user.cpp にある

extern int act_num;       // 次の行動を決める値、重要
extern int act_last;      // 直前の行動を覚えておく
extern int state;         // 動作の状態を表す、開始、継続中、終了
extern float act_param[]; // パラメータの受け渡し用配列

void MyMotion(C1 m[]) // m[] はモータコントローラの配列、モータを直接動かすために必要
{
//////////////////////////////////////////////////////////
// 
// 各種の行動を実行する順番や繋がりを定義する
// フローチャートのように記述できる
// 
// 
//////////////////////////////////////////////////////////
    
//  act_param[AP_STATE] = ACT_START;           // ここで実行すると毎回初期化になってまずいので不可

// act_num の決め方は（必要なら動作結果を act_param[] で戻して）、次の if 文内で決める
    if (ACT_START == state) {                  // プログラム起動直後、最初はここから
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を覚えておく、後で必要になるかも
	act_num = ACT_INITPOSE;                // 最初は初期姿勢、立ち上がる
    }
    if (ACT_END == state) {                    // 直前の行動が終了したら
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を覚えておく、後で必要になるかも

	if (ACT_INITPOSE == act_last) {
// 直前の行動が ACT_INITPOSE の場合の次の行動を、この if 文内で決める、他の場合も同様の書き方になる
/*	    act_num = ACT_LIDARVSCAN;
	    act_param[AP_LIDAR_VUP]   = 2; // パラメータの設定が必要な場合はこんな感じ
	    act_param[AP_LIDAR_VDOWN] = 2;
*/
	    act_num = ACT_STAY; //ACT_LIDAR;
	}
	if (ACT_LIDAR  == act_last) act_num = ACT_LIDAR2; // 直前が ACT_LIDAR の時の次の行動
	if (ACT_LIDAR2 == act_last) act_num = ACT_LIDAR3; // 次の行動を 1 行で指定することも可能
	if (ACT_LIDAR3 == act_last) act_num = ACT_WALK; // 直前が ??? の時の次の行動を決める
	if (ACT_LIDARVSCAN == act_last) act_num = ACT_STAY; // 直前が ??? の時の次の行動を決める
	if (ACT_1 == act_last) { // 直前が ??? の時の次の行動を決める
//          if (1.0 == act_param[??]) act_num = ACT_2; // act_param[] によって次の動作を決める例
	    act_num = ACT_2;
	}
	if (ACT_2 == act_last) act_num = ACT_1; // 次の行動を決める
	if (ACT_12 == act_last) act_num = ACT_12; // 次の行動を決める
	if (ACT_ERR == act_last) act_num = ACT_RELAX; // エラー時の次の行動を決める
    }
//  if (ACT_MOVING == state) ; // 動作が継続中の時は特別な理由がなければ指示を変更しない
    if (ACT_ERR == state) act_num = ACT_ERR; // 状態がエラーの時はエラー処理へ

//////////////////////////////////////////////////////////
// 
// この後は個々の行動の具体的な内容を定義する
// 必要なパラメータをセットして動きを実現する関数を呼び出す
// 
//////////////////////////////////////////////////////////
    
// 次の 2 行は ACT_LIDARVSCAN 用の繰り返し回数の管理
    static int iter = 0;
    int iterend = act_param[AP_LIDAR_VUP] - act_param[AP_LIDAR_VDOWN] + 1;
    
// ここから先は個々の行動に必要なパラメータをセットして関数を呼び出す
    switch (act_num) {
        case ACT_INITPOSE : // 初期姿勢、立ち上がる
	    act_param[AP_TIMETOTAL] = 6.0; // 動作時間 (sec)
	    state = ActInitialPose(m, act_param);
	    break;
        case ACT_LIDAR : // LIDAR で指定した首振り角度を調べる
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    act_param[AP_NECK_HA] = +0.0;  // 左右首振り目標角度 (degree)
	    act_param[AP_NECK_VA] = +21.0; // 上下首振り目標角度 (degree)
	    state = ActLidar(m, act_param);
	    break;
	case ACT_LIDAR2 :
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    act_param[AP_NECK_HA] = +0.0;  // 左右首振り目標角度 (degree)
	    act_param[AP_NECK_VA] = +12.0; // 上下首振り目標角度 (degree)
	    state = ActLidar(m, act_param);
	    break;
	case ACT_LIDAR3 :
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    act_param[AP_NECK_HA] = +0.0;  // 左右首振り目標角度 (degree)
	    act_param[AP_NECK_VA] = +1.0;  // 上下首振り目標角度 (degree)
	    state = ActLidar(m, act_param);
	    break;
        case ACT_LIDARVSCAN : // LIDAR を縦方向に SCAN、ほぼ完成、結果を要確認
	    act_param[AP_LIDAR_VUP]   = +30.0; // VSCAN 上の角度 (degree)
	    act_param[AP_LIDAR_VDOWN] = -10.0;  // VSCAN 下の角度 (degree)

	    act_param[AP_TIME1] = 1.0;        // 動き１の動作時間 (sec)
	    act_param[AP_PHASE] = 1;
	    act_param[AP_ITER]  = 1;
	    act_param[AP_NECK_HA] = +0.0;  // 左右首振り目標角度 (degree)
	    act_param[AP_NECK_VA] = act_param[AP_LIDAR_VUP] - iter;  // 首振り目標角度 (degree)
	    act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER] - 0.01; // 合計動作時間 (sec)
	    state = ActLidar(m, act_param);

//	    printf("%d ", (int)(act_param[AP_NECK_VA]));
	    
	    if (ACT_END == state) {
		for (int x = 0; x < gLIDAR -> numdata; x++) {
		    gLIDAR -> dist2D[iter][x] = gLIDAR -> distance[x];
		}
		iter++;
		if (iter < iterend) {
		    state = ACT_MOVING;
		    act_param[AP_STATE] = ACT_MOVING;
		}
		else if (iter == iterend) {

		    FILE *fp;
		    if ((FILE *)NULL == (fp = fopen("/tmp/lidar.pgm", "w"))) {
			fprintf(stderr, "Error: [%s] can not open\n", "/tmp/lidar.pgm");
			exit(1);
		    }
		    fprintf(fp, "P5\n%d %d\n255\n", gLIDAR -> numdata, iter);
		    for (int i = 0; i < iter; i++) {
			for (int j = gLIDAR -> numdata - 1; 0 <= j; j--) {
			    unsigned char d;
			    if (2.0 < gLIDAR -> dist2D[i][j]) d = 255;
			    else d = (1.0 - gLIDAR -> dist2D[i][j] / 2.0) * 255;
			    fwrite(&d, sizeof(unsigned char), 1, fp);
			}
		    }
		    fclose(fp);

		    state = ACT_END;
		    act_param[AP_STATE] = ACT_END;
		    iter = 0;
		}
	    }
/*
	    act_param[AP_TIME1] = 0.25;        // 動き１の動作時間 (sec)
	    act_param[AP_PHASE] = act_param[AP_LIDAR_VUP] - act_param[AP_LIDAR_VDOWN] + 1;
	    act_param[AP_ITER]  = 1;           // １組の行動の繰り返し回数
	    act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER] - 0.01; // 合計動作時間 (sec)
	    state = ActLidarVScan(m, act_param);
*/
	    break;
        case ACT_1 : // 動きの例、実用的なものではない
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    state = Act1(m, act_param);
	    break;
	case ACT_2 : // 動きの例、実用的なものではない
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    state = Act2(m, act_param);
	    break;
	case ACT_12 : // 複数の動きの組み合わせの例、実用的なものではない
	    act_param[AP_TIME1] = 1.0;         // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 2.0;         // 行動の動作時間 (sec)
	    act_param[AP_TIME3] = 1.0;         // 行動の動作時間 (sec)
	    act_param[AP_PHASE] = 3;           // １組の行動に含まれる動きの種類
	    act_param[AP_ITER]  = 1;           // １組の行動の繰り返し回数
	    act_param[AP_TIMETOTAL] = act_param[AP_TIME1] + act_param[AP_TIME2] +
		                      act_param[AP_TIME3] - 0.01; // 全体の動作時間 (sec)
	    state = Act12(m, act_param);
	    break;
	case ACT_WALK : // 普通の歩行
	    act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	    act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
// 頻繁に変更される値は別の場所で値を設定してもよい
//	    act_param[AP_ITER]  = 1;           // １組の行動の繰り返し回数、１回で約１ｍ進む
	    act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
	    act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
	    state = ActWalk(m, act_param);
	    break;
        case ACT_FASTWALK : // 高速な歩行
	    act_param[AP_TIME1] = 0.25;        // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 0.25;        // 浮いている脚の最後の着地の動作時間 (sec)
	    act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
//	    act_param[AP_ITER]  = 1;           // １組の行動の繰り返し回数
	    act_param[AP_CTRL]  = MODENONE;    // 制御のアルゴリズム
	    act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
	    state = ActWalk(m, act_param);
	    break;
        case ACT_TURN : // その場で転回、旋回する
	    act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	    act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
//	    act_param[AP_ITER]  = 1;           // １組の行動の繰り返し回数、１回で約３０度回転
	    act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
	    act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
	    state = ActTurn(m, act_param);
	    break;
        case ACT_NECK : // 首を動かす
//	    act_param[AP_TIME1] = 1.0;         // 動き１の動作時間 (sec)
	    act_param[AP_PHASE] = 1;           // １組の行動に含まれる動きの種類
	    act_param[AP_ITER]  = 1;           // １組の行動の繰り返し回数、１回で約３０度回転
	    act_param[AP_CTRL]  = MODENONE;    // 制御のアルゴリズム
	    act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER]; // 合計動作時間 (sec)
	    state = ActNeck(m, act_param);
	    break;
        case ACT_STAY : // 現状維持
	    act_param[AP_TIMETOTAL] = 10.0; // 合計動作時間 (sec)
	    state = ActStay(m, act_param);
	    break;
//      case ACT_ERR : // ACT_ERROR -> ACT_ERR の変更に伴ってこの行は不要になった
        default : // どの case にもマッチしないとき、本来ここにはマッチしないはず
	    fprintf(stderr, "Error: Bad act_num specified, or other error occured.\n");
	    state = ACT_END;
            break;
    }
}
    
int Act1(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; 
    int cycletotal = (int)(duration*100);

// 動作の指示はここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	ikparam[0] = duration;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODENONE;
        timer = 0;
    }
// １番目の動き
    if (0 == (timer % cycletotal)) {
	float target1[XYZ] = { 0.65, 0.65, -0.1};
	float target2[XYZ] = { 0.65,-0.65, 0.5};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
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

int Act2(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	ikparam[0] = duration;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODENONE;
        timer = 0;
    }    
// １番目の動き
    if (0 == (timer % cycletotal)) {
	float target1[XYZ] = { 0.65, 0.65, 0.5};
	float target2[XYZ] = { 0.65,-0.65, -0.1};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
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

int Act12(C1 m[], float p[])
{
    static int timer  = 0; // 関数が呼び出された回数をカウントする
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
    float time1 = p[AP_TIME1]; // 動き１の時間、秒
    float time2 = p[AP_TIME2]; // 動き２の時間、秒
    float time3 = p[AP_TIME3]; // 動き３の時間、秒
    int cycletotal = (int)(duration*100); // 動作終了までに関数が呼び出される回数
    int cycle1 = (int)(time1*100); // 動き１の動作終了までに関数が呼び出される回数
    int cycle2 = (int)(time2*100); // 動き２の動作終了までに関数が呼び出される回数
    int cycle3 = (int)(time3*100); // 動き３の動作終了までに関数が呼び出される回数

// 動作の指示はここから ////////////////////////////////

// 複数の動きを順番に行わせるときの書き方
// 初期設定は最初に１回
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { 
	ikparam[0] = time1;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
    }
// １番目の動き
    if (0 == (timer % cycletotal)) {
	float target1[XYZ] = { 0.65, 0.65, 0.0};
	float target2[XYZ] = { 0.65,-0.65, 0.5};

	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ２番目の動き
    if (cycle1 == (timer % cycletotal)) { // cycle1 回呼び出されたら次の動きに移る
//	float ikparam[IKNUMPARAM]; // 設定値が前と同じなら省略可
	ikparam[0] = time2; // 動きによって時間が異なる場合
//	ikparam[1] = IKHAND;
//	ikparam[2] = IKELBOWUP;
//	ikparam[3] = (float)MODEPROFILE;
	float target1[XYZ] = { 0.65, 0.65, 0.25};
	float target2[XYZ] = { 0.65,-0.65, 0.25};

	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ３番目の動き
    if ((cycle1 + cycle2) == (timer % cycletotal)) { // 次の動きに移る
	ikparam[0] = time3; // 動きによって時間が異なる場合
	float target1[XYZ] = { 0.65, 0.65, 0.5};
	float target2[XYZ] = { 0.65,-0.65, 0.0};

	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
/*  if ((cycle1 + cycle2 +cycle3) == (timer % cycletotal)) {
// この処理は次でされるのでここでは不要
    }
*/
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

int ActEX1(C1 m[], float p[])
{
    static int timer = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	m[A7J1].ctrlmode = MODENONE;
        m[A7J1].duration = duration;
	m[A7J1].targetangle.data = 1.0; // radian
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
	return ACT_END;
    }
}

int ActEX2(C1 m[], float p[])
{
    static int timer = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	m[A7J1].ctrlmode = MODEPROFILE;
        m[A7J1].duration = duration;
	m[A7J1].targetangle.data = 0.0; // radian
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
	return ACT_END;
    }
}
