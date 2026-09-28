// suzuki.cpp
// これは個別ユーザ(仮想鈴木君)が独自の動きを記述する場合の例
// このファイルを自分用にコピーして内容を書き換えればそのまま使える
// user.cpp 、その他のファイルを書き換える手間を非常に少なくできる

// 自分が作ったファイルや関数を配布されたプロジェクトに統合する手順
/*
(1)
自分で作成したファイル suzuki.cpp の名前を prj4/CMakeLists.txt の最後
の方にある次の行に以下のように追加する。１行で書くこと。
add_executable(prg4 src/prg4.cpp src/motion.cpp src/motorclass.cpp src/ikmotion.cpp 
               src/user.cpp src/action.cpp src/action2.cpp src/suzuki.cpp)

(2)
(2-1), (2-2) のどちらかを実行する。(2-1) の方が楽なのでおすすめ、(2-2)
は参考程度の情報として残してある。

(2-1)
user.cpp.suzuki を参考にして自分で作成した関数
SuzukiMotion() を呼び出すように user.cpp を作成して、もともとある
user.cpp と置き換える。SuzukiMotion() のプロトタイプ宣言は
自分で作成した user.cpp に記述すれば user.h は変更不要。

(2-2)
別の方法として、一連の動きを定義する関数 SuzukiMotion() の
プロトタイプ宣言は user.cpp から見えるように user.h に記述する。
また SuzukiMotion() を UserMotion() から呼び出すように user.cpp 
を書き換える。
当初はこちらの方法を用意したが、後に (2-1) の方法でできるようにした
結果、今では (2-1) の方が簡単だろう。

(3)
自分で作成した関数 ActSuzuki() をこのファイル内でだけ利用するなら、
そのプロトタイプ宣言はこのファイル suzuki.cpp 内に記述すればよい。
int ActSuzuki(C1 m[], float p[]);
一方、他のファイルからも呼び出せるようにするなら、
ActSuzuki() のプロトタイプ宣言は user.h に記述する。

(4)
自分が作った動作を表す任意の数値を以下のように定義する。
const int ACT_SUZUKI = 51;
const.h 内の他の数値 ACT_**** と被らないか事前に確認しておく。
(3) と同じように他のファイルからも利用できるようにする場合は
const.h または user.h に追記する。
*/

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

// 様々な定数の設定は const.h に移動した
#include "const.h"
#include "image.h"
#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h"
#include "lidarclass.h"

// user.h にはこれまでに作成した Act?????(); のプロトタイプ宣言が
// 記述されている。このヘッダを include することで、それらの関数を
// 利用することができる。
#include "user.h"

// LIDAR 測距センサのデータにアクセスするためのポインタ
// LIDAR を使わなければ消せる
extern LIDAR *gLIDAR;

// 自分が作った動作を表す数値、const.h 内の他の数値および他のプログラム内で
// 定義されている数値と被らないか要確認
// suzuki.cpp 内は 200 番台 とする
const int ACT_SUZUKI = 201;
// const int ACT_SUZUKI2 = 202; // 必要なら何個でも定義できる
// const int ACT_SUZUKI3 = 203;
const int ACT_SUZUKIVISION = 204;
const int ACT_SUZUKIGRAB = 205;

// 自分が作った独自の動きを定義する関数のプロトタイプ宣言、
// 他のファイルから参照しないならここに(だけ)書けばよい
int ActSuzuki(C1 m[], float p[]);
// int ActSuzuki2(C1 m[], float p[]); // 必要なら何個でも作れる
// int ActSuzuki3(C1 m[], float p[]);
int ActSuzukiVision(C1 m[], RGBCam *c, DepthCam *d, float p[]);
int ActSuzukiGrab(C1 m[], DepthCam *d, float p[]); // 引数が増えたため
//int ActSuzukiGrab(C1 m[], float p[]);
int SuzukiShell(char token[][STRBUF]);
void SuzukiShellHelp(void);

// SuzukiMotion() のプロトタイプ宣言は user.cpp に記述するか、
// または user.hに存在している必要あり

// 以下の変数の実体は user.cpp にある

extern int act_num;       // 次の行動を決める値、重要
extern int act_last;      // 直前の行動を覚えておく
extern int state;         // 動作の状態を表す、開始、継続中、終了
extern float act_param[]; // パラメータの受け渡し用配列

void SuzukiMotion(C1 m[], RGBCam *c, DepthCam *d)
// m[] はモータコントローラの配列、モータを直接動かすために必要
// *c はRGBカメラの画像処理、画像を扱うために必要
// *d は深度(depth)カメラの画像処理、深度画像を扱うために必要
{
//////////////////////////////////////////////////////////
// 
// 各種の行動を実行する順番や繋がりを定義する
// フローチャートのように記述できる
// 
//////////////////////////////////////////////////////////
    
//  act_param[AP_STATE] = ACT_START;           // ここで実行すると毎回初期化になってまずいので不可

// act_num の次の値は(必要なら動作結果を act_param[] で受け取って)、次の if 文内で決める
    if (ACT_START == state) {                  // プログラム起動直後、最初はここから
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を覚えておく、後で必要になるかも
	act_num = ACT_INITPOSE;                // 最初は初期姿勢、立ち上がる
    }
//  if (ACT_MOVING == state) ; // 動作が継続中の時は特別な理由がなければ指示を変更しない
    if (ACT_END == state) {                    // 直前の行動が終了したら
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を覚えておく、後で必要になるかも

	if (ACT_INITPOSE == act_last) {
	    act_param[AP_NECK_VA] = -0.8;
	    act_num = ACT_NECK;
//	    m[A1J1].duration = 1.0;         // ついでに脚を動かすことも可能
//	    m[A1J1].targetangle.data = 0.0; // 赤い箱を隠さないように左脚を後ろに下げる
	}
	if (ACT_BODYSHIFT == act_last) {
	    act_param[AP_NECK_VA] = -0.8;
	    act_num = ACT_NECK;
	}
	if (ACT_NECK == act_last) {
	    act_num = ACT_SUZUKIVISION;
	}
	if (ACT_SUZUKIVISION == act_last) {
	    if (0 < act_param[AP_OBJFIND]) { // 1個以上見つけたら
// 順番に首を動かすとき
		static int count = 0;
		float ha = 0; // 水平回転角度、-1.57=左向き90度
		int ncase = 26;   // 試行の場合の数
		if (0 == (count % ncase)) {
		    act_param[AP_NECK_HA] =  ha;
		    act_param[AP_NECK_VA] = -0.8; // 下向 0.8rad, 45.8366度
		}
		else if (1 == (count % ncase)) {
		    act_param[AP_NECK_HA] =  ha;
		    act_param[AP_NECK_VA] = -0.6;
		}
		else if (2 == (count % ncase)) {
		    act_param[AP_NECK_HA] =  ha;
		    act_param[AP_NECK_VA] = -0.4;
		}
		else if (3 == (count % ncase)) {
		    act_param[AP_NECK_HA] =  ha;
		    act_param[AP_NECK_VA] = -0.2; // 下向 0.2rad, 11.4592度
		}
		else if (4 == (count % ncase)) {
		    act_param[AP_NECK_HA] =  ha;
		    act_param[AP_NECK_VA] =  0.0;
		}
		else if (5 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.4;
		    act_param[AP_NECK_VA] = -0.8;
		}
		else if (6 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.4;
		    act_param[AP_NECK_VA] = -0.6;
		}
		else if (7 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.4;
		    act_param[AP_NECK_VA] = -0.4;
		}
		else if (8 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.4;
		    act_param[AP_NECK_VA] = -0.2;
		}
		else if (9 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.4;
		    act_param[AP_NECK_VA] =  0.0;
		}
		else if (10 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.785;
		    act_param[AP_NECK_VA] = -0.8;
		}
		else if (11 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.785;
		    act_param[AP_NECK_VA] = -0.6;
		}
		else if (12 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.785;
		    act_param[AP_NECK_VA] = -0.4;
		}
		else if (13 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.785;
		    act_param[AP_NECK_VA] = -0.2;
		}
		else if (14 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.785;
		    act_param[AP_NECK_VA] = -0.0;
		}
		else if (15 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.785;
		    act_param[AP_NECK_VA] =  0.2;
		}
		else if (16 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -0.785;
		    act_param[AP_NECK_VA] =  0.4;
		}
		else if (17 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -1.5708;
		    act_param[AP_NECK_VA] = -0.4;
		}
		else if (18 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -1.5708;
		    act_param[AP_NECK_VA] = -0.2;
		}
		else if (19 == (count % ncase)) {
		    act_param[AP_NECK_HA] = -1.5708;
		    act_param[AP_NECK_VA] =  0.0;
		}
		else if (20 == (count % ncase)) {
		    act_param[AP_NECK_HA] = 0.786;
		    act_param[AP_NECK_VA] = -0.4;
		}
		else if (21 == (count % ncase)) {
		    act_param[AP_NECK_HA] = 0.786;
		    act_param[AP_NECK_VA] = -0.2;
		}
		else if (22 == (count % ncase)) {
		    act_param[AP_NECK_HA] = 0.786;
		    act_param[AP_NECK_VA] = 0.0;
		}
		else if (23 == (count % ncase)) {
		    act_param[AP_NECK_HA] = 2.5;
		    act_param[AP_NECK_VA] = -0.4;
		}
		else if (24 == (count % ncase)) {
		    act_param[AP_NECK_HA] = 2.5;
		    act_param[AP_NECK_VA] = -0.2;
		}
		else if (25 == (count % ncase)) {
		    act_param[AP_NECK_HA] = 2.5;
		    act_param[AP_NECK_VA] = 0.0;
		}

		count++;
		act_param[AP_TIMETOTAL] = 5;
		act_num = ACT_STAY; // 3次元座標の精度確認のため
//		act_num = ACT_SUZUKIGRAB; // 何かものを見つけたら掴む
	    }
	    else {
		act_param[AP_TIMETOTAL] = 5;
		act_num = ACT_STAY;       // 無ければ待機
	    }
	}
	if (ACT_SUZUKIGRAB == act_last) {
	    act_param[AP_TIMETOTAL] = 2;
	    act_num = ACT_STAY;
	}
	if (ACT_SUZUKI == act_last) {
	    act_param[AP_TIMETOTAL] = 2;
	    act_num = ACT_STAY;
	}
	if (ACT_RELAX == act_last) act_num = ACT_STAY;
	if (ACT_STAY == act_last) act_num = ACT_NECK;
	if (ACT_SHELL == act_last) {
//	    act_num = ACT_SUZUKIVISION; // 再開する動作は shell() で決める、ここでは不要
	}
	if (ACT_SHELLSLEEP == act_last) {
	    act_num = ACT_SHELLSLEEP;
	}
// もともと user.cpp で定義されていた関数は action.cpp, action2.cpp に移された。
// 次行のようにaction.cpp, action2.cpp にある関数はこれまでと同様に利用可能
//	if (ACT_WALK == act_last) act_num = ACT_TURN;
//	if (ACT_TURN == act_last) act_num = ACT_WALK;
	if (ACT_ERR == act_last) act_num = ACT_RELAX; // エラー時の次の行動を決める
    }
    if (ACT_ERR == state) act_num = ACT_ERR; // 状態がエラーの時はエラー処理へ

//////////////////////////////////////////////////////////
// 
// 個々の行動の具体的な内容を定義する
// 必要なパラメータをセットして動きを実現する関数を呼び出す
// 
//////////////////////////////////////////////////////////
    
// ここから先は個々の行動に必要なパラメータをセットして関数を呼び出す
    int bsflag[NUMARMS] = {0}; // body shift flag
    switch (act_num) {
        case ACT_INITPOSE : // 初期姿勢、立ち上がる
	    act_param[AP_TIMETOTAL] = 8.0; // 動作時間 (sec)
	    state = ActInitialPose(m, act_param);
	    break;
	case ACT_SUZUKIVISION : // 画像処理
	    act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	    state = ActSuzukiVision(m, c, d, act_param);
	    break;
	case ACT_SUZUKIGRAB : // ものを掴み持ち上げて移動後落とす
	    act_param[AP_TIMETOTAL] = 7.0; // 動作時間 (sec)
	    act_param[AP_TIME1] = 1.0; // 動作時間 (sec)
	    act_param[AP_TIME2] = 2.0; // 動作時間 (sec)
	    act_param[AP_TIME3] = 1.0; // 動作時間 (sec)
	    act_param[AP_TIME4] = 2.0; // 動作時間 (sec)
	    act_param[AP_TIME5] = 1.0; // 動作時間 (sec)
	    state = ActSuzukiGrab(m, d, act_param); // 引数が増えたため
//	    state = ActSuzukiGrab(m, act_param);
	    break;
        case ACT_BODYSHIFT : // 脚先を固定して本体を動かす
	    act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	    act_param[AP_BSHIFTX] = 0.0; // シフト距離 x [m]
	    act_param[AP_BSHIFTY] = 0.0; // シフト距離 y [m]
	    act_param[AP_BSHIFTZ] = 0.1; // シフト距離 z [m]
	    bsflag[0] = ON;
	    bsflag[1] = OFF;
	    bsflag[2] = ON;
	    bsflag[3] = ON;
	    bsflag[4] = OFF;
	    bsflag[5] = ON;
	    act_param[AP_BSHIFTFLAG] = (float)encode_leg(bsflag); // 動かす脚のオンオフの設定
	    state = ActBodyShift(m, act_param);
	    break;
        case ACT_NECK : // 首を動かす
	    act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	    state = ActNeck(m, act_param);
	    break;
	case ACT_SUZUKI : // 動きの例、前の腕を交互に動かす
	    act_param[AP_TIMETOTAL] = 9.0; // 動作時間 (sec)
	    act_param[AP_TIME1] = 3.0; // 動作時間 (sec)
	    act_param[AP_TIME2] = 3.0; // 動作時間 (sec)
	    act_param[AP_TIME3] = 3.0; // 動作時間 (sec)
	    state = ActSuzuki(m, act_param);
	    break;
        case ACT_STAY : // 現状維持
//	    act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
	    state = ActStay(m, act_param);
	    break;
        case ACT_RELAX : // 脱力
//	    act_param[AP_TIMETOTAL] = 3.0; // ActRelax() 中にある 
	    state = ActRelax(m, act_param);
	    break;
        case ACT_SHELL : // shell() による操作のためにここでは何もしない
	    act_param[AP_STATE] = ACT_MOVING;
	    state = ACT_MOVING;
	    break;
        case ACT_SHELLSLEEP : // shell() による操作のためにロボットの動きを静止させて待つ
	    act_param[AP_TIMETOTAL] = 60.0; // 合計動作時間 (sec)
	    state = ActStay(m, act_param);
	    break;
//      case ACT_ERR : // ACT_ERROR -> ACT_ERR の変更に伴ってこの行は不要になった
        default : // どの case にもマッチしないとき、本来ここにはマッチしないはず
	    fprintf(stderr, "Error: Bad act_num specified, or other error occured.\n");
	    state = ACT_END;
            break;
    }
}

// カラー画像、距離画像から検出した物体の3次元ローカル座標を求める
int ActSuzukiVision(C1 m[], RGBCam *c, DepthCam *d, float p[])
{
    return ActVision(m, c, d, p);
}

// 指定した座標の物体を掴む、持ち上げる、落とす
int ActSuzukiGrab(C1 m[], DepthCam *d, float p[]) 
{
    return ActGrabRelease(m, d, p);
}

int ActSuzuki(C1 m[], float p[]) // 3種類の異なる動きを組み合わせる例
{
    static int timer = 0; // 関数が呼び出された回数をカウントする
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
#ifdef ACTEXEC
	fprintf(stderr, "start ActSuzuki()\n");
#endif
	ikparam[0] = time1;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
//	ikparam[3] = (float)MODENONE; // 速い動き
	ikparam[3] = (float)MODEPROFILE; // ゆっくり
//	ikparam[3] = (float)MODEPROFILE2; // 中間
        timer = 0;
    }
// １番目の動き
    if (0 == (timer % cycletotal)) {
	float target1[XYZ] = { 0.65, 0.65, 0.5};
	float target2[XYZ] = { 0.65,-0.65, 0.5};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ２番目の動き
    if (cycle1 == (timer % cycletotal)) { // cycle1 回呼び出されたら次の動きに移る
	ikparam[0] = time2; // 動きによって時間が異なる場合
//	ikparam[1] = IKHAND; // 設定値が前と同じなら省略可
//	ikparam[2] = IKELBOWUP;
//	ikparam[3] = (float)MODEPROFILE;
	float target1[XYZ] = { 0.65, 0.65, 0.0};
	float target2[XYZ] = { 0.65,-0.65, 0.0};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ３番目の動き
    if ((cycle1 + cycle2) == (timer % cycletotal)) { // 次の動きに移る
	ikparam[0] = time3; // 動きによって時間が異なる場合
	float target1[XYZ] = { 0.65, 0.15, 0.0};
	float target2[XYZ] = { 0.65,-0.15, 0.0};
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
#ifdef ACTEXEC
	fprintf(stderr, "end ActSuzuki()\n");
#endif
	return ACT_END;
    }
}

/*
int ActSuzuki2(C1 m[], float p[]) // 自分独自の関数は何個作ってもよい
{

}

int ActSuzuki3(C1 m[], float p[]) // 自分独自の関数は何個作ってもよい
{

}
*/

int     SuzukiShell(char token[][STRBUF])
// UserShell() から呼び出される
{
	if (0 == strcmp("suzuki", token[0])) { // 自分で独自のコマンドを追加可能
	    printf("suzuki\n"); // 複雑な動きの場合は関数にすることも可能
	    return YES; // コマンド名がマッチした場合は YES を返す
        }
	else if (0 == strcmp("suzuki2", token[0])) { // 自分で独自のコマンドを追加可能
	    printf("suzuki2\n");
	    return YES;
        }
	else if (0 == strcmp("suzuki3", token[0])) { // 自分で独自のコマンドを追加可能
	    printf("suzuki3\n");
	    return YES;
        }

	return NO; // コマンド名がマッチしなかった場合
}

void    SuzukiShellHelp(void)
// UserShellHelp() から呼び出される
{
    printf("suzuki         : suzuki's original command\n"); // 独自に追加したコマンドの説明
    printf("suzuki2        : suzuki's original command 2\n");
    printf("suzuki3        : suzuki's original command 3\n");
}
