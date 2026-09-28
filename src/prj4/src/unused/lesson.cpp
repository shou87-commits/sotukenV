// lesson.cpp
/*
これは 6 脚ロボットが実行できる様々な動きをプログラムでどう記述する
かを具体的な例として示す練習用のプログラムである。
このファイルを自分用に別名でコピーして内容を書き換えればそのまま使える。
またここでの書き方をもとにして自分で変更すれば、独自の動きを作成、
記述できる。よって最初はここに含まれる内容を一通り理解することを目指す。


自分が作ったファイルや関数を配布されたプロジェクトに統合する手順

(1)
自分で作成したファイル (例 tanaka.cpp) の名前を prj4/CMakeLists.txt の最後
の方にある次の行に以下のように追加する。１行で書くこと。
add_executable(prg4 src/prg4.cpp src/motion.cpp src/motorclass.cpp src/ikmotion.cpp 
               src/user.cpp src/action.cpp src/action2.cpp src/tanaka.cpp)

(2)
(2-1), (2-2) のどちらかを実行する。(2-1) の方が楽なのでおすすめ、(2-2)
は参考程度の情報として残してある。

(2-1)
user.cpp.lesson を参考にして自分で作成した関数
TanakaMotion() を呼び出すように user.cpp を作成して、もともとある
user.cpp と置き換える。TanakaMotion() のプロトタイプ宣言は
自分で作成した user.cpp に記述すれば user.h は変更不要。

(2-2)
別の方法として、一連の動きを定義する関数 TanakaMotion() の
プロトタイプ宣言は user.cpp から見えるように user.h に記述する。
また TanakaMotion() を UserMotion() から呼び出すように user.cpp 
を書き換える。
当初はこちらの方法を用意したが、後に (2-1) の方法でできるようにした
結果、今では (2-1) の方が簡単だろう。

(3)
自分で作成した関数 ActTanaka() をこのファイル内でだけ利用するなら、
そのプロトタイプ宣言はこのファイル tanaka.cpp 内に記述すればよい。
int ActTanaka(C1 m[], float p[]);
一方、他のファイルからも呼び出せるようにするなら、
ActTanaka() のプロトタイプ宣言は user.h に記述する。

(4)
自分が作った動作を表す任意の数値を以下のように定義する。
const int ACT_TANAKA = 301;
const.h 内の他の数値 ACT_**** と被らないか事前に確認しておく。
(3) と同じように他のファイルからも利用できるようにする場合は
const.h または user.h に追記する。
*/

#include <cmath>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 様々な定数の設定は const.h に移動した
#include "const.h"
#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h"

// user.h にはこれまでに作成した Act?????(); のプロトタイプ宣言が
// 記述されている。このヘッダを include することで、それらの関数を
// 利用することができる。
#include "user.h"

// 自分が作った動作を表す数値、const.h 内の他の数値と被らないか要確認
// suzuki.cpp 内は 200 番台、tanaka.cpp 内は 300 番台、
// lesson.cpp 内は 400 番台 とする
const int ACT_LESSON1 = 401;
const int ACT_LESSON2 = 402; // 必要なら何個でも定義できる
const int ACT_LESSON3 = 403;
const int ACT_LESSON4 = 404;

// 自分が作った独自の動きを定義する関数のプロトタイプ宣言、
// 他のファイルから参照しないならここに(だけ)書けばよい
int ActLesson1(C1 m[], float p[]);
int ActLesson2(C1 m[], float p[]); // 必要なら何個でも作れる
int ActLesson3(C1 m[], float p[]);
int ActLesson4(C1 m[], float p[]);
int LessonShell(char token[][STRBUF]);
void LessonShellHelp(void);

// LessonMotion() のプロトタイプ宣言は user.cpp に記述するか、
// または user.hに存在している必要あり

// 以下の変数の実体は user.cpp にある、すべて重要

extern int act_num;       // 次の行動を決める値
extern int act_last;      // 直前の行動を覚えておく
extern int state;         // 動作の状態を表す、開始、継続中、終了
extern float act_param[]; // パラメータの受け渡し用配列

void LessonMotion(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[])
// m[] はモータコントローラのインスタンスのポインタの配列、モータを直接動かすのに必要
// *c はRGBカメラのインスタンスのポインタ、画像処理、画像を扱うために必要
// *d は深度(depth)カメラのインスタンスのポインタ、深度画像を扱うために必要
// ft[] は力覚・トルクセンサのインスタンスのポインタの配列
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
//  if (ACT_MOVING == state) NOTHING;          // 動作継続中の時は理由がなければ指示を変更しない
    if (ACT_END == state) {                    // 直前の行動が終了したら
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を覚えておく、後で必要になるかも

	if (ACT_INITPOSE == act_last) {
            act_param[AP_NECK_HA] =  0.0; // 水平回転、0.0rad, 正面
            act_param[AP_NECK_VA] = -0.8; // 下向 0.8rad, 45.8366度
            act_num = ACT_NECK;
//          m[A1J1].duration = 1.0;         // ついでに脚を動かすことも可能
//          m[A1J1].targetangle.data = 0.0; // 赤い箱を隠さないように左脚を後ろに下げる
        }
	else if (ACT_NECK == act_last) {
		act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
		act_param[AP_BSHIFTX] = -0.3; // シフト距離 x [m]
		act_param[AP_BSHIFTY] =  0.0; // シフト距離 y [m]
		act_param[AP_BSHIFTZ] =  0.0; // シフト距離 z [m]
		act_num = ACT_BODYSHIFT; // 脚先を固定して本体を動かす
	}
	else if (ACT_VISION == act_last) {
	    if (0 < act_param[AP_OBJFIND]) { // 1個以上見つけたら
		act_num = ACT_GRABRELEASE; // それを掴む、放す
	    }
	    else { // 見つからなかったら
		act_num = ACT_LESSON1; // 次は関節を直接動かす例
	    }
	}
	else if (ACT_GRABRELEASE == act_last) {
	    act_num = ACT_LESSON1; // 次は関節を直接動かす例
	}
	else if (ACT_BODYSHIFT == act_last) {
//	    static int local_param = 0; // 複数の処理を切り替えるためのローカル変数
//	    if (0 == local_param) { // ローカル変数で複数の処理を切り替える、こちらでも可能
	    if (0 == (int)act_param[AP_P1]) { // グローバル変数で複数の処理を切り替える書き方
		act_param[AP_P1] = 1; // この値を変えることで動作を変化させる
//		local_param = 1;    // ローカル変数で複数の処理を切り替える、こちらでも可能
		act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
		act_param[AP_BSHIFTX] =  0.5; // シフト距離 x [m]
		act_param[AP_BSHIFTY] =  0.2; // シフト距離 y [m]
		act_param[AP_BSHIFTZ] =  0.0; // シフト距離 z [m]
		act_num = ACT_BODYSHIFT; // 脚先を固定して本体を動かす
	    }
	    else if (1 == (int)act_param[AP_P1]) { // 同じ動きで複数の処理を切り替える書き方
		act_param[AP_P1] = 2;
//		local_param = 2;    // ローカル変数で複数の処理を切り替える場合
		act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
		act_param[AP_BSHIFTX] = -0.2; // シフト距離 x [m]
		act_param[AP_BSHIFTY] = -0.2; // シフト距離 y [m]
		act_param[AP_BSHIFTZ] = -0.0; // シフト距離 z [m]
		act_num = ACT_BODYSHIFT; // 脚先を固定して本体を動かす
	    }
	    else if (2 == (int)act_param[AP_P1]) {
		act_param[AP_P1] = 0;
//		local_param = 0;    // ローカル変数で複数の処理を切り替える場合
		act_num = ACT_VISION; // カメラ画像から物体検出
	    }
	}
	else if (ACT_LESSON1 == act_last) { // 直前の動きから次の行動を決める
	    act_num = ACT_LESSON2;     // 直前の動きが ACT_LESSON1 なら次は ACT_LESSON2 ということ
	}                              // 次は逆運動学で脚を動かす例
	else if (ACT_LESSON2 == act_last) {
	    act_param[AP_TIMETOTAL] = 9; // 次の動きのパラメータを指定可能
	    act_num = ACT_LESSON3;       // 次は複数の動きを順番に行う下書き方の例
	}
	else if (ACT_LESSON3 == act_last) {
	    act_param[AP_TIMETOTAL] = 2;
	    act_param[AP_P1] = 1;        // この値を変えることでACT_LESSON4 の動作を変化させる
	    act_num = ACT_LESSON4;       // 条件によって実行内容を変化させる例
	}
	else if (ACT_LESSON4 == act_last) { // 条件判断によって次の処理の内容を変える例
	    act_param[AP_TIMETOTAL] = 1;
	    act_param[AP_P1] = rand() % 14; // ここでは乱数により次の実行内容を変えてみる
	    if (0 != act_param[AP_P1]) { // 条件によって実行内容を変化させる例、これは繰り返し
		act_num = ACT_LESSON4;      
	    }
	    else if (0 == act_param[AP_P1]) { // この場合は別の処理 ACT_HAND に進める
		// 1回動かすだけならここからの5行でできる
		act_param[AP_TIMETOTAL] = 1; // 1秒
		act_param[AP_HANDNUM] = LF; // 左前脚のハンド
		act_param[AP_HANDSTATE] = HANDOPEN; // 開く
		act_param[AP_HANDRAD] = 1.57; // PI/2, 90度
		act_num = ACT_HAND; // 次は Hand を開く、閉じる
	    }
	}
	else if (ACT_HAND == act_last) {
	    static int count = 0;
	    if (count++ < 7) { // 開くと閉じるを繰り返す
		act_param[AP_HANDSTATE] = HANDOPEN;
		if (0 == count % 4) {
		    act_param[AP_HANDNUM] = LF;
		    act_param[AP_HANDRAD] = 1.57;
		}
		else if (1 == count % 4) {
		    act_param[AP_HANDNUM] = RF;
		    act_param[AP_HANDRAD] = 1.57;
		}
		else if (2 == count % 4) {
		    act_param[AP_HANDNUM] = LF;
		    act_param[AP_HANDRAD] = 0.0; // 0 度に開く→ 閉じる
		}
		else if (3 == count % 4) {
		    act_param[AP_HANDNUM] = RF;
		    act_param[AP_HANDRAD] = 0.0;
		}
		act_num = ACT_HAND; // 次は Hand を開く、閉じる
	    }
	    else {
		act_param[AP_ITER] = 3;
		act_param[AP_WALK_DIR] = FORWARD; // これは前進、or BACKWARD で後退する
		act_num = ACT_FASTWALK; // 次は高速歩行
	    }
	}
	else if (ACT_FASTWALK == act_last) {
	    act_param[AP_ITER] = 3;
	    act_param[AP_TURN_DIR] = TURNCW; // 時計周り、or TURNCCW は反時計周り
	    act_num = ACT_TURN; // 次はその場で旋回
	}
	else if (ACT_TURN == act_last) {
	    act_param[AP_ITER] = 3;
	    act_param[AP_WALK_DIR] = BACKWARD; // 後退、or FORWARD で前進する
	    act_num = ACT_WALK; // 次は通常歩行
	}
	else if (ACT_WALK == act_last) {
            act_num = ACT_STAY; // 次は動きを止めて待つ
        }
	else if (ACT_STAY == act_last) {
            act_num = ACT_RELAX; // 次は脱力、関節をすべて０度に
        }
	else if (ACT_RELAX == act_last) {
            act_num = ACT_INITPOSE; // 次は初期姿勢、立ち上がる
        }
// もともと user.cpp で定義されていた動作の関数は action.cpp に移された。
// ここまでの例のようにaction.cpp にある関数はこのファイル内の ACT_LESSON? と同様に利用可能
	
// 以下は shell() 関連の指定、最初のうちは触らなくてよい	
	else if (ACT_SHELL == act_last) {
//	    act_num = ACT_LESSONVISION; // 再開する動作は shell() で決める、ここでは指定不要
	}
	else if (ACT_SHELLSLEEP == act_last) {
	    act_num = ACT_SHELLSLEEP;
	}
	else if (ACT_ERR == act_last) act_num = ACT_RELAX; // エラー時の次の行動を決める
    }
    if (ACT_ERR == state) act_num = ACT_ERR; // 状態がエラーの時はエラー処理へ

//////////////////////////////////////////////////////////
// 
// 個々の行動の具体的な内容を定義する
// 必要なパラメータをセットして動きを実現する関数を呼び出す
// 
//////////////////////////////////////////////////////////
    
    int bsflag[NUMARMS] = {0}; // ACT_BODYSHIFT 用、以下の case 内に書けずやむを得ずここに

// ここから先は個々の行動に必要なパラメータをセットして関数を呼び出す
    switch (act_num) {
        case ACT_INITPOSE : // 初期姿勢、立ち上がる
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActInitialPose(), 初期姿勢、立ち上がる\n");
	    act_param[AP_TIMETOTAL] = 5.0; // 動作に関連するパラメータはここでも指定可能
	    state = ActInitialPose(m, act_param);
	    break;
	case ACT_LESSON1 : // 動きの例、関節を指定して動かす
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActLesson1(), 関節を指定して個別に回転させる\n");
	    act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
	    state = ActLesson1(m, act_param);
	    break;
	case ACT_LESSON2 : // 動きの例、逆運動学で腕を動かす
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActLesson2(), 逆運動学で腕を動かす\n");
	    act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
	    state = ActLesson2(m, act_param);
	    break;
	case ACT_LESSON3 : // 複数の動きを順番に動かす、常に順番が同じ時はこの書き方が可能
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActLesson3(), 複数の動きを順番に動かす\n");
	    act_param[AP_TIMETOTAL] = 10.0; // 合計動作時間 (sec)
	    act_param[AP_TIME1] = 2.0; // 動作１の時間 (sec)、合計が一致すべし
	    act_param[AP_TIME2] = 3.5; // 動作２の時間 (sec)
	    act_param[AP_TIME3] = 2.5; // 動作３の時間 (sec)
	    act_param[AP_TIME4] = 2.0; // 動作４の時間 (sec)
	    state = ActLesson3(m, act_param);
	    break;
        case ACT_LESSON4 : // 条件により動きを変化させる
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActLesson4(), 外部からの条件により動きを変化させる\n");
	    state = ActLesson4(m, act_param);
	    break;
	case ACT_VISION : // 画像処理
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActVision(), カメラ画像から色や輪郭を用いて物体を検出する\n");
	    act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	    state = ActVision(m, c, d, act_param);
	    break;
	case ACT_GRABRELEASE : // ものを掴み持ち上げて移動後落とす
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActGrabRelease(), ものを掴み持ち上げてロボット前に移動後放す\n");
	    act_param[AP_TIMETOTAL] = 7.0; // 動作時間 (sec)
	    act_param[AP_TIME1] = 1.0; // 動作時間 (sec)
	    act_param[AP_TIME2] = 2.0; // 動作時間 (sec)
	    act_param[AP_TIME3] = 1.0; // 動作時間 (sec)
	    act_param[AP_TIME4] = 2.0; // 動作時間 (sec)
	    act_param[AP_TIME5] = 1.0; // 動作時間 (sec)
	    state = ActGrabRelease(m, act_param);
	    break;
        case ACT_BODYSHIFT : // 脚先を固定して本体を動かす
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActBodyShift(), 脚先を固定して本体を動かす\n");
// 動作時間とシフト距離はここで指定するか、この関数の前半で指定するか、どちらも可能
//	    act_param[AP_TIMETOTAL] = 10.0; // 合計動作時間 (sec)
//	    act_param[AP_BSHIFTX] = 0.3; // シフト距離 x [m]
//	    act_param[AP_BSHIFTY] = 0.2; // シフト距離 y [m]
//	    act_param[AP_BSHIFTZ] = 0.1; // シフト距離 z [m]

//	    int bsflag[NUMARMS] = {0}; // body shift flag 本来ここでよいがここには書けない
	    bsflag[0] = ON;
	    bsflag[1] = ON; // OFF にすると動かさない
	    bsflag[2] = ON;
	    bsflag[3] = ON;
	    bsflag[4] = ON;
	    bsflag[5] = ON;
	    act_param[AP_BSHIFTFLAG] = (float)encode_leg(bsflag); // 動かす脚のオンオフの設定
	    state = ActBodyShift(m, act_param);
	    break;
        case ACT_HAND : // Hand を開く、閉じる
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActHand(), Hand を開く、閉じる\n");
	    act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	    state = ActHand(m, act_param);
	    break;
        case ACT_NECK : // 首を動かす
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActNeck(), 首を動かす\n");
	    act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	    state = ActNeck(m, act_param);
	    break;
	case ACT_WALK : // 普通の歩行
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActWalk(), 通常歩行\n");
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
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActWalk(), 高速歩行\n");
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
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActTurn(), その場で転回、旋回する\n");
	    act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	    act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
//	    act_param[AP_ITER]  = 1;           // １組の行動の繰り返し回数、１回で約３０度回転
	    act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
	    act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
	    state = ActTurn(m, act_param);
	    break;
        case ACT_STAY : // 現状維持
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActStay(), 動きを止めて現状維持\n");
	    act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
	    state = ActStay(m, act_param);
	    break;
        case ACT_RELAX : // 脱力
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActRelax(), 脱力する\n");
//	    act_param[AP_TIMETOTAL] = 3.0; // ActRelax() 中にある 
	    state = ActRelax(m, act_param);
	    break;

// ここから関数の末尾までは最初は気にしなくてよい
	    
        case ACT_SHELL : // shell() による操作のためにここでは何もしない
	    act_param[AP_STATE] = ACT_MOVING;
	    state = ACT_MOVING;
	    break;
        case ACT_SHELLSLEEP : // shell() による操作のためにロボットの動きを静止させて待つ
	    act_param[AP_TIMETOTAL] = 10.0; // 合計動作時間 (sec)
	    state = ActStay(m, act_param);
	    break;
//      case ACT_ERR : // ACT_ERROR -> ACT_ERR の変更に伴ってこの行は不要になった
        default : // どの case にもマッチしないとき、本来ここにはマッチしないはず
	    fprintf(stderr, "Error: Bad act_num %d specified, or other error occured.\n", act_num);
	    state = ACT_END;
            break;
    }
}

int ActLesson1(C1 m[], float p[])
{
    static int timer = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	// A1J1 は1番の arm の1番の joint 、左前脚の肩
	m[A1J1].ctrlmode = MODENONE; // モータ1個にパラメータ3個、これは制御モード PID
        m[A1J1].duration = duration; // 回転時間
	m[A1J1].targetangle.data = -1.3; // 目標角度 radian
	
	m[A1J2].ctrlmode = MODENONE; // ここから2個めのモータのパラメータ
        m[A1J2].duration = duration;
	m[A1J2].targetangle.data = 1.3; // radian
        timer = 0;
    }    
    timer++;
    if (timer < cycletotal) { // 動作の継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // 最後はタイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
	return ACT_END;
    }
}

int ActLesson2(C1 m[], float p[]) // 逆運動学で脚を動かす
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; 
    int cycletotal = (int)(duration*100);

// 動作の指示はここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	ikparam[0] = duration; // 逆運動学ではパラメータは4個、これは動く時間
	ikparam[1] = IKHAND; // これは手先を目標座標に動かす指定、手首ならIKWRIST
	ikparam[2] = IKELBOWUP; // これは肘を上向きにする指定、下ならIKELBOWDOWN
	ikparam[3] = (float)MODENONE; // 制御モード、モータの指定と同じ内容
        timer = 0;
    }
    if (0 == (timer % cycletotal)) {
	float target1[XYZ] = { 0.65, 0.15, 0.1}; // 原点（本体重心）からの目標座標
	float target2[XYZ] = { 0.65,-0.15, 0.5};
	IKMotions(m, LF, target1, ikparam); // 左前脚を target1 に動かす
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

int ActLesson3(C1 m[], float p[]) // 3種類の異なる逆運動学の動きを組み合わせる例
{
    static int timer  = 0; // 関数が呼び出された回数をカウントする
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
    float time1 = p[AP_TIME1]; // 動き１の時間、秒
    float time2 = p[AP_TIME2]; // 動き２の時間、秒
    float time3 = p[AP_TIME3]; // 動き３の時間、秒
    float time4 = p[AP_TIME4]; // 動き４の時間、秒
    int cycletotal = (int)(duration*100); // 動作終了までに関数が呼び出される回数
    int cycle1 = (int)(time1*100); // 動き１の動作終了までに関数が呼び出される回数
    int cycle2 = (int)(time2*100); // 動き２の動作終了までに関数が呼び出される回数
    int cycle3 = (int)(time3*100); // 動き３の動作終了までに関数が呼び出される回数
    int cycle4 = (int)(time4*100); // 動き４の動作終了までに関数が呼び出される回数

// 動作の指示はここから ////////////////////////////////

// 複数の動きを順番に行わせるときの書き方
// 初期設定は最初に１回
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { 
#ifdef ACTEXEC
	fprintf(stderr, "start ActLesson()\n");
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
	fprintf(stderr, "ActLesson3(), 1番目の動き\n");
	float target1[XYZ] = { 0.65, 0.35, 0.5};
	float target2[XYZ] = { 0.65, 0.05, 0.5};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ２番目の動き
    if (cycle1 == (timer % cycletotal)) { // cycle1 回呼び出されたら次の動きに移る
	fprintf(stderr, "ActLesson3(), 2番目の動き\n");
	ikparam[0] = time2; // 動きによって時間が異なる場合
//	ikparam[1] = IKHAND; // 設定値が前と同じなら省略可
//	ikparam[2] = IKELBOWUP;
//	ikparam[3] = (float)MODEPROFILE;
	float target1[XYZ] = { 0.65, 0.35, 0.1};
	float target2[XYZ] = { 0.65, 0.05, 0.1};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ３番目の動き
    if ((cycle1 + cycle2) == (timer % cycletotal)) { // 次の動きに移る
	fprintf(stderr, "ActLesson3(), 3番目の動き\n");
	ikparam[0] = time3; // 動きによって時間が異なる場合
	float target1[XYZ] = { 0.65,-0.05, 0.1};
	float target2[XYZ] = { 0.65,-0.35, 0.1};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ４番目の動き
    if ((cycle1 + cycle2 + cycle3) == (timer % cycletotal)) { // 次の動きに移る
	fprintf(stderr, "ActLesson3(), 4番目の動き\n");
	ikparam[0] = time4; // 動きによって時間が異なる場合
	float target1[XYZ] = { 0.65,-0.05, 0.5};
	float target2[XYZ] = { 0.65,-0.35, 0.5};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
/*  if ((cycle1 + cycle2 + cycle3 + cycle4) == (timer % cycletotal)) {
//  この処理は次でされるのでここでは不要
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
	fprintf(stderr, "end ActLesson()\n");
#endif
	return ACT_END;
    }
}

int ActLesson4(C1 m[], float p[]) // 条件判断により動きを変化させる例
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; 
    int cycletotal = (int)(duration*100);

// 動作の指示はここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	ikparam[0] = duration; // 逆運動学ではパラメータは4個、これは動く時間
	ikparam[1] = IKHAND; // これは手先を目標座標に動かす指定、手首ならIKWRIST
	ikparam[2] = IKELBOWUP; // これは肘を上向きにする指定、下ならIKELBOWDOWN
	ikparam[3] = (float)MODENONE; // 制御モード、モータの指定と同じ内容
        timer = 0;
    }
    if (0 == (timer % cycletotal)) {
	float h = p[AP_P1] / 20.0;
	if (0 == (int)p[AP_P1] % 2) {
	    float target1[XYZ] = { 0.65, 0.15, h};
	    IKMotions(m, LF, target1, ikparam);
	}
	else {
	    float target2[XYZ] = { 0.65,-0.15, h};
	    IKMotions(m, RF, target2, ikparam);
	}
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

// UserShell() を使わないうちはこれ以降は触らなくても大丈夫

int     LessonShell(char token[][STRBUF])
// UserShell() から呼び出される
{
	if (0 == strcmp("lesson", token[0])) { // 自分で独自のコマンドを追加可能
	    printf("lesson\n"); // 複雑な動きの場合は関数にすることも可能
	    return YES; // コマンド名がマッチした場合は YES を返す
        }
	else if (0 == strcmp("lesson2", token[0])) { // 自分で独自のコマンドを追加可能
	    printf("lesson2\n");
	    return YES;
        }
	else if (0 == strcmp("lesson3", token[0])) { // 自分で独自のコマンドを追加可能
	    printf("lesson3\n");
	    return YES;
        }

	return NO; // コマンド名がマッチしなかった場合
}

void    LessonShellHelp(void)
// UserShellHelp() から呼び出される
{
    printf("lesson         : lesson's original command\n"); // 独自に追加したコマンドの説明
    printf("lesson2        : lesson's original command 2\n");
    printf("lesson3        : lesson's original command 3\n");
}
