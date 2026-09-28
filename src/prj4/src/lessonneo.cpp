// lessonneo.cpp
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
int ActTanaka(C1 m[], foat p[]);
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

#include <geometry_msgs/WrenchStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 様々な定数の設定は const.h に移動した
#include "const.h"
#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h" // 202507 腹のカメラの3D座標計算で使うことになった
#include "ftclass.h"
#include "robotclass.h"

#include "image.h" // for imFindObjs() and others
    
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


//ーーーーーーーーーーーーーーーー※追加ーーーーーーーーーーーーーーーーー
const int ACT_NECKFIND = 405;
const int ACT_NECKLEFT = 406;
const int ACT_NECKRIGHT = 407;
const int ACT_NECKFRONT = 408;
const int ACT_GRL = 409; // GrabReleaseLeftの略
const int ACT_NVCFH = 410; // Hはハリネズミ
const int ACT_NVCFH_L = 411; // LはLeft
const int ACT_NVFFD = 412; // Fenceを見つける
const int ACT_WALKTF = 413; // Fenceまで歩く
const int ACT_GRABFENCE_F = 414; // 前足でフェンスを掴む
//const int ACT_GRABFENCE_M = 415; // 中足でフェンスを掴む
const int ACT_GRABUF = 416; // バンザイ
const int ACT_BSTOFENCE_F = 417; // 前足で柵を掴んで引き寄せる
const int ACT_GRABFENCE_ML = 418; // 左中足で柵を掴む

int CallWalkToFence(C1 m[], int next);

int CallNeckFind(C1 m[], int next);
int CallNeckLeft(C1 m[], int next);
int CallNeckRight(C1 m[], int next);
int CallNeckFront(C1 m[], int next);

int CallGrabUp_Front(C1 m[], int next);
int CallGrabFence_Front(C1 m[], DepthCam *d, int next);
int CallGrabFence_MiddleLeft(C1 m[], DepthCam *d, int next);
//int CallGrabFence_Middle(C1 m[], DepthCam *d, int next);
int CallGrabReleaseLeft(C1 m[], DepthCam *d, int next);


int CallBodyShiftToFence_Front(C1 m[], int next);

int CallNVCFindH(C1 m[], RGBCam *c, DepthCam *d, int next);
int CallNVCFindH_Left(RGBCam *c, DepthCam *d, int next);
int CallNVFindFenceDepth(RGBCam *c, DepthCam *d, int next);

int ActWalkToFence(C1 m[], float p[]);
int ActGrabUp_Front(C1 m[], float p[]);
int ActGrabFence_Front(C1 m[], DepthCam *d, float p[]);
int ActGrabFence_MiddleLeft(C1 m[], DepthCam *d, float p[]);
//int AcgGrabFence_Middle(C1 m[], DepthCam *d, float p[]);
int ActGrabReleaseLeft(C1 m[], DepthCam *d, float p[]);
int ActBodyShiftToFence_Front(C1 m[], float p[]);


//ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー

// センサの状態の確認や、その結果により現在実行中の動作や次の動作を変更できる関数
void Think(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[]);

// ロボットの移動経路など、周期的に位置と向きを URDF 形式で記録する
void Log();

// 各動作の次の動作の初期設定、以前は定数だったが変数、配列に変更した。
void SetNextMotion();

// 次の関数群は動作選択の仕組みが Call????() に変わって追加された
int CallInitialPose(C1 m[], int next);
int CallLesson1(C1 m[], int next);
int CallLesson2(C1 m[], int next);
int CallLesson3(C1 m[], int next);
int CallLesson4(C1 m[], int next);
int CallNeck(C1 m[], int next);
int CallHatch(C1 m[], int next);
int CallHatchOpen(C1 m[], int next);
int CallHatchClose(C1 m[], int next);
int CallHand(C1 m[], int next);
int CallBodyShift(C1 m[], int next);
int CallTilt(C1 m[], int next);

int CallSetRobotPos(float xyzrpy[], int next);
int CallGetRobotPos(float xyzrpy[], int next);

int CallNVisionColor(C1 m[], RGBCam *c, DepthCam *d, int next); // m[]を追加し現在のモータの角度を取得し続けられるように修正
int CallNVisionDepth(RGBCam *c, DepthCam *d, int next);
int CallNVisionColorOld(RGBCam *c, DepthCam *d, int next); 	
int CallNVisionDepthOld(RGBCam *c, DepthCam *d, int next);
int CallNVisionDepthOldOld(RGBCam *c, DepthCam *d, int next);

int SimpleVision(C1 m[], RGBCam *c, DepthCam *d, int next); // ACT_VISION に仮に割り当てられている
int CallVision(C1 m[], RGBCam *c, DepthCam *d, int next);
int CallVisionFront(C1 m[], RGBCam *c, DepthCam *d, int next);
int CallVisionLeft(C1 m[], RGBCam *c, DepthCam *d, int next);
int CallVisionRight(C1 m[], RGBCam *c, DepthCam *d, int next);
int CallVision3Cam(C1 m[], RGBCam *c, DepthCam *d, int next);
int CallVisionDemo(C1 m[], RGBCam *c, DepthCam *d, int next);

int CallFTDemo(C1 m[], FTSENSOR ft[], int next);
int CallGrabRelease(C1 m[], DepthCam *d, int next);
int CallWalkP1(C1 m[], int next); // 通常歩行、前進
int CallWalkP2(C1 m[], int next); // 高速歩行、前進
int CallWalkP3(C1 m[], int next); // 精密歩行、前進
int CallWalkP4(C1 m[], int next); // 通常歩行、後進
int CallWalk(C1 m[], int next); // 直接 call してはいけない
int CallLegsP1(C1 m[], int next); // 任意の脚を動かす
// 次の関数は直接 call してはいけない、CallLegsP1()を経由する
int CallLegs(C1 m[], int next, float target[NUMARMS][MAXPHASE][XYZ], int onflag[NUMARMS]); 
// int CallFastWalk(C1 m[], int next); // 削除した
int CallTurnP1(C1 m[], int next); // 旋回、時計回り
int CallTurnP2(C1 m[], int next); // 旋回、反時計回り
int CallTurnP3(C1 m[], int next); // 精密旋回、時計回り
int CallTurnP4(C1 m[], int next); // 精密旋回、反時計回り
int CallTurn(C1 m[], int next); // 直接 call してはいけない
int CallStay(C1 m[], int next);    
int CallRelax(C1 m[], int next);
int CallShell(C1 m[], int next);
int CallShellSleep(C1 m[], int next);

// 自分が作った独自の動きを定義する関数のプロトタイプ宣言、
// 他のファイルから参照しないならここに(だけ)書けばよい
int ActLesson1(C1 m[], float p[]);
int ActLesson2(C1 m[], float p[]); // 必要ならこのような関数を何個でも作ってよい
int ActLesson3(C1 m[], float p[]);
int ActLesson4(C1 m[], float p[]);
int LessonShell(char token[][STRBUF]);
void LessonShellHelp(void);

// LessonMotion() のプロトタイプ宣言は user.cpp に記述するか、
// または user.hに存在している必要あり

// 以下の変数の実体は user.cpp にある、すべて重要
extern int act_num;       // 現在の行動を表す値
extern int act_last;      // 直前の行動を覚えておく
extern int state;         // 動作の状態を表す、開始、継続中、終了
extern float act_param[]; // パラメータの受け渡し用配列

// 次の変数は動作選択の仕組みが Call????() に変わって追加された
// 別の場所で必要になったので user.cpp に移動した
//extern int act_next;      // 次の行動を指定する値、2023年8月に追加、202603以後不使用

// 上記 act_next の代わりに次の行動の指定の仕方を配列を使うように変更した
extern int next_a[];      // 次の行動を指定する値の配列、2026年3月に追加された

extern ROBOT *gRobot;


void LessonMotion(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[])
// m[] はモータコントローラのインスタンスへのポインタの配列、モータを直接動かすのに必要
// *c はRGBカメラのインスタンスへのポインタ、画像処理、画像を扱うために必要
// *d は深度(depth)カメラのインスタンスへのポインタ、深度画像を扱うために必要
// ft[] は力覚・トルクセンサのインスタンスへのポインタの配列
{
// 各種の行動を実行する順番や繋がりを Call????() で定義する
    
//  act_param[AP_STATE] = ACT_START;           // ここで実行すると毎回初期化になってまずいので不可

// 以下の4個の if 文で行う処理は、 Call????() を導入後変更されたので注意する。
// act_num, act_next の機能の変更に伴うものである

// state の取り得る値は以下の if 文の4種類。
// ACT_START == state は起動直後のみで、その後はならない。
//     代わりに act_param[AP_STATE] == ACT_START によって動作開始を表す。
// ACT_MOVING == state は動作中なので何もすることはない。
// ACT_END == stateは動作終了で次の動作に移る。
// ACT_ERR == state はエラーが起きたときで、通常は関係ない。
    
    if (ACT_START == state) {                  // プログラム起動直後、最初「だけ」ここから
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_num = ACT_INITPOSE;                // 最初の動作は ACT_INITPOSE
	act_last = ACT_NONE;                   // 直前の動作はまだない
//	act_next = ACT_INITPOSE;               // 次の動作を指定、Call????() 用
	SetNextMotion();                       // 各動作の次の動作の初期設定をする
    }
//  if (ACT_MOVING == state) NOTHING;          // 動作継続中の時は理由がなければ指示を変更しない
    if (ACT_END == state) {                    // 直前の行動が終了したら次を決める
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_num = next_a[act_num];
	act_last = act_num;                    // 直前の行動を記録、後で必要になるかもしれないので
//      act_next = ???;                        // act_next は直前の Call??() 末尾で指定、ここでは不要
    }
    if (ACT_ERR == state) {                    // 状態がエラーの時の処理
	act_last = act_num;                    // 直前の動作を記録
	act_param[AP_STATE] = ACT_START;       // 今までの動きを終了
	act_param[AP_TIMETOTAL] = 60.0;
	act_num = ACT_STAY;                    // 60 秒間動きを停止
//	act_next = ACT_STAY;                   // 60 秒間動きを停止、202603以後ここでは指定不要
	fprintf(stderr, "ACT_ERR was detected. 60 sec stopped.\n");
    }

// ロボットの位置や向きをログファイルに記録する
    
    Log();
    
// センサの状態の確認や、その結果により現在実行中の動作や
// 次の動作を変更できる関数 Think() を新たに用意した。

    Think(m, c, d, ft);
    
// 通常は以下の Call????() のどれかが条件にマッチして対応する動作が実行される

    float xyzrpy[6] = {0, 0, 0.3, 0, 0, 0};
    
    switch (act_num) {
    case ACT_INITPOSE : CallInitialPose(m, 0); break; // 第二引数は使わなくなったので 0 でも可
//  初期姿勢、停止、脱力はいつでも使えるように
    case ACT_STAY : CallStay(m, next_a[ACT_STAY]); break; // 動きを止める、第二引数は 0 でも可、他も同様
    case ACT_RELAX : CallRelax(m, next_a[ACT_RELAX]); break; // 脱力、全関節を0度に
    
// 202507 以後のデモ、CallNV????() と Hough 変換, CallWalkP1() など
    case ACT_NECK : CallNeck(m, next_a[ACT_NECK]); break; // 首を動かす
    case ACT_HATCH : CallHatch(m, next_a[ACT_HATCH]); break; // 腹下のハッチの開け閉め
    case ACT_HATCHOPEN : CallHatchOpen(m, next_a[ACT_HATCHOPEN]); break; // 腹下のハッチの開け
    case ACT_HATCHCLOSE : CallHatchClose(m, next_a[ACT_HATCHCLOSE]); break; // 腹下のハッチの閉め

// 新しいカメラ、ビジョンの機能 NVFOC: NewVisionFindObjsColor, NVFOD: NewVisionFindObjsDepth
    case ACT_NVFOC : CallNVisionColor(m, c, d, next_a[ACT_NVFOC]); break; // new vision 正面color
    case ACT_NVFOD : CallNVisionDepth(c, d, next_a[ACT_NVFOD]); break; // new vision 腹右depth
//  case ACT_NVFOC : CallNVisionColor(c, d, next_a[ACT_NVFOC]); break;// これで 正面 Camera の画像処理
//  case ACT_NVFOC : CallNVisionColor(c+1, d+1, next_a[ACT_NVFOC]); break;// これで 腹左 Camera 
//  case ACT_NVFOD : CallNVisionDepth(&(c[2]), &(d[2]), next_a[ACT_NVFOD]); break;// これで 腹右 Camera 

    case ACT_WALKP1 : CallWalkP1(m, next_a[ACT_WALKP1]); break; // 通常歩行、前進
    case ACT_WALKP2 : CallWalkP2(m, next_a[ACT_WALKP2]); break;	// 高速歩行、前進
    case ACT_WALKP3 : CallWalkP3(m, next_a[ACT_WALKP3]); break;	// 精密歩行、前進
    case ACT_WALKP4 : CallWalkP4(m, next_a[ACT_WALKP4]); break;	// 通常歩行、後進
    case ACT_TURNP1 : CallTurnP1(m, next_a[ACT_TURNP1]); break; // 旋回、時計回り
    case ACT_TURNP2 : CallTurnP2(m, next_a[ACT_TURNP2]); break; // 旋回、反時計回り
    case ACT_TURNP3 : CallTurnP3(m, next_a[ACT_TURNP3]); break; // 精密旋回、時計回り
    case ACT_TURNP4 : CallTurnP4(m, next_a[ACT_TURNP4]); break; // 精密旋回、反時計回り
    case ACT_LEGSP1 : CallLegsP1(m, next_a[ACT_LEGSP1]); break; // 任意の脚を動かす
    case ACT_GETROBOTPOS : //float xyzrpy[6] = {{0.0}};
                      CallGetRobotPos(xyzrpy, next_a[ACT_GETROBOTPOS]); break; // ロボットの座標読み出し
    case ACT_SETROBOTPOS : //float xyzrpy[6] = {0, 0, 0.3, 0, 0,0};
                      CallSetRobotPos(xyzrpy, next_a[ACT_SETROBOTPOS]); break; // ロボットの座標設定
    case ACT_LESSON1 : CallLesson1(m, next_a[ACT_LESSON1]); break; // 練習用、関節を動かす
    case ACT_LESSON2 : CallLesson2(m, next_a[ACT_LESSON2]); break; // 練習用、逆運動学
    case ACT_LESSON3 : CallLesson3(m, next_a[ACT_LESSON3]); break; // 練習用、複数の動きの組合せ
    case ACT_LESSON4 : CallLesson4(m, next_a[ACT_LESSON4]); break; // 場合分け、複数の次動作を表す
    case ACT_HAND : CallHand(m, next_a[ACT_HAND]); break; // ハンドを動かす
    case ACT_BODYSHIFT : CallBodyShift(m, next_a[ACT_BODYSHIFT]); break; // 脚先を固定して体をずらす
    case ACT_TILT : CallTilt(m, next_a[ACT_TILT]); break; // 脚先を固定して体を傾ける
// 次の動きは赤い箱を見つけてつまみ上げるデモ
    case ACT_GRABRELEASE : CallGrabRelease(m, d, next_a[ACT_GRABRELEASE]); break; 
//  case ACT_WALK : CallFastWalk(m, next_a[ACT_FastWalk]); break; // CallFastWalk() は削除された

// CallVision() 関連    
    case ACT_VISIONFRONT : CallVisionFront(m, c, d, next_a[ACT_VISIONFRONT]); break; // 前カメラ
    case ACT_VISIONLEFT : CallVisionLeft(m, c, d, next_a[ACT_VISIONLEFT]); break; // 左腹カメラ
    case ACT_VISIONRIGHT : CallVisionRight(m, c, d, next_a[ACT_VISIONRIGHT]); break; // 右腹カメラ
//  case ACT_  : SimpleVision(m, c, d, next_a[ACT_SIMPLEVISION]); break;// ACT_SIMPLEVISION はない
    case ACT_VISION  : CallVision(m, c, d, next_a[ACT_VISION]); break;// これは Front Camera
//  以下のように書くと CallVision() から左右のカメラ画像を読み出すことも可能
//  case ACT_VISION : CallVision(m, c+1, d+1, next_a[ACT_Vision]); break;// これで Left Camera を選択
//  case ACT_VISION : CallVision(m, &(c[2]), &(d[2]), next_a[ACT_Vision]); break;// これで Right Camera 
    case ACT_VISION3CAM : CallVision3Cam(m, c, d, next_a[ACT_VISION3CAM]); break; // 3カメラ
    case ACT_VISIONDEMO : CallVisionDemo(m, c, d, next_a[ACT_VISIONDEMO]); break; // 画像関連のデモ

// FT センサ 関連
    case ACT_FTDEMO : CallFTDemo(m, ft, next_a[ACT_FTDEMO]); break; // FTセンサのデモ

// Shell 関連
    case ACT_SHELL : CallShell(m, next_a[ACT_SHELL]); break; // シェル、コマンド入力で動かせる
    case ACT_SHELLSLEEP : CallShellSleep(m, next_a[ACT_SHELLSLEEP]); break; // シェル、一時停止
    
    
    
//ーーーーーーーーーーーーーーーーーー※追加ーーーーーーーーーーーーーーーーーーーーーーーーーーー
	case ACT_WALKTF : CallWalkToFence(m, next_a[ACT_WALKTF]); break; // フェンスを見つけたら歩く
	
	case ACT_NECKFIND : CallNeckFind(m, next_a[ACT_NECKFIND]); break; // 画像処理で認識した物体の方向に首を向ける
	case ACT_NECKLEFT : CallNeckLeft(m, next_a[ACT_NECKLEFT]); break; // 首を左下に向ける
	case ACT_NECKRIGHT : CallNeckRight(m, next_a[ACT_NECKRIGHT]); break; // 首を右下に向ける
	case ACT_NECKFRONT : CallNeckFront(m, next_a[ACT_NECKFRONT]); break; // 首を正面に向ける
	
	case ACT_GRABUF : CallGrabUp_Front(m, next_a[ACT_GRABUF]); break; // バンザイ
	case ACT_GRABFENCE_F : CallGrabFence_Front(m, d, next_a[ACT_GRABFENCE_F]); break; // 前足で柵を掴む
	case ACT_GRABFENCE_ML : CallGrabFence_MiddleLeft(m, d, next_a[ACT_GRABFENCE_ML]); break; // 前足で柵を掴む
//	case ACT_GRABFENCE_M : CallGrabFence_Middle(m, d, next_a[ACT_GRABFENCE_M]); break; // 前足で柵を掴む
	case ACT_GRL : CallGrabReleaseLeft(m, d, next_a[ACT_GRL]); break; // 左側の足でハリネズミをつかむ
	case ACT_BSTOFENCE_F : CallBodyShiftToFence_Front(m, next_a[ACT_BSTOFENCE_F]); break; // 前足で引き寄せる 
	
	case ACT_NVCFH : CallNVCFindH(m, c, d, next_a[ACT_NVCFH]); break; // ハリネズミを見つける
	case ACT_NVCFH_L : CallNVCFindH_Left(c+1, d+1, next_a[ACT_NVCFH_L]); break; // ハリネズミを見つける
	case ACT_NVFFD : CallNVFindFenceDepth(c, d, next_a[ACT_NVFFD]); break; // フェンスを見つける
	
//ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー


	
    default : 
// どの Call????() にもマッチしないときの処理、本来ここには達しないはず
    fprintf(stderr, "Error: Bad next_a[] %d specified, or other error occured.\n", 0);
    state = ACT_ERR; break;
    }
    return;
}

void Think(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[])
// ロボット全体の動作を調整、制御することを考える
{
// 引数として受け取った m, c, d, ft によりセンサやモータにアクセス可

/* 以下の変数が参照、変更可能
    int act_last;      // 直前の行動を覚えておく
    int act_num;       // 現在の行動を表す値
//  int act_next;      // 次の行動を表す値 
    int state;         // 動作の状態を表す、開始、継続中、終了
    float act_param[]; // パラメータの受け渡し用配列
*/

// ここで act_next を変更すると次の動作が変わる
// act_param[] を変更すると動作の特性を随時変更できる。
}

void Log()
// 指定した周期でロボットの位置や向きを出力する
{
    static long int clock = 0;
    static FILE *fp;
    static int pn = 1;
    float xyzrpy[6] = {{0.0}};
    
    if (0 == clock) { // 最初の１回
	if ((FILE *)NULL == (fp = fopen("xyzrpy.urdf", "w"))) {
	    fprintf(stderr, "Error: [%s] can not open\n", "xyzrpy.urdf");
	    exit(1);
	}
	gRobot -> getrobotpos(xyzrpy); // 配列に読み込む
	float x = xyzrpy[0], y = xyzrpy[1], z = xyzrpy[2];
	fprintf(fp, "<?xml version=\"1.0\" ?>\n<robot name=\"points\">\n<link name=\"p0\">\n"); // URDF で記録
	fprintf(fp, "<visual>\n<origin xyz=\"%.3f %.3f %.3f\" rpy=\"0 0 0\" />\n", x, y, z);
	fprintf(fp, "<geometry> <box size=\"0.2 0.2 0.2\"/> </geometry>\n</visual>\n");
	fprintf(fp, "<collision>\n<origin xyz=\"%.3f %.3f %.3f\" rpy=\"0 0 0\" />\n", x, y, z);
	fprintf(fp, "<geometry> <box size=\"0.2 0.2 0.2\"/> </geometry>\n</collision>\n");
        fprintf(fp, "<inertial>\n<origin xyz=\"%.3f %.3f %.3f\" rpy=\"0 0 0\" />\n", x, y, z);
        fprintf(fp, "<mass value=\"0.1\" />\n<inertia ixx=\"0.001\" ");
        fprintf(fp, "ixy=\"0\" ixz=\"0\" iyy=\"0.001\" iyz=\"0\" izz=\"0.001\" />\n</inertial>\n</link>\n");
        fprintf(fp, "<gazebo reference=\"p0\"> <material>Gazebo/Yellow</material> </gazebo>\n");
//      fprintf(fp, "</robot>\n"); // 継続して記録できるようにここでは閉じない

//	fclose(fp); // 継続して記録できるようにここでは閉じない
    }

    if (0 != clock && 0 == (clock % 500)) { // 記録の周期、500 なら5秒おき
	gRobot -> getrobotpos(xyzrpy); // 現在の位置と向きを配列に読み込む
	float x = xyzrpy[0], y = xyzrpy[1], z = xyzrpy[2];
	fprintf(fp, "\n<joint name=\"j0%d\" type=\"fixed\">\n", pn);
	fprintf(fp, "<parent link=\"p0\"/>\n<child link=\"p%d\"/>\n", pn);
	fprintf(fp, "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n</joint>\n");
	fprintf(fp, "<link name=\"p%d\">\n", pn);
	fprintf(fp, "<visual>\n<origin xyz=\"%.3f %.3f %.3f\" rpy=\"0 0 0\" />\n", x, y, z);
	fprintf(fp, "<geometry> <box size=\"0.2 0.2 0.2\"/> </geometry>\n</visual>\n</link>\n");
        fprintf(fp, "<gazebo reference=\"p%d\"> <material>Gazebo/Yellow</material> </gazebo>\n", pn);
	pn++;
    }
    clock++; // 呼び出された回数
// 終了時に fprintf(fp, "</robot>\n"); fclose(fp); を実行できないので、結果を Gazebo で
// 表示するときには事前に手動でファイル末尾に </robot> を追記する必要あり
// 結果を表示するコマンドは以下になる
// $ roslaunch prj4 prg4_gz.launch.xyzrpy
}

void SetNextMotion() // 各動作の次の動作の初期設定をする
{
// ここでの設定は「後から出た」設定が優先されて残ることに注意

// 立ち上がった後、ハッチの開閉、下を見て赤い小さなハリネズミを見つけてGRABRELEASEを繰り返す
    next_a[ACT_INITPOSE] = ACT_HATCHOPEN;
    next_a[ACT_STAY] = ACT_WALKP1;
    next_a[ACT_RELAX] = ACT_INITPOSE;
    next_a[ACT_NECK] = ACT_VISION; // ACT_NVFOC;
    next_a[ACT_HATCH] = ACT_HATCHOPEN;
    next_a[ACT_HATCHOPEN] = ACT_HATCHCLOSE;
    next_a[ACT_HATCHCLOSE] = ACT_NECK;
    next_a[ACT_NVFOC] = ACT_NVFOD; // new vision 腹右color
    next_a[ACT_NVFOD] = ACT_WALKP1; // new vision 腹右depth
    next_a[ACT_WALKP1] = ACT_WALKP2; // 通常歩行、前進
    next_a[ACT_WALKP2] = ACT_WALKP3; // 高速歩行、前進
    next_a[ACT_WALKP3] = ACT_WALKP4; // 精密歩行、前進
    next_a[ACT_WALKP4] = ACT_TURNP1; // 通常歩行、後進
    next_a[ACT_TURNP1] = ACT_TURNP2; // 旋回、時計回り
    next_a[ACT_TURNP2] = ACT_TURNP3; // 旋回、反時計回り
    next_a[ACT_TURNP3] = ACT_TURNP4; // 精密旋回、時計回り
    next_a[ACT_TURNP4] = ACT_LEGSP1; // 精密旋回、反時計回り
    next_a[ACT_LEGSP1] = ACT_GETROBOTPOS; // 任意の脚を動かす
    next_a[ACT_GETROBOTPOS] = ACT_SETROBOTPOS; // ロボットの座標読み出し
    next_a[ACT_SETROBOTPOS] = ACT_BODYSHIFT; // ロボットの座標設定
    next_a[ACT_LESSON1] = ACT_LESSON2;
    next_a[ACT_LESSON2] = ACT_LESSON3;
    next_a[ACT_LESSON3] = ACT_LESSON4;
    next_a[ACT_LESSON4] = ACT_STAY; // ACT_BRANCH; 今後は使わない
    next_a[ACT_HAND] = ACT_STAY; // ACT_BRANCH;
    next_a[ACT_BODYSHIFT] = ACT_TILT; 
    next_a[ACT_TILT] = ACT_RELAX; 
    next_a[ACT_GRABRELEASE] = ACT_VISION; // ACT_LESSON1;
    next_a[ACT_VISIONFRONT] = ACT_VISION;
    next_a[ACT_VISIONLEFT] = ACT_VISIONRIGHT;
    next_a[ACT_VISIONRIGHT] = ACT_VISIONFRONT;
//  next_a[ACT_SIMPLEVISION] = ACT_BRANCH; // ACT_SIMPLEVISION は存在しない
    next_a[ACT_VISION] = ACT_GRABRELEASE; // ACT_BRANCH 今後は使わない
    next_a[ACT_VISION3CAM] = ACT_INITPOSE;
    next_a[ACT_VISIONDEMO] = ACT_STAY; // ACT_BRANCH;
    next_a[ACT_FTDEMO] = ACT_NECK;
    next_a[ACT_SHELL] = ACT_SHELL;
    next_a[ACT_SHELLSLEEP] = ACT_SHELLSLEEP;

// 立ち上がった後、正面のカメラで赤い小さなハリネズミを見つけてGRABRELEASEを繰り返す
	next_a[ACT_INITPOSE] = ACT_NECKFRONT;
    next_a[ACT_GRABRELEASE] = ACT_NECKFRONT;
    
    next_a[ACT_NECKFRONT] = ACT_NVCFH;
    next_a[ACT_NECKLEFT] = ACT_NVCFH;
    next_a[ACT_NECKRIGHT] = ACT_NVCFH;
    next_a[ACT_NECKFIND] = ACT_NVCFH;
    
    next_a[ACT_TURNP1] = ACT_NECKFRONT; // 時計回り
    next_a[ACT_TURNP2] = ACT_NECKFRONT; // 半時計回り
    next_a[ACT_WALKP1] = ACT_TURNP1; // 通常歩行の後、時計回りに旋回
    next_a[ACT_WALKP2] = ACT_NECKFRONT; // 結構歩く

// 左カメラで赤いハリネズミを見つけてGRABRELEASEで左側の足で掴む
/*
	next_a[ACT_INITPOSE] = ACT_NVCFH_L;
	next_a[ACT_NVCFH_L] = ACT_NVCFH_L;
	next_a[ACT_GRL] = ACT_NVCFH_L;
*/	
// カメラが認識している座標を確認
/*
	next_a[ACT_INITPOSE] = ACT_NVFOC;
	next_a[ACT_NVFOC] = ACT_STAY;
	next_a[ACT_STAY] = ACT_NVFOC;
*/	

// 正面のDepthカメラで柵を認識する

	next_a[ACT_INITPOSE] = ACT_NVFFD;
	next_a[ACT_NVFFD] = ACT_NVFFD;
	
	next_a[ACT_GRABUF] = ACT_WALKTF;
	
	next_a[ACT_WALKP2] = ACT_NVFFD;
	next_a[ACT_TURNP2] = ACT_NVFFD;
	next_a[ACT_WALKTF] = ACT_NVFFD;
	next_a[ACT_GRABFENCE_F] = ACT_NVFFD;
	next_a[ACT_GRABFENCE_ML] = ACT_NVFFD;
	next_a[ACT_BSTOFENCE_F] = ACT_NVFFD;

// 両手を上げて歩く：テスト
/*
	next_a[ACT_INITPOSE] = ACT_GRABUF;
	next_a[ACT_GRABUF] = ACT_WALKTF;
	next_a[ACT_WALKTF] = ACT_STAY;
*/

	next_a[ACT_INITPOSE] = ACT_WALKP1;
	next_a[ACT_WALKP1] = ACT_WALKP1;
}

int CallInitialPose(C1 m[], int next) // 初期姿勢、立ち上がる
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallInitialPose(), 初期姿勢、立ち上がる\n");
#endif
	act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
    }
    state = ActInitialPose(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}




// --------------------------※追加------------------------------------


int CallNeckLeft(C1 m[], int next) // 首を左に動かす
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallNeckLeft(), 首を左に動かす\n");
#endif
	act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	act_param[AP_NECK_HA] = -(45 * M_PI / 180); // 水平回転、0.0rad, 正面
	act_param[AP_NECK_VA] = -0.6; // 下向 0.6rad, 34度くらい
//      m[A1J1].duration = 1.0;         // おまけ、ついでに脚を動かすことも可能
//      m[A1J1].targetangle.data = 0.0; // 赤い箱を隠さないように左脚を後ろに下げる
    }
    state = ActNeck(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallNeckRight(C1 m[], int next) // 首を右に動かす
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallNeckRight(), 首を右に動かす\n");
#endif
	act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	act_param[AP_NECK_HA] = (45 * M_PI / 180); // 水平回転、0.0rad, 正面
	act_param[AP_NECK_VA] = -0.6; // 下向 0.6rad, 34度くらい
//      m[A1J1].duration = 1.0;         // おまけ、ついでに脚を動かすことも可能
//      m[A1J1].targetangle.data = 0.0; // 赤い箱を隠さないように左脚を後ろに下げる
    }
    state = ActNeck(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallNeckFront(C1 m[], int next) // 首を正面に向ける
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallNeckFront(), 首を正面に向ける\n");
#endif
	act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	act_param[AP_NECK_HA] = 0.1; // 水平回転、0.0rad, 正面
	act_param[AP_NECK_VA] = -0.6; // 下向 0.6rad, 34度くらい
//      m[A1J1].duration = 1.0;         // おまけ、ついでに脚を動かすことも可能
//      m[A1J1].targetangle.data = 0.0; // 赤い箱を隠さないように左脚を後ろに下げる
    }
    state = ActNeck(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}



int CallNeckFind(C1 m[], int next) // 首を動かす
{
	float x = act_param[AP_GRABX];
	float y = act_param[AP_GRABY];
	float z = act_param[AP_GRABZ];
	
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallNeckFind(), 見つけた\n");
#endif
	act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	act_param[AP_NECK_HA] = -atan2(y, x); // 水平回転、0.0rad, 正面
	act_param[AP_NECK_VA] = atan2(z, x) - (10.0 * M_PI / 180.0); // 下向 -10度下に下げる
//      m[A1J1].duration = 1.0;         // おまけ、ついでに脚を動かすことも可能
//      m[A1J1].targetangle.data = 0.0; // 赤い箱を隠さないように左脚を後ろに下げる
    }
    state = ActNeck(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallWalkToFence(C1 m[], int next) // フェンスの縁まで歩く
{
	if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
		fprintf(stderr, "CallWalkToFence(), フェンスの縁まで歩く\n");
		act_param[AP_WALK_DIR] = FORWARD; // FORWARD:前進, BACKWARD:後退する
		act_param[AP_ITER]  = 3;           // １組の行動の繰り返し回数、距離に相当
		act_param[AP_HOHABAX] = 0.1; // 歩幅(m)、前後の振幅(2倍の距離進む)、0.225以下

		act_param[AP_TIME1] = 0.25;         // 動き１の動作時間 (sec)
		act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
		act_param[AP_TIME3] = 0.5;         // 最後に足を初期位置に戻す
		act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
		
		act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
		act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
	    act_param[AP_ITER] + act_param[AP_TIME2] + act_param[AP_TIME3]; // 合計動作時間 (sec) ※TIME3追加：白藤
		}
		state =  ActWalkToFence(m, act_param);
		return CALLMATCH;
}

int CallGrabUp_Front(C1 m[], int next) // バンザーーーーーイ
{
	if (ACT_START == (int)act_param[AP_STATE]) {
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
		fprintf(stderr, "CallGrabUp_Front(), バンザイ\n");
#endif
		act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
		act_param[AP_TIME1] = 3.0; // バンザイ
		act_param[AP_TIME2] = 2.0; // 中足を前へ、後ろ足を後ろへ
//		act_param[AP_TIME3] = 2.0; // 中足を前へ、後ろ足を後へ
		act_num = ACT_GRABUF;
	}
	state = ActGrabUp_Front(m, act_param);
	return CALLMATCH;
}

int CallGrabFence_Front(C1 m[], DepthCam *d, int next) // ものを掴み持ち上げてロボット前に移動後放す
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
		fprintf(stderr, "CallGrabFence_Front(), 前足でフェンスを掴む\n");
#endif
		act_param[AP_TIMETOTAL] = 10.0; // 動作時間 (sec)
		act_param[AP_TIME1] = 2.0; // 動作時間 (sec)
		act_param[AP_TIME2] = 2.0; // 動作時間 (sec)
		act_param[AP_TIME3] = 2.0; // 動作時間 (sec)
		act_param[AP_TIME4] = 2.0; // 動作時間 (sec)
		act_param[AP_TIME5] = 2.0; // 動作時間 (sec)
		act_num = ACT_GRABFENCE_F; // 動作中の動きの登録
	}
		state = ActGrabFence_Front(m, d, act_param);
		return CALLMATCH;
}

int CallGrabFence_MiddleLeft(C1 m[], DepthCam *d, int next) // ものを掴み持ち上げてロボット前に移動後放す
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
		fprintf(stderr, "CallGrabFence_MiddleLeft(), 左中足でフェンスを掴む\n");
#endif
		act_param[AP_TIMETOTAL] = 10.0; // 動作時間 (sec)
		act_param[AP_TIME1] = 2.0; // 動作時間 (sec)
		act_param[AP_TIME2] = 2.0; // 動作時間 (sec)
		act_param[AP_TIME3] = 2.0; // 動作時間 (sec)
		act_param[AP_TIME4] = 2.0; // 動作時間 (sec)
		act_param[AP_TIME5] = 2.0;

		act_num = ACT_GRABFENCE_ML; // 動作中の動きの登録
	}
		state = ActGrabFence_MiddleLeft(m, d, act_param);
		return CALLMATCH;
}

int CallBodyShiftToFence_Front(C1 m[], int next) // 脚先を固定して本体を動かす、平行移動
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallBodyShift(), 脚先を固定して本体を動かす\n");
#endif
	act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
	int bsflag[NUMARMS] = {0};
	
	// 前足で体を引き寄せる。
	bsflag[0] = OFF; bsflag[1] = bsflag[2] = ON;
	bsflag[3] = OFF; bsflag[4] = bsflag[5] = ON;
	act_param[AP_BSHIFTFLAG] = (float)encode_leg(bsflag); // 動かす脚のオンオフの設定

	act_param[AP_BSHIFTX] = 0.1; // シフト距離 x [m]
//	act_param[AP_BSHIFTY] = 0.1; // シフト距離 y [m]
//	act_param[AP_BSHIFTZ] = 0.1; // シフト距離 z [m]
    }
    state = ActBodyShift(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

// ---------------------------------------------------------------------------



int CallLesson1(C1 m[], int next) // 動きの例、関節を指定して動かす
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLesson1(), 関節を指定して個別に回転させる\n");
#endif
	act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
    }
    state = ActLesson1(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallLesson2(C1 m[], int next) // 動きの例、逆運動学で腕を動かす
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLesson2(), 逆運動学で脚を動かす\n");
#endif
	act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
    }
    state = ActLesson2(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallLesson3(C1 m[], int next) // 複数の動きを順番に動かす、常に順番が同じ時はこの書き方が可能
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLesson3(), 複数の動きを順番に動かす\n");
#endif
	act_param[AP_TIMETOTAL] = 10.0; // 合計動作時間 (sec)
	act_param[AP_TIME1] = 2.0; // 動作１の時間 (sec)、合計が一致すべし
	act_param[AP_TIME2] = 3.5; // 動作２の時間 (sec)
	act_param[AP_TIME3] = 2.5; // 動作３の時間 (sec)
	act_param[AP_TIME4] = 2.0; // 動作４の時間 (sec)
    }
    state = ActLesson3(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallLesson4(C1 m[], int next) // 動きの例、外部からの条件により動きを変化させる
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLesson4(), 外部からの条件により動きを変化させる\n");
#endif
	act_param[AP_TIMETOTAL] = 1;
	act_param[AP_P1] = rand() % 14;
    }
    state = ActLesson4(m, act_param);
    if (ACT_END == state) {
	act_param[AP_P1] = rand() % 14; // ここでは乱数により次の実行内容を変えてみる                
	if (0 != (int)act_param[AP_P1]) { // 条件によって実行内容を変化させる例
	    next_a[ACT_LESSON4] = ACT_LESSON4; // これは繰り返し
	}
	else { // これは別の処理に進む
	    next_a[ACT_LESSON4] = ACT_HAND;
	}
    }
    return CALLMATCH; // 起動したことを返り値で戻す
}
int CallNeck(C1 m[], int next) // 首を動かす
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallNeck(), 首を動かす\n");
#endif
	act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	act_param[AP_NECK_HA] =  0.1; // 水平回転、0.0rad, 正面
	act_param[AP_NECK_VA] = -0.6; // 下向 0.6rad, 34度くらい
//      m[A1J1].duration = 1.0;         // おまけ、ついでに脚を動かすことも可能
//      m[A1J1].targetangle.data = 0.0; // 赤い箱を隠さないように左脚を後ろに下げる
    }
    state = ActNeck(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallHatchOpen(C1 m[], int next) // 腹下のハッチを開閉する
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
	act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	act_param[AP_HATCH_LEFT] = M_PI / 2; // ハッチを開く時
	act_param[AP_HATCH_RIGHT] = M_PI / 2 * -1; // ハッチを開く時
    }
    state = ActHatch(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
/*
    fprintf(stderr, "CallHatchOpen(), 腹下のハッチを開閉する\n");
    act_param[AP_HATCH_LEFT]  = M_PI / 2;      // ハッチを開く
    act_param[AP_HATCH_RIGHT] = M_PI / 2 * -1; // ハッチを開く

    act_next = ACT_HATCH;
    int	rcode =	 CallHatch(m, next);
    act_num = ACT_HATCHOPEN;
    return rcode;
*/
}

int CallHatchClose(C1 m[], int next) // 腹下のハッチを開閉する
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
	act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	act_param[AP_HATCH_LEFT] = 0;      // ハッチを閉じる時
	act_param[AP_HATCH_RIGHT] = 0;
    }
    state = ActHatch(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallHatch(C1 m[], int next) // 腹下のハッチを開閉する
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallHatch(), 腹下のハッチを開閉する\n");
#endif
	act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
//	act_param[AP_HATCH_LEFT] = M_PI / 2;      // ハッチを開く時
//	act_param[AP_HATCH_RIGHT] = M_PI / 2 * -1; // ハッチを開く時
//	act_param[AP_HATCH_LEFT] = 0; // ハッチを閉じる時
//	act_param[AP_HATCH_RIGHT] = 0; // ハッチを閉じる時
    }
    state = ActHatch(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallHand(C1 m[], int next) // Hand を開く、閉じる
{
    static int count = 0; // 動作の切り替え用
    
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallHand(), ハンドの開閉の例\n");
#endif
	act_param[AP_TIMETOTAL] = 2; // 2秒
	act_param[AP_HANDSTATE] = HANDOPEN; // 開く
	if (count < 8) { // 開くと閉じるを繰り返す
	    if (0 == count % 4) {
		act_param[AP_HANDNUM] = LF; // 左前脚のハンド
		act_param[AP_HANDRAD] = 1.57; // PI/2, 90度
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
	}
    }
    state = ActHand(m, act_param);
    if (ACT_END == state) {
	if (count < 8) { // 開くと閉じるを繰り返す
	    count++;
	    next_a[ACT_HAND] = ACT_HAND; // Hand を開く、閉じるを繰り返す
	}
	else {
	    count = 0;
	    next_a[ACT_HAND] = ACT_WALKP1; // 次は歩行
	}
    }
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallBodyShift(C1 m[], int next) // 脚先を固定して本体を動かす、平行移動
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallBodyShift(), 脚先を固定して本体を動かす\n");
#endif
	act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
	int bsflag[NUMARMS] = {0};
	bsflag[0] = bsflag[1] = bsflag[2] = ON; // OFF にすると動かさない設定
	bsflag[3] = bsflag[4] = bsflag[5] = ON;
	act_param[AP_BSHIFTFLAG] = (float)encode_leg(bsflag); // 動かす脚のオンオフの設定

	act_param[AP_BSHIFTX] = 0.1; // シフト距離 x [m]
	act_param[AP_BSHIFTY] = 0.1; // シフト距離 y [m]
	act_param[AP_BSHIFTZ] = 0.1; // シフト距離 z [m]
    }
    state = ActBodyShift(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallTilt(C1 m[], int next) // 脚先を固定して本体を動かす、回転
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト  
	fprintf(stderr, "CallTilt(), 脚先を固定して本体を動かす、回転\n");
#endif
	act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
	int bsflag[NUMARMS] = {0};
	bsflag[0] = bsflag[1] = bsflag[2] = ON; // OFF にすると動かさない設定
	bsflag[3] = bsflag[4] = bsflag[5] = ON;
	act_param[AP_BSHIFTFLAG] = (float)encode_leg(bsflag); // 動かす脚のオンオフの設定

	act_param[AP_BSHIFTX] =  0.1; // ピッチ角度 x軸周り [rad]
	act_param[AP_BSHIFTY] =  0.1; // ロール角度 y軸周り [rad]
	act_param[AP_BSHIFTZ] =  0.1; // ヨー角度 z軸周り [rad]
    }
    state = ActTilt(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallRelax(C1 m[], int next) // 動きの例、脱力して関節をすべて０度に
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallRelax(), 脱力して関節をすべて０度に\n");
#endif
//	act_param[AP_TIMETOTAL] = 3.0; // 合計動作時間 (sec)
    }
    state = ActRelax(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallStay(C1 m[], int next) // 動きの例、動きを止めて現状維持
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallStay(), 動きを止めて現状維持\n");
#endif
	act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
    }
    state = ActStay(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallWalkP1(C1 m[], int next) // パラメータ設定の例、精密歩行、前進
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
    fprintf(stderr, "CallWalkP1(), 少し進む\n");
	act_param[AP_WALK_DIR] = FORWARD; // FORWARD:前進, BACKWARD:後退する
	act_param[AP_ITER]  = 1;           // １組の行動の繰り返し回数、１回で約?ｍ進む
	act_param[AP_HOHABAX] = 0.05; // 歩幅(m)、前後の振幅(2倍の距離進む)、0.225以下

	act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	act_param[AP_TIME3] = 0.5;		   // 最後に足を初期位置に戻す
	act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
    }
    return CallWalk(m, next);
}

int CallWalkP2(C1 m[], int next) // 前進
{		
	static int count = 0;
    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
		fprintf(stderr, "CallWalkP2(), 結構進む\n");
		act_param[AP_WALK_DIR] = FORWARD; // FORWARD:前進, BACKWARD:後退する
		act_param[AP_ITER]  = 6;           // １組の行動の繰り返し回数、距離に相当
		act_param[AP_HOHABAX] = 0.05; // 歩幅(m)、前後の振幅(2倍の距離進む)、0.225以下

		act_param[AP_TIME1] = 0.25;         // 動き１の動作時間 (sec)
		act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
		act_param[AP_TIME3] = 0.5;         // 最後に足を初期位置に戻す
		act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
		
		count++;
		if (count == 1) {
			next_a[ACT_WALKP2] = ACT_TURNP2;
			count = 0;
		}
    }
    return CallWalk(m, next);
}

int CallWalkP3(C1 m[], int next) // 精密歩行、前進
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
	act_param[AP_WALK_DIR] = FORWARD; // FORWARD:前進, BACKWARD:後退する
	act_param[AP_ITER]  = 2;           // １組の行動の繰り返し回数、１回で約?ｍ進む
	act_param[AP_HOHABAX] = 0.05; // 歩幅(m)、前後の振幅(2倍の距離進む)、0.225以下

	act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
    }
    return CallWalk(m, next);
}

int CallWalkP4(C1 m[], int next) // 通常歩行、後進
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
	act_param[AP_WALK_DIR] = BACKWARD; // FORWARD:前進, BACKWARD:後退する
	act_param[AP_ITER]  = 2;           // １組の行動の繰り返し回数、１回で約?ｍ進む
	act_param[AP_HOHABAX] = 0.2; // 歩幅(m)、前後の振幅(2倍の距離進む)、0.225以下

	act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
    }
    return CallWalk(m, next);
}

int CallWalk(C1 m[], int next) // 動きの例、通常歩行
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
/*
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallWalk(), 通常歩行\n");
#endif
*/
	act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
	act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
	    act_param[AP_ITER] + act_param[AP_TIME2] + act_param[AP_TIME3]; // 合計動作時間 (sec) ※TIME3追加：白藤
//	act_num = ACT_WALK; // 動作中の動きの登録は CallWalkP1() などに移動
    }
// act_param[AP_?????] への代入はここに書いても有効
    state = ActWalk(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallLegsP1(C1 m[], int next) // 
{
    static float t[NUMARMS][MAXPHASE][XYZ] = {0}; // target xyz
    static int   f[NUMARMS] = {0};

    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
	act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
	act_param[AP_ITER]  = 3;           // １組の行動の繰り返し回数

	act_param[AP_TIME1] = 1.0;         // 動き１の動作時間 (sec)
	act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
	act_num = ACT_LEGSP1;
//	for (int i = 0; i < act_param[AP_PHASE]; i++) {}

	f[LF-1] = ON; // 動かしたい脚を ON にする、次行から AP_PHASE 分、座標を用意
	t[LF-1][0][X] = 0.65; t[LF-1][0][Y] = 0.15; t[LF-1][0][Z] =  0.4;
	t[LF-1][1][X] = 0.65; t[LF-1][1][Y] = 0.25; t[LF-1][1][Z] =  0.4;
	t[LF-1][2][X] = 0.65; t[LF-1][2][Y] = 0.25; t[LF-1][2][Z] =  0.1;
	t[LF-1][3][X] = 0.5;  t[LF-1][3][Y] = 0.5;  t[LF-1][3][Z] = -0.3;

	f[RM-1] = ON; // 2本以上も動かせる
	t[RM-1][0][X] = 0.2; t[RM-1][0][Y] = -0.5; t[RM-1][0][Z] =  0.3;
	t[RM-1][1][X] = 0.2; t[RM-1][1][Y] = -0.5; t[RM-1][1][Z] =  0.4;
	t[RM-1][2][X] = 0.2; t[RM-1][2][Y] = -0.5; t[RM-1][2][Z] =  0.5;
	t[RM-1][3][X] = 0.2; t[RM-1][3][Y] = -0.5; t[RM-1][3][Z] = -0.3;
    }
    return CallLegs(m, next, t, f);
}

int CallLegs(C1 m[], int next, float target[NUMARMS][MAXPHASE][XYZ], int onflag[NUMARMS])
{
//  if (ACT_LEGS != act_next) return CALLTHRU; // Call????P1() の事前の実行が必要

    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLegs(), 任意の脚の動き\n");
#endif
	act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
	    act_param[AP_ITER]; // 合計動作時間 (sec)
    }
// act_param[AP_?????] への代入はここに書いても有効
    state = ActLegs(m, act_param, target, onflag);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallTurnP1(C1 m[], int next) // 旋回、時計回り
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
	act_param[AP_TURN_DIR] = TURNCW;   // 時計周り、or TURNCCW は反時計周り
	act_param[AP_ITER]  = 1;           // 回転の繰り返し回数
	act_param[AP_TURNDEG] = 30;         // 旋回1回の角度(deg)

	act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
    }
    return CallTurn(m, next);
}

int CallTurnP2(C1 m[], int next) // 旋回、反時計回り
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
	act_param[AP_TURN_DIR] = TURNCCW;   // 時計周り、or TURNCCW は反時計周り
	act_param[AP_ITER]  = 1;           // 回転の繰り返し回数 
	act_param[AP_TURNDEG] = 10;         // 旋回1回の角度(deg)

	act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
    }
    return CallTurn(m, next);
}

int CallTurnP3(C1 m[], int next) // 精密旋回、時計回り
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
	act_param[AP_TURN_DIR] = TURNCW;   // 時計周り、or TURNCCW は反時計周り
	act_param[AP_ITER]  = 2;           // １組の行動の繰り返し回数、１回で 回転
	act_param[AP_TURNDEG] = 5;         // 旋回1回の角度(deg)

	act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
    }
    return CallTurn(m, next);
}

int CallTurnP4(C1 m[], int next) // 精密旋回、反時計回り
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 動作開始時のみ設定
	act_param[AP_TURN_DIR] = TURNCCW;   // 時計周り、or TURNCCW は反時計周り
	act_param[AP_ITER]  = 2;           // １組の行動の繰り返し回数、１回で 回転
	act_param[AP_TURNDEG] = 5;         // 旋回1回の角度(deg)

	act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	act_param[AP_TIME2] = 0.5;         // 浮いている脚の最後の着地の動作時間 (sec)
	act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
    }
    return CallTurn(m, next);
}

int CallTurn(C1 m[], int next) // 動きの例、その場で転回、旋回する
{
//  if (ACT_WALK != act_next) return CALLTHRU; // Call????P1() の事前の実行が必要

    if (ACT_START == (int)act_param[AP_STATE]) { // 
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallTurn(), その場で転回、旋回する\n");
#endif
	act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
	act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
	    act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
//      act_num = ACT_TURN; // 動作中の動きの登録は CallTurnP1() などに移動                         
    }
//  act_param[AP_?????] への代入はここに書いても有効 
    state = ActTurn(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallSetRobotPos(float xyzrpy[6], int next) //  // ロボットの座標や向きを設定する
{
    fprintf(stderr, "CallSetRobotPos(), ロボットの位置や向きを設定する\n");

//  float xyzrpy[6] = {0.0, 0.0, 0.3, 0.0, 0.0, 0.0}; // X,Y,Z座標、ロール、ピッチ、ヨー degree
// その後引数に変更した
    gRobot -> setrobotpos(xyzrpy); // ロボットに設定
    
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallGetRobotPos(float xyzrpy[6], int next) // ロボットの座標や向きを読み出す
{
    fprintf(stderr, "CallGetRobotPos(), ロボットの位置や向きを読み出す\n");

//  float xyzrpy[6] = {0.0}; // この配列に読み込む、その後引数に変更した
    gRobot -> getrobotpos(xyzrpy); // 配列に読み込む
    // 表示してみる。X,Y,Z座標、ロール、ピッチ、ヨー degree
    ROS_INFO("Robot Position: %.3f %.3f %.3f %.3f %.3f %.3f",
	     xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
    
    return CALLMATCH; // 起動したことを返り値で戻す
}




//------------------------------------※追加--------------------------------------------

int CallNVCFindH(C1 m[], RGBCam *c, DepthCam *d, int next) // 色や輪郭からハリネズミを見つける
{
    fprintf(stderr, "CallNVCFindH(), 画像処理\n");

    c -> cvmary[CVMIN] = c -> cv_ptr -> image; // 今後はカメラオブジェに画像バッファ(配列)を用意
    cv::Mat *inp = &(c -> cv_ptr -> image); // 右辺が長いので簡潔に書けるように左辺を使う

// 色による同色の領域検出、物体検出
    cv::Mat output(inp -> rows, inp -> cols, CV_8UC1);
//  cv::Mat output(*(inp). rows, *(inp). cols, CV_8UC1); // これは g++ が通らない
    int dummy = imFindColorObj(inp, &output, RED); // 指定色の物体を見つける

// 輪郭検出、Laplacian or Sobel どちらか
    cv::Mat output3(inp -> rows, inp -> cols, CV_8UC1);
    dummy = imLaplacian(&output, &output3, DEPTH); // Laplacian
//  dummy = imSobel    (&output, &output3, DEPTH); // Sobel
    output = output3;
    cv::imshow("LaplacianColor", output); cv::waitKey(5); // 表示してみる、見なくてよければなし
//  cv::imshow("SobelColor", output); cv::waitKey(5);

// Hough 変換、直線検出
    cv::Mat output2;
    int p[3] = {70, 20, 10}; // Hough 変換のパラメータ、最低投票数、最小線分長、最大許容間隔
    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換、膨張と収縮をする前
    cv::imshow("HoughBefore", output2); cv::waitKey(5); // 表示してみる

// 膨張と収縮、してもしなくても可、良い方を選ぶ
    cv::dilate(output, output, cv::noArray(), cv::Point(-1, -1), 3); // 膨張 2 回
    cv::erode (output, output, cv::noArray(), cv::Point(-1, -1), 3); // 収縮 2 回

    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換、膨張と収縮をした後
    cv::imshow("HoughAfter", output2); cv::waitKey(5);	// 表示してみる

// 重心と外接長方形を求める関数, 返り値は領域の個数
    int label[ROWS][COLS] = {{0}};      // ラベリングの結果を画素毎に記録する配列
    int objs = imLabel(&output, label); // ラベリングにより物体を分離する処理
    c -> fareanum = imCogCorners(&output, objs, label, c -> fareacog, c -> fareacorners); 

// 重心と外接長方形、結果の表示が必要なら
    cv::Mat tmpr;
    imShowCog    (output, &tmpr, "imColorObjsCOG", c -> fareacog, c -> fareanum); // 表示してみる
    imShowCorners(output, &tmpr, "imColorObjsBB" ,  c -> fareacorners, c -> fareanum); // 表示

// 検出した領域、物体の 3 次元ローカル座標
    float x, y, z;            // 計算後のローカル座標を記憶する変数               
// ※コメントアウト解除：白藤                       
    float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要                
	float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要

    fprintf(stderr, "3次元ローカル座標 %d個\n", c -> fareanum);
    
    for (int i = 0; i < c -> fareanum; i++) {
//  for (int i = 0; i < objs; i++) {
// 次の処理で画像中の座標から3次元ローカル座標を求める
		d -> LocalXyz(c -> fareacog[i][X], c -> fareacog[i][Y], &x, &y, &z, ha, va);
// 検出した個々の物体の3次元ローカル座標を配列に記憶する
		d -> fobjxyz[i][0] = x; // c -> fobjxyz[][] を用意したほうがよいかも
		d -> fobjxyz[i][1] = y; // depthclass.h 中の宣言で float fobjxyz[MAXOBJS][XYZ];
		d -> fobjxyz[i][2] = z; // となっていることを要確認、もし最後が [XY] なら要修正
		fprintf(stderr, "%d(%.2f %.2f %.2f), \n", i+1, x, y, z);
	}
	
	static int count = 0;
	static int countNV = 0;
	if (0 < c -> fareanum) { // 1個以上見つけたら。c,d どちらかよく考える
	    for (int i = 0; i < (int)(d -> fobjnum); i++) {
	        fprintf(stderr, "物体%dの座標 x:%.3f y:%.3f z:%.3f\n", d -> fobjnum, 
			d -> fobjxyz[(int)(d -> fobjnum) -1][0],
			d -> fobjxyz[(int)(d -> fobjnum) -1][1],
			d -> fobjxyz[(int)(d -> fobjnum) -1][2]);
	    }
	    act_param[AP_GRABX] = d -> fobjxyz[0][0];
	    act_param[AP_GRABY] = d -> fobjxyz[0][1];
	    act_param[AP_GRABZ] = d -> fobjxyz[0][2];
	    // ハリネズミを見つけたらその座標に首を動かす
		if(0.75> x) {
			next_a[ACT_NVCFH] = ACT_NECKFIND;
			// 視界の右端にある場合：時計回り
			if (-0.30 > y) {
				next_a[ACT_NVCFH] = ACT_TURNP1;
			}
			// 視界の左端にある場合：半時計回り
			else if (0.30 < y) {
				next_a[ACT_NVCFH] = ACT_TURNP2;
			}
			// ハリネズミを視界の正面に入れたら掴む
            else if (0.75 > x && -0.30 < y && y < 0.30) {
				next_a[ACT_NVCFH] = ACT_GRABRELEASE;
			}
		}
		// ハリネズミが遠い場合
		else {
			// ちょい歩く
			if (0.75 <= x && x <= 1.5 && -0.35 < y && y < 0.35) {
					next_a[ACT_NVCFH] = ACT_WALKP1;
			}
			// 結構歩く
			else if (1.5 < x && -0.6 < y && y < 0.6) {
				next_a[ACT_NVCFH] = ACT_WALKP2;
			}
			// 視界の右端にある場合：時計回り
			else if (-0.60 > y) {
				next_a[ACT_NVCFH] = ACT_TURNP1;
			}
			// 視界の左端にある場合：半時計回り
			else if (0.60 < y) {
				next_a[ACT_NVCFH] = ACT_TURNP2;
			}
			count = 0;
		}
	}
	// 見つからなかった場合　首を左右正面に動かし探すフェーズ
	else {
		countNV = 0;
	    if(count == 0){ // 首を左に動かした後右に動かす
			next_a[ACT_NVCFH] = ACT_NECKRIGHT;
			count = 1;
		}
		else if(count == 1){
			next_a[ACT_NVCFH] = ACT_NECKLEFT;
			count = 2;
		}
		else if(count == 2){
			next_a[ACT_NVCFH] = ACT_TURNP1;
			count = 0;
		}
	}

    
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallNVCFindH_Left(RGBCam *c, DepthCam *d, int next) // 画像から色や輪郭を用いて物体検出
{
    fprintf(stderr, "CallNVFindObjsColor(), カメラ画像を処理する\n");

    c -> cvmary[CVMIN] = c -> cv_ptr -> image; // 今後はカメラオブジェに画像バッファ(配列)を用意
    cv::Mat *inp = &(c -> cv_ptr -> image); // 右辺が長いので簡潔に書けるように左辺を使う

// 色による同色の領域検出、物体検出
    cv::Mat output(inp -> rows, inp -> cols, CV_8UC1);
//  cv::Mat output(*(inp). rows, *(inp). cols, CV_8UC1); // これは g++ が通らない
    int dummy = imFindColorObj(inp, &output, RED); // 指定色の物体を見つける

// 輪郭検出、Laplacian or Sobel どちらか
    cv::Mat output3(inp -> rows, inp -> cols, CV_8UC1);
    dummy = imLaplacian(&output, &output3, DEPTH); // Laplacian
//  dummy = imSobel    (&output, &output3, DEPTH); // Sobel
    output = output3;
    cv::imshow("LaplacianColor", output); cv::waitKey(5); // 表示してみる、見なくてよければなし
//  cv::imshow("SobelColor", output); cv::waitKey(5);

// Hough 変換、直線検出
    cv::Mat output2;
    int p[3] = {70, 20, 10}; // Hough 変換のパラメータ、最低投票数、最小線分長、最大許容間隔
    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換、膨張と収縮をする前
    cv::imshow("HoughBefore", output2); cv::waitKey(5); // 表示してみる


// 膨張と収縮、してもしなくても可、良い方を選ぶ
    cv::dilate(output, output, cv::noArray(), cv::Point(-1, -1), 3); // 膨張 2 回
    cv::erode (output, output, cv::noArray(), cv::Point(-1, -1), 3); // 収縮 2 回

    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換、膨張と収縮をした後
    cv::imshow("HoughAfter", output2); cv::waitKey(5);	// 表示してみる

// 重心と外接長方形を求める関数, 返り値は領域の個数
    int label[ROWS][COLS] = {{0}};      // ラベリングの結果を画素毎に記録する配列
    int objs = imLabel(&output, label); // ラベリングにより物体を分離する処理
    c -> fareanum = imCogCorners(&output, objs, label, c -> fareacog, c -> fareacorners); 

// 重心と外接長方形、結果の表示が必要なら
    cv::Mat tmpr;
    imShowCog    (output, &tmpr, "imColorObjsCOG", c -> fareacog, c -> fareanum); // 表示してみる
    imShowCorners(output, &tmpr, "imColorObjsBB" ,  c -> fareacorners, c -> fareanum); // 表示

// 検出した領域、物体の 3 次元ローカル座標
    float x, y, z;            // 計算後のローカル座標を記憶する変数                                     
//  float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要                
//	float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要

    fprintf(stderr, "3次元ローカル座標 %d個\n", c -> fareanum);
    for (int i = 0; i < c -> fareanum; i++) {
//  for (int i = 0; i < objs; i++) {
// 次の処理で画像中の座標から3次元ローカル座標を求める
	d -> LocalXyz(c -> fareacog[i][X], c -> fareacog[i][Y], &x, &y, &z, d -> ha, d -> va);
// 検出した個々の物体の3次元ローカル座標を配列に記憶する
	d -> fobjxyz[i][0] = x; // c -> fobjxyz[][] を用意したほうがよいかも
	d -> fobjxyz[i][1] = y; // depthclass.h 中の宣言で float fobjxyz[MAXOBJS][XYZ];
	d -> fobjxyz[i][2] = z; // となっていることを要確認、もし最後が [XY] なら要修正
	fprintf(stderr, "%d(%.2f %.2f %.2f), ", i+1, x, y, z);
    }
    
    if (0 < c -> fareanum) { // 1個以上見つけたら。c,d どちらかよく考える
	    for (int i = 0; i < (int)(d -> fobjnum); i++) {
	        fprintf(stderr, "物体%dの座標 x:%.3f y:%.3f z:%.3f\n", d -> fobjnum, 
			d -> fobjxyz[(int)(d -> fobjnum) -1][0],
			d -> fobjxyz[(int)(d -> fobjnum) -1][1],
			d -> fobjxyz[(int)(d -> fobjnum) -1][2]);
	    }
	    act_param[AP_GRABX] = d -> fobjxyz[0][0];
	    act_param[AP_GRABY] = d -> fobjxyz[0][1];
	    act_param[AP_GRABZ] = d -> fobjxyz[0][2];
	    
	   next_a[ACT_NVCFH_L] = ACT_GRL;
	}
	
	else {
		next_a[ACT_NVCFH_L] = ACT_NVCFH_L;
	}
    
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallNVFindFenceDepth(RGBCam *c, DepthCam *d, int next) // 画像から色や輪郭を用いて物体検出
{
    fprintf(stderr, "CallNVFindFenceDepth(), 深度カメラでフェンスを探す\n");

//    act_num = ACT_NVFOC; // 動作中の動きの登録

    d -> cvmary[CVMIN] = d -> cv_ptr -> image; // 今後はカメラオブジェに画像バッファ(配列)を用意
    cv::Mat *inp = &(d -> cv_ptr -> image); // 右辺が長いので簡潔に書けるように左辺を使う

    cv::Mat output(inp -> rows, inp -> cols, CV_8UC1);
//  cv::Mat output(*(inp). rows, *(inp). cols, CV_8UC1); // これは通らない
    int dummy;
// 深度による同距離の領域検出、今は不要、もし必要なら実施
//  int dummy = imFindColorObj(inp, &output, ???); // 距離が同じ領域を見つける
    
// 輪郭検出、Laplacian or Sobel どちらか、深度画像は通常ここから始める
    cv::Mat output3(inp -> rows, inp -> cols, CV_8UC1);
    dummy = imLaplacian(inp, &output3, DEPTH); // Laplacian
//  dummy = imSobel    (&output, &output3, DEPTH); // Sobel
    output = output3;
    cv::imshow("LaplacianDepth", output); cv::waitKey(5); // 表示してみる、見なくてよければなし
//  cv::imshow("SobelDepth", output); cv::waitKey(5);

// Hough 変換、直線検出
    cv::Mat output2;
    int p[3] = {70, 20, 10}; // Hough 変換のパラメータ、最低投票数、最小線分長、最大許容間隔
//  std::vector<cv::Vec4i> lines; // lines は RGBカメラ、深度カメラのクラスに移動
//  dummy = imHough(&output, &output2, &lines, p); // Hough 変換、ローカル変数の lines        
    dummy = imHough(&output, &output2, &(d ->lines), p); // Hough 変換、膨張と収縮をする前
    cv::imshow("HoughDBefore", output2); cv::waitKey(5); // 表示してみる

// 膨張と収縮、してもしなくてもどちらでも可   
    cv::dilate(output, output, cv::noArray(), cv::Point(-1, -1), 3); // 膨張 2 回
    cv::erode (output, output, cv::noArray(), cv::Point(-1, -1), 3); // 収縮 2 回

    dummy = imHough(&output, &output2, &(d -> lines), p); // Hough 変換、膨張と収縮をした後
    cv::imshow("HoughDAfter", output2); cv::waitKey(5); // 表示してみる

// 重心と外接長方形を求める関数, 返り値は領域の個数
    int label[ROWS][COLS] = {{0}};      // ラベリングの結果を画素毎に記録する配列
    int objs = imLabel(&output, label); // ラベリングにより物体を分離する処理
    d -> fareanum = imCogCorners(&output, objs, label, d -> fareacog, d -> fareacorners); 

// 重心と外接長方形、結果の表示が必要なら
    cv::Mat tmpr;
    imShowCog    (output, &tmpr, "imFindObjsDepthCOG", d -> fareacog, d -> fareanum);
    imShowCorners(output, &tmpr, "imFindObjsDepthBB" , d -> fareacorners, d -> fareanum);

// 検出した領域、物体の 3 次元ローカル座標
    float x, y, z;            // 計算後のローカル座標を記憶する変数
//  float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要
//  float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要

    fprintf(stderr, "3次元ローカル座標 %d個\n", d -> fareanum);
    
//    bool found = false;
 //   int fx, fy, fz;
 	static int count = 0;
	if (0 < d -> fareanum) { // 1個以上見つけたら。
		for (int i = 0; i < d -> fareanum; i++) {
			//  for (int i = 0; i < objs; i++) {
			// 次の処理で画像中の座標から3次元ローカル座標を求める
			d -> LocalXyz(d -> fareacog[i][X], d -> fareacog[i][Y], &x, &y, &z, d -> ha, d -> va);
			// 検出した個々の物体の3次元ローカル座標を配列に記憶する
			d -> fobjxyz[i][X] = x;
			d -> fobjxyz[i][Y] = y; // depthclass.h 中の宣言で float fobjxyz[MAXOBJS][XYZ];
			d -> fobjxyz[i][Z] = z; // となっていることを要確認、もし最後が [XY] なら要修正
			fprintf(stderr, "%d(%.2f %.2f %.2f), \n", i+1, x, y, z);
							
			int x1 = d -> fareacorners[i][0][0]; // 左上
			int y1 = d -> fareacorners[i][0][1];
			int x2 = d -> fareacorners[i][1][0]; // 右下
			int y2 = d -> fareacorners[i][1][1];

			int width = std::abs(x1 - x2);
			int height = std::abs(y1 - y2);
			 	
			act_param[AP_GRABX] = x;
			act_param[AP_GRABY] = y;
			act_param[AP_GRABZ] = z;
			
			if (0 <= x && x < 3.0) {
				fprintf(stderr, "スタート\n");
			 	
			 	// 柵に向かって歩き始める
			 	if (width > (height * 2.5) && width > 50 && 2.0 < x && x < 3.0) {
					// 見つけた柵の大きさを表示
					fprintf(stderr, "柵見つけた 幅：%d 高さ：%d\n", width, height);
					fprintf(stderr, "(%.2f %.2f %.2f), \n", x, y, z);
					next_a[ACT_NVFFD] = ACT_WALKP2;
					break;
				}
				
				// 四足歩行開始
				else if (0.6 < x && x < 1.3) {
					fprintf(stderr, "柵近い (%.2f %.2f %.2f), \n", x, y, z);
					
					next_a[ACT_NVFFD] = ACT_GRABUF;
					break;
				}
				
				// 前足で柵を掴む
				else if (0.1 < x && x < 0.6 && count == 0) {
					fprintf(stderr, "柵掴む (%.2f %.2f %.2f), \n", x, y, z);
					act_param[AP_GRABX] = x;
					act_param[AP_GRABY] = y;
					act_param[AP_GRABZ] = z;
					
					next_a[ACT_NVFFD] = ACT_GRABFENCE_F;
					count = 1;
					break;
				}
				// 左中足で柵を掴む
				else if(count == 1 && 0 < x && x < 0.5) {
					fprintf(stderr, "掴む柵みっけ (%.2f %.2f %.2f), \n", x, y, z);
					act_param[AP_GRABX] = x;
					act_param[AP_GRABY] = y;
					act_param[AP_GRABZ] = z;
					
					next_a[ACT_NVFFD] = ACT_GRABFENCE_ML;
					count = 2;
					break;
				}
			}
		}
	}
	else {
		state = ACT_MOVING;
	}
    return CALLMATCH; // 起動したことを返り値で戻す
}

//-------------------------------------※追加----------------------------------------------




int CallNVisionColor(C1 m[], RGBCam *c, DepthCam *d, int next) // 画像から色や輪郭を用いて物体検出
{
    fprintf(stderr, "CallNVFindObjsColor(), カメラ画像を処理する\n");

    c -> cvmary[CVMIN] = c -> cv_ptr -> image; // 今後はカメラオブジェに画像バッファ(配列)を用意
    cv::Mat *inp = &(c -> cv_ptr -> image); // 右辺が長いので簡潔に書けるように左辺を使う

// 色による同色の領域検出、物体検出
    cv::Mat output(inp -> rows, inp -> cols, CV_8UC1);
//  cv::Mat output(*(inp). rows, *(inp). cols, CV_8UC1); // これは g++ が通らない
    int dummy = imFindColorObj(inp, &output, RED); // 指定色の物体を見つける

// 輪郭検出、Laplacian or Sobel どちらか
    cv::Mat output3(inp -> rows, inp -> cols, CV_8UC1);
    dummy = imLaplacian(&output, &output3, DEPTH); // Laplacian
//  dummy = imSobel    (&output, &output3, DEPTH); // Sobel
    output = output3;
    cv::imshow("LaplacianColor", output); cv::waitKey(5); // 表示してみる、見なくてよければなし
//  cv::imshow("SobelColor", output); cv::waitKey(5);

// Hough 変換、直線検出
    cv::Mat output2;
    int p[3] = {70, 20, 10}; // Hough 変換のパラメータ、最低投票数、最小線分長、最大許容間隔
    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換、膨張と収縮をする前
    cv::imshow("HoughBefore", output2); cv::waitKey(5); // 表示してみる
/*
    p[0] = 80; P[1] = 30; P[2] = 10; // パラメータを変えた時の比較
    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換
    cv::imshow("Hough803010", output2); cv::waitKey(5); // 表示して見比べてみる

    p[0] = 80; P[1] = 20; P[2] = 10; // パラメータを変えた時の比較
    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換
    cv::imshow("Hough802010", output2); cv::waitKey(5); // 表示して見比べてみる

    p[0] = 70; P[1] = 30; P[2] = 10; // パラメータを変えた時の比較
    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換
    cv::imshow("Hough703010", output2); cv::waitKey(5); // 表示して見比べてみる
*/

//  std::vector<cv::Vec4i> lines; // lines は RGBカメラ、深度カメラのクラスに移動した
//  dummy = imHough(&output, &output2, &lines, p); // Hough 変換、ローカル変数の lines の書法

// 膨張と収縮、してもしなくても可、良い方を選ぶ
    cv::dilate(output, output, cv::noArray(), cv::Point(-1, -1), 3); // 膨張 2 回
    cv::erode (output, output, cv::noArray(), cv::Point(-1, -1), 3); // 収縮 2 回

    dummy = imHough(&output, &output2, &(c -> lines), p); // Hough 変換、膨張と収縮をした後
    cv::imshow("HoughAfter", output2); cv::waitKey(5);	// 表示してみる

// 重心と外接長方形を求める関数, 返り値は領域の個数
    int label[ROWS][COLS] = {{0}};      // ラベリングの結果を画素毎に記録する配列
    int objs = imLabel(&output, label); // ラベリングにより物体を分離する処理
    c -> fareanum = imCogCorners(&output, objs, label, c -> fareacog, c -> fareacorners); 

// 重心と外接長方形、結果の表示が必要なら
    cv::Mat tmpr;
    imShowCog    (output, &tmpr, "imColorObjsCOG", c -> fareacog, c -> fareanum); // 表示してみる
    imShowCorners(output, &tmpr, "imColorObjsBB" ,  c -> fareacorners, c -> fareanum); // 表示

// 検出した領域、物体の 3 次元ローカル座標
    float x, y, z;            // 計算後のローカル座標を記憶する変数               
// ※コメントアウト解除：白藤                       
    float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要                
	float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要

    fprintf(stderr, "3次元ローカル座標 %d個\n", c -> fareanum);
    for (int i = 0; i < c -> fareanum; i++) {
//  for (int i = 0; i < objs; i++) {
// 次の処理で画像中の座標から3次元ローカル座標を求める
	d -> LocalXyz(c -> fareacog[i][X], c -> fareacog[i][Y], &x, &y, &z, ha, va);
// 検出した個々の物体の3次元ローカル座標を配列に記憶する
	d -> fobjxyz[i][0] = x; // c -> fobjxyz[][] を用意したほうがよいかも
	d -> fobjxyz[i][1] = y; // depthclass.h 中の宣言で float fobjxyz[MAXOBJS][XYZ];
	d -> fobjxyz[i][2] = z; // となっていることを要確認、もし最後が [XY] なら要修正
	fprintf(stderr, "%d(%.2f %.2f %.2f), ", i+1, x, y, z);
    }
    
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallNVisionDepth(RGBCam *c, DepthCam *d, int next) // 画像から色や輪郭を用いて物体検出
{
    fprintf(stderr, "CallNVFindObjsDepth(), カメラ画像を処理する\n");

    act_num = ACT_NVFOC; // 動作中の動きの登録

    d -> cvmary[CVMIN] = d -> cv_ptr -> image; // 今後はカメラオブジェに画像バッファ(配列)を用意
    cv::Mat *inp = &(d -> cv_ptr -> image); // 右辺が長いので簡潔に書けるように左辺を使う

    cv::Mat output(inp -> rows, inp -> cols, CV_8UC1);
//  cv::Mat output(*(inp). rows, *(inp). cols, CV_8UC1); // これは通らない
    int dummy;
// 深度による同距離の領域検出、今は不要、もし必要なら実施
//  int dummy = imFindColorObj(inp, &output, ???); // 距離が同じ領域を見つける
    
// 輪郭検出、Laplacian or Sobel どちらか、深度画像は通常ここから始める
    cv::Mat output3(inp -> rows, inp -> cols, CV_8UC1);
    dummy = imLaplacian(inp, &output3, DEPTH); // Laplacian
//  dummy = imSobel    (&output, &output3, DEPTH); // Sobel
    output = output3;
    cv::imshow("LaplacianDepth", output); cv::waitKey(5); // 表示してみる、見なくてよければなし
//  cv::imshow("SobelDepth", output); cv::waitKey(5);

// Hough 変換、直線検出
    cv::Mat output2;
    int p[3] = {70, 20, 10}; // Hough 変換のパラメータ、最低投票数、最小線分長、最大許容間隔
//  std::vector<cv::Vec4i> lines; // lines は RGBカメラ、深度カメラのクラスに移動
//  dummy = imHough(&output, &output2, &lines, p); // Hough 変換、ローカル変数の lines        
    dummy = imHough(&output, &output2, &(d ->lines), p); // Hough 変換、膨張と収縮をする前
    cv::imshow("HoughDBefore", output2); cv::waitKey(5); // 表示してみる

// 膨張と収縮、してもしなくてもどちらでも可   
    cv::dilate(output, output, cv::noArray(), cv::Point(-1, -1), 3); // 膨張 2 回
    cv::erode (output, output, cv::noArray(), cv::Point(-1, -1), 3); // 収縮 2 回

    dummy = imHough(&output, &output2, &(d -> lines), p); // Hough 変換、膨張と収縮をした後
    cv::imshow("HoughDAfter", output2); cv::waitKey(5); // 表示してみる

// 重心と外接長方形を求める関数, 返り値は領域の個数
    int label[ROWS][COLS] = {{0}};      // ラベリングの結果を画素毎に記録する配列
    int objs = imLabel(&output, label); // ラベリングにより物体を分離する処理
    d -> fareanum = imCogCorners(&output, objs, label, d -> fareacog, d -> fareacorners); 

// 重心と外接長方形、結果の表示が必要なら
    cv::Mat tmpr;
    imShowCog    (output, &tmpr, "imFindObjsDepthCOG", d -> fareacog, d -> fareanum);
    imShowCorners(output, &tmpr, "imFindObjsDepthBB" , d -> fareacorners, d -> fareanum);

// 検出した領域、物体の 3 次元ローカル座標
    float x, y, z;            // 計算後のローカル座標を記憶する変数
//  float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要
//  float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要

    fprintf(stderr, "3次元ローカル座標 %d個\n", d -> fareanum);
    for (int i = 0; i < d -> fareanum; i++) {
//  for (int i = 0; i < objs; i++) {
// 次の処理で画像中の座標から3次元ローカル座標を求める
	d -> LocalXyz(d -> fareacog[i][X], d -> fareacog[i][Y], &x, &y, &z, d -> ha, d -> va);
// 検出した個々の物体の3次元ローカル座標を配列に記憶する
	d -> fobjxyz[i][X] = x;
	d -> fobjxyz[i][Y] = y; // depthclass.h 中の宣言で float fobjxyz[MAXOBJS][XYZ];
	d -> fobjxyz[i][Z] = z; // となっていることを要確認、もし最後が [XY] なら要修正
	fprintf(stderr, "%d(%.2f %.2f %.2f), ", i+1, x, y, z);
    }
    
    return CALLMATCH; // 起動したことを返り値で戻す
}



int CallNVisionDepthOld(RGBCam *cc, DepthCam *d, int next) // 画像から色や輪郭を用いて物体検出
{
    fprintf(stderr, "CallNVFindObjsDepth(), カメラ画像を処理する\n");

    d -> cvmary[CVMIN] = d -> cv_ptr -> image; // 今後はカメラオブジェクトに配列を用意

    cv::Mat output(d -> cvmary[CVMIN]. rows, d -> cvmary[CVMIN]. cols, CV_8UC1);

    int dummy = imLaplacian(&(d -> cvmary[CVMIN]), &output, DEPTH); // Laplacian

    cv::Mat output2;
    std::vector<cv::Vec4i> lines;
    int p[3] = {70, 20, 10}; // Hough 変換のパラメータ、最低投票数、最小線分長、最大許容間隔
    dummy = imHough(&output, &output2, &lines, p); // Hough 変換、膨張と収縮をする前

    cv::dilate(output, output, cv::noArray(), cv::Point(-1, -1), 3); // 膨張 2 回
    cv::erode (output, output, cv::noArray(), cv::Point(-1, -1), 3); // 収縮 2 回

    dummy = imHough(&output, &output2, &lines, p); // Hough 変換、膨張と収縮をした後

// ラベリング
    int label[ROWS][COLS] = {{0}};      // ラベリングの結果を画素毎に記録する配列
    int objs = imLabel(&output, label); // ラベリングにより物体を分離する処理

// 重心と外接長方形を求める関数, 返り値は領域の個数
    d -> fareanum = imCogCorners(&output, objs, label, d -> fareacog, d -> fareacorners); 

// 結果の表示が必要なら
    cv::Mat tmpr;
    imShowCog    (output, &tmpr, "imFindObjsDepthCOG", d -> fareacog, d -> fareanum);
    imShowCorners(output, &tmpr, "imFindObjsDepthBB" , d -> fareacorners, d -> fareanum);

    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallNVisionColorOld(RGBCam *c, DepthCam *d, int next) // 画像から色や輪郭を用いて物体検出
{
    fprintf(stderr, "CallNVFindObjsColor(), カメラ画像を処理する\n");

//  cv::Mat in = c -> cv_ptr -> image; // 従来はローカル変数
    c -> cvmary[CVMIN] = c -> cv_ptr -> image; // 今後はカメラオブジェクトに配列を用意
    int filter = FOC; // 指定した色の物体を検出
    int color = RED;  // 赤色
//  int objs = imFindObjs(&in, filter, color, SHOW, "imFindObjsColor", // 従来
    int objs = imFindObjs(&(c -> cvmary[CVMIN]), filter, color, NOSHOW, "imFindObjsColor",
		      c -> fareacog, c -> fareacorners);
    c -> fareanum = objs; // 検出した領域の個数

    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallNVisionDepthOldOld(RGBCam *c, DepthCam *d, int next) // 画像から色や輪郭を用いて物体検出
{
    fprintf(stderr, "CallNVFindObjsDepth(), カメラ画像を処理する\n");

//  cv::Mat in = d -> cv_ptr -> image; // ローカル変数
    d -> cvmary[CVMIN] = d -> cv_ptr -> image; // 今後はカメラオブジェクトに配列を用意
    int filter = LAPLACIAN; // ラプラシアンフィルタ
    int type = DEPTH;       // 距離画像
//  int objs = imFindObjs(&in, filter, type, SHOW, "imFindObjsDepth",
    int objs = imFindObjs(&(d -> cvmary[CVMIN]), filter, type, SHOW, "imFindObjsDepth",
		      d -> fareacog, d -> fareacorners);
    d -> fareanum = objs; // 検出した領域の個数

    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallVisionFront(C1 m[], RGBCam *c, DepthCam *d, int next) // Front カメラ画像から物体検出
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallVisionFront(), Front カメラ画像から物体を検出する\n");
#endif
	act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	act_param[AP_FILTER] = FOC; act_param[AP_IMGORCOL] = RED; // 赤色検出
//	act_param[AP_FILTER] = LAPLACIAN; act_param[AP_IMGORCOL] = DEPTH; // 輪郭、深度、LAPLACIAN
    }
    state = ActVision(m, &(c[IDDEPTHCAMCOLOR]), &(d[IDDEPTHCAMDEPTH]), act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallVisionLeft(C1 m[], RGBCam *c, DepthCam *d, int next) // Left カメラ画像から物体検出
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallVisionLeft(), Left カメラ画像から物体を検出する\n");
#endif
	act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	act_param[AP_FILTER] = FOC; act_param[AP_IMGORCOL] = RED; // 赤色検出
//	act_param[AP_FILTER] = LAPLACIAN; act_param[AP_IMGORCOL] = DEPTH; // 輪郭、深度、LAPLACIAN
    }
    state = ActVision(m, &(c[IDDEPTHCAMCOLORSL]), &(d[IDDEPTHCAMDEPTHSL]), act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallVisionRight(C1 m[], RGBCam *c, DepthCam *d, int next) // Right カメラ画像から物体検出
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallVisionRight(), Right カメラ画像から物体を検出する\n");
#endif
	act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	act_param[AP_FILTER] = FOC; act_param[AP_IMGORCOL] = RED; // 赤色検出
//	act_param[AP_FILTER] = LAPLACIAN; act_param[AP_IMGORCOL] = DEPTH; // 輪郭、深度、LAPLACIAN
    }
    state = ActVision(m, &(c[IDDEPTHCAMCOLORSR]), &(d[IDDEPTHCAMDEPTHSR]), act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallVision(C1 m[], RGBCam *c, DepthCam *d, int next) // カメラ画像から色や輪郭を用いて物体を検出する
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallVision(%d), カメラ画像から色や輪郭を用いて物体を検出する\n", d -> id);
#endif
	act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	act_param[AP_FILTER] = FOC; act_param[AP_IMGORCOL] = RED; // 赤色検出
//	act_param[AP_FILTER] = LAPLACIAN; act_param[AP_IMGORCOL] = DEPTH; // 輪郭、深度、LAPLACIAN
    }
    state = ActVision(m, c, d, act_param); // 前カメラ
    if (ACT_END == state) {
	if (0 < act_param[AP_OBJFIND]) { // 1個以上見つけたら
	    for (int i = 0; i < (int)(d -> fobjnum); i++) {
	        fprintf(stderr, "物体%dの座標 x:%.3f y:%.3f z:%.3f\n", d -> fobjnum, 
			d -> fobjxyz[(int)(d -> fobjnum) -1][0],
			d -> fobjxyz[(int)(d -> fobjnum) -1][1],
			d -> fobjxyz[(int)(d -> fobjnum) -1][2]);
	    }
	    next_a[ACT_VISION] = ACT_GRABRELEASE; // それを掴む、放す
	}
	else { // 見つからなかったら
	    next_a[ACT_VISION] = ACT_LESSON1;
	}
    }
    return CALLMATCH; // 起動したことを返り値で戻す
}

int SimpleVision(C1 m[], RGBCam *c, DepthCam *d, int next) // カメラ画像から色や輪郭を用いて物体検出
{
    fprintf(stderr, "SimpleVision(), カメラ画像を処理する\n");

    float x, y, z;            // 計算後のローカル座標を記憶する変数
    float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要
    float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要
    int filter = FOC;
    int imgORcol = RED;
    cv::Mat in;
    if (DEPTH == imgORcol) in = d -> cv_ptr -> image; // DEPTH
    else                   in = c -> cv_ptr -> image; // COLOR(RGB), RED, GREEN, ...
    int objs;                            // 見つけた領域、物体の個数

    objs = imFindObjs(&in, filter, imgORcol, SHOW, "imFindObjs",
		      c -> fareacog, c -> fareacorners);
    c -> fareanum = objs; // 検出した領域の個数
    d -> fobjnum  = objs; // 検出した物体の個数、両者は同じ値になる

    if (0 < objs) {        // もし注目領域、物体があれば
	int a[MAXOBJS*2];  // 2次元座標なので要素数の2倍の記憶領域が必要
	int n = 0; // カウンタ
	for (int i = 0; i < objs; i++) {
	    a[n++] = c -> fareacog[i][0]; // x 座標、画像中に + マークを表示するため
	    a[n++] = c -> fareacog[i][1]; // y 座標
// カメラ画像の window で注目する画素の位置（重心）に + マークを描画する、不要なら省略可
	    c -> SetMark(objs, a);
// 次の処理で画像中の座標から3次元ローカル座標を求める
	    d -> LocalXyz(c -> fareacog[i][0], c -> fareacog[i][1], &x, &y, &z, ha, va);
// 検出した個々の物体の3次元ローカル座標を配列に記憶する
	    d -> fobjxyz[i][0] = x;
	    d -> fobjxyz[i][1] = y; // depthclass.h 中の宣言で float fobjxyz[MAXOBJS][XYZ];
	    d -> fobjxyz[i][2] = z; // となっていることを要確認、もし最後が [XY] なら要修正
	    fprintf(stderr, "物体%dの座標 x:%.3f y:%.3f z:%.3f\n", i, 
		    d -> fobjxyz[i][0], d -> fobjxyz[i][1], d -> fobjxyz[i][2]);
	}
    }

    if (0 < objs) { // 1個以上見つけたら
	next_a[ACT_VISION] = ACT_GRABRELEASE; // それを掴む、放す
    }
    else {
	next_a[ACT_VISION] = ACT_LESSON1;
    }

    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallVision3Cam(C1 m[], RGBCam *c, DepthCam *d, int next) // カメラ画像から色や輪郭を用いて物体を検出する
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallVision3Cam(), カメラ画像から色や輪郭を用いて物体を検出する\n");
#endif
	act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	act_param[AP_FILTER] = FOC; act_param[AP_IMGORCOL] = RED;
//	act_param[AP_FILTER] = LAPLACIAN; act_param[AP_IMGORCOL] = DEPTH;
    }
// 以下は３台のカメラを利用する例、カメラの指定の仕方は以下のようになる
// 返り値 state の処理がまだ決まっていない
    state = ActVision(m, c, d, act_param);             // Front Camera
    state = ActVision(m, &(c[1]), &(d[1]), act_param); // Left Camera, 配列形式の指定
    state = ActVision(m, c+2, d+2, act_param);         // Right Camera, ポインタ形式
    if (ACT_END == state) {
	if (0 < act_param[AP_OBJFIND]) { // 1個以上見つけたら
	    int camid = IDDEPTHCAMDEPTH; // or IDDEPTHCAMDEPTHSL or IDDEPTHCAMDEPTHSR
	    for (int i = 0; i < (int)(d[camid]. fobjnum); i++) {
	        fprintf(stderr, "物体%dの座標 x:%.2f y:%.2f z:%.2f\n", d[camid]. fobjnum, 
// データアクセスの書き方の例
// 新しい書き方、camid を指定することでカメラの切り替えが可能
			d[camid]. fobjxyz[(int)(d[camid]. fobjnum) -1][0],
			d[camid]. fobjxyz[(int)(d[camid]. fobjnum) -1][1],
			d[camid]. fobjxyz[(int)(d[camid]. fobjnum) -1][2]);
// または、こちらの形式で書くことも可能、意味は同じ
/*			(d + camid) -> fobjxyz[(int)((d + camid) -> fobjnum) -1][0],
			(d + camid) -> fobjxyz[(int)((d + camid) -> fobjnum) -1][1],
			(d + camid) -> fobjxyz[(int)((d + camid) -> fobjnum) -1][2]); */
// 従来の書き方、これだと Front Camera のみ指定することになる
/*			d -> fobjxyz[(int)(d -> fobjnum) -1][0],
			d -> fobjxyz[(int)(d -> fobjnum) -1][1],
			d -> fobjxyz[(int)(d -> fobjnum) -1][2]); */
	    }
	    next_a[ACT_VISION3CAM] = ACT_GRABRELEASE; // それを掴む、放す
	}
	else { // 見つからなかったら
	    next_a[ACT_VISION3CAM] = ACT_LESSON1;
	}
    }
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallVisionDemo(C1 m[], RGBCam *c, DepthCam *d, int next) // 色や輪郭を用いて物体を検出するデモ
{
    static int iter = 0; // デモの繰り返し回数

    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallVisionDemo(), カメラ画像から物体を検出するデモ %d\n", iter);
#endif
	act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	if (0 == iter) {
	    act_param[AP_FILTER] = FOC; act_param[AP_IMGORCOL] = RED;
	}
	else if (1 == iter) {
	    act_param[AP_FILTER] = SOBEL; act_param[AP_IMGORCOL] = COLOR;
	}
	else if (2 == iter) {
	    act_param[AP_FILTER] = SOBEL; act_param[AP_IMGORCOL] = DEPTH;
	}
	else if (3 == iter) {
	    act_param[AP_FILTER] = LAPLACIAN; act_param[AP_IMGORCOL] = COLOR;
	}
	else if (4 == iter) {
	    act_param[AP_FILTER] = LAPLACIAN; act_param[AP_IMGORCOL] = DEPTH;
	}
	else { // 以下の値の表示は前カメラの結果
	    fprintf(stderr, "画像処理で検出した物体の3次元座標のアクセスの例\n");
	    fprintf(stderr, "検出した物体の個数 %d\n", d -> fobjnum);
	    fprintf(stderr, "1個めの物体の座標 x:%.3f y:%.3f z:%.3f\n",
		    d -> fobjxyz[0][0], d -> fobjxyz[0][1], d -> fobjxyz[0][2]);
	    fprintf(stderr, "%d個めの物体の座標 x:%.3f y:%.3f z:%.3f\n", d -> fobjnum, 
		    d -> fobjxyz[(int)(d -> fobjnum) -1][0],
		    d -> fobjxyz[(int)(d -> fobjnum) -1][1],
		    d -> fobjxyz[(int)(d -> fobjnum) -1][2]);
	}
    }
    state = ActVision(m, c, d, act_param);
    if (ACT_END == state) {
	if (iter <= 4) { // デモを繰り返す
	    iter++;
	    next_a[ACT_VISIONDEMO] = ACT_VISIONDEMO;
	}
	else { // 次の処理
	    iter = 0;
	    next_a[ACT_VISIONDEMO] = ACT_BODYSHIFT;
	}
    }
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallFTDemo(C1 m[], FTSENSOR ft[], int next) // FTセンサのデータにアクセスするデモ
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallFTDemo(), FTセンサのデータにアクセスするデモ\n");
	fprintf(stderr, "              10秒以内に、背中の緑色の箱を下ろして値の変化を見よう\n");
#endif
	fprintf(stderr, "背中の青いFTセンサの値、直近 0.1秒の平均 ");
	act_param[AP_TIMETOTAL] = 3; // 動作時間 (sec)
    }
    state = ActStay(m, act_param); // これは静止する機能で、FTセンサには関係ない
    if (ACT_MOVING == state) {
	if (0 == (int)act_param[AP_ELAPSEDTIME] % 10) { // 0.1sec に1回表示
	    fprintf(stderr, "%.2f ", ft[FT5]. Ftlastave());
	}
    }
    if (ACT_END == state) {
	fprintf(stderr, "\n");
    }
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallGrabRelease(C1 m[], DepthCam *d, int next) // ものを掴み持ち上げてロボット前に移動後放す
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallGrabRelease(), ものを掴み持ち上げてロボット前に移動後放す\n");
#endif
	act_param[AP_TIMETOTAL] = 16.0; // 動作時間 (sec)
	act_param[AP_TIME1] = 2.0; // 動作時間 (sec)
	act_param[AP_TIME2] = 2.0; // 動作時間 (sec)
	act_param[AP_TIME3] = 2.0; // 動作時間 (sec)
	act_param[AP_TIME4] = 2.0; // 動作時間 (sec)
	act_param[AP_TIME5] = 2.0; // 動作時間 (sec)
	act_param[AP_TIME6] = 2.0;
	act_param[AP_TIME7] = 2.0;
	act_param[AP_TIME8] = 2.0;
	act_num = ACT_GRABRELEASE; // 動作中の動きの登録
    }

// ActGrabRelease() の呼び出しは以下のどれか一つ
    
// 今までの書き方、これは前カメラの指定に相当する
//  state = ActGrabRelease(m, d, act_param);

// 新しい書き方、ポインタ形式、これは前カメラ
// IDDEPTHCAMDEPTH の定義は const.h にあり、前カメラの ID で値は 0
    state = ActGrabRelease(m, d+IDDEPTHCAMDEPTH, act_param);
//  state = ActGrabRelease(m, d+0, act_param);
// 新しい書き方、ポインタ形式、これは左カメラ
//	state = ActGrabRelease(m, d+IDDEPTHCAMDEPTHSL, act_param);
//  state = ActGrabRelease(m, d+1, act_param);
// 別の新しい書き方、配列形式、これは右カメラ
//  state = ActGrabRelease(m, &(d[IDDEPTHCAMDEPTHSR]), act_param);
//  state = ActGrabRelease(m, &(d[2]), act_param);

    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallGrabReleaseLeft(C1 m[], DepthCam *d, int next) // ものを掴み持ち上げてロボット前に移動後放す
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallGrabReleaseLeft(), ものを掴み持ち上げてロボット前に移動後放す\n");
#endif
	act_param[AP_TIMETOTAL] = 11.0; // 動作時間 (sec)
	act_param[AP_TIME1] = 1.0; // 動作時間 (sec)
	act_param[AP_TIME2] = 2.0; // 動作時間 (sec)
	act_param[AP_TIME3] = 3.0; // 動作時間 (sec)
	act_param[AP_TIME4] = 2.0; // 動作時間 (sec)
	act_param[AP_TIME5] = 1.0; // 動作時間 (sec)
	act_param[AP_TIME6] = 2.0;
	act_num = ACT_GRL; // 動作中の動きの登録
	}
	// 新しい書き方、ポインタ形式、これは左カメラ
	state = ActGrabReleaseLeft(m, d+IDDEPTHCAMDEPTHSL, act_param);
	return CALLMATCH; // 起動したことを返り値で戻す
}

int CallShell(C1 m[], int next) // shell() による操作
{
//  if (ACT_START == (int)act_param[AP_STATE]) { // この関数では不要
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
//	fprintf(stderr, "CallShell(), shell() による操作\n");
#endif
//  act_num = ACT_SHELL; // 動作中の動きの登録
    act_param[AP_STATE] = ACT_MOVING;                                                            
    state = ACT_MOVING;

    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallShellSleep(C1 m[], int next) // shell() による操作のためにロボットの動きを静止させて待つ
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
//	fprintf(stderr, "CallShellSleep(), shell() による操作のために動きを止めて待つ\n");
// シェルのコマンド入力の邪魔になるので例外的に個別にコメントアウトした
#endif
	act_param[AP_TIMETOTAL] = 10.0; // 動作時間 (sec)
//	act_num = ACT_SHELLSLEEP; // 動作中の動きの登録
    }
    state = ActStay(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
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
#ifdef ACTEXEC
	fprintf(stderr, "ActLesson3(), 1番目の動き\n");
#endif
	float target1[XYZ] = { 0.65, 0.35, 0.5};
	float target2[XYZ] = { 0.65, 0.05, 0.5};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ２番目の動き
    if (cycle1 == (timer % cycletotal)) { // cycle1 回呼び出されたら次の動きに移る
#ifdef ACTEXEC
	fprintf(stderr, "ActLesson3(), 2番目の動き\n");
#endif
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
#ifdef ACTEXEC
	fprintf(stderr, "ActLesson3(), 3番目の動き\n");
#endif
	ikparam[0] = time3; // 動きによって時間が異なる場合
	float target1[XYZ] = { 0.65,-0.05, 0.1};
	float target2[XYZ] = { 0.65,-0.35, 0.1};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ４番目の動き
    if ((cycle1 + cycle2 + cycle3) == (timer % cycletotal)) { // 次の動きに移る
#ifdef ACTEXEC
	fprintf(stderr, "ActLesson3(), 4番目の動き\n");
#endif
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
	fprintf(stderr, "end ActLesson3()\n");
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

int LessonShell(char token[][STRBUF])
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

/*
int CallLesson0(C1 m[], int next) // Call????() の書き方の例
{
    int goflag = NO;
    if (ACT_LESSON0 == act_next) goflag = YES;
//  if (ACT_???? == act_last) goflag = YES;
    if (NO == goflag) return CALLTHRU;
    
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これは書かない
#ifdef CALLMSG
	fprintf(stderr, "CallLesson0(), Call????() の書き方の例\n");
#endif
	act_param[AP_TIMETOTAL] = 3.0;
//	act_param[AP_????1] = 1.0; // 必要なパラメータの値を設定する
//	act_param[AP_????2] = 2.0;
	act_num = ACT_LESSON0;
    }

    state = ActLesson0(m, act_param);

    if (ACT_END == state) act_next = next;
    return CALLMATCH;
}

int CallLesson0(C1 m[], int next) // Call????() の書き方の例、コメント付
{
// ここから初期設定、最初に1回だけ実行される    
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLesson0(), Call????() の書き方の例\n");
#endif
	act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
//	act_param[AP_????1] = 1.0; // Act????() の実行に必要なパラメータを
//	act_param[AP_????2] = 2.0; // 何個でも設定できる 
	act_num = ACT_LESSON0; // 選択した動作の番号を登録
    }
// ここまで初期設定

// ここで Act????() を実行する
    state = ActLesson0(m, act_param);

// 実行終了なら次の動きを指定可能、この機能を使うと動作の繋がりの管理が楽
    if (ACT_END == state) act_next = next; // 必要なら次の動きを指定可能

// Act????() を実行したら返り値で呼び出し元に通知する 
    return CALLMATCH; // 起動したことを返り値で戻す
}
*/
