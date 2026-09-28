// shell.cpp

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/WrenchStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// belows are for kbhit()
// #include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "const.h"
#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h"
// #include "lidarclass.h" // ここでは LIDAR を使ってないので
#include "ftclass.h"

#include "user.h"

const int  SHELLCONTINUE = 0; // shell() から抜けるか留まるか
const int  SHELLBREAK    = 1;

// shell, web に入るときに直前の動作を中断するか、
// 抜ける時に再開するかを設定するフラグ
const int  SHELL_LASTACT_SUSPEND = YES; // YES は shell に入るときに直前の動作を中断する
const int  SHELL_LASTACT_RESUME  = YES; // YES は shell から抜ける時に中断した動作を再開する
const int  WEB_LASTACT_SUSPEND   = NO;
const int  WEB_LASTACT_RESUME    = NO;

const int  NUMCMDARY = 8; // パラメータ受け渡し用の配列 cmdary[] の要素数

const char TAB = 0x09;
const char RET = 0x0a;
const char SPC = 0x20;
const char CMT = 0x23;

/* ACT_???? などの定数の定義は const.h にある
const int ACT_INITPOSE = 0; // 初期姿勢
const int ACT_ ...
*/

// 以下はマルチスレッドを使うため
#include <pthread.h>
pthread_t gTID;  // for shell
pthread_t gTID2; // for Web

int gShellOpen = NO; // マルチスレッドでシェルを開いているか判定するフラグ

// Web, Flask から publish された指示を受けて処理をしているか判定するフラグ
int gWebOpen = NO;
// Web, Flask 用の変数、関数内に置くのが難しいため
float gCmdAry[10] = {0};
int gLastAct = 0;

// 次のポインタは、いくつかのデータを、オブジェクト外の AI の関数 
// intelligence() やシェルの関数からまとめてアクセスできるようにするため
extern C1 *gMotor;
extern RGBCam *gDepthRGB;
extern DepthCam *gDepthD;

// これらの変数は shell() から様々な指示をするために必要
// 変数の実体は user.cpp にある                                                                      
extern int act_num;       // 次の行動を決める値、重要
extern int act_last;      // 直前の行動を覚えておく
extern int state;         // 動作の状態を表す、「開始、継続中、終了」
extern float act_param[]; // パラメータの受け渡し用配列      
extern int act_next;      // 次の行動を指定する

void   openshell(const ros::TimerEvent&);
void*  shell(void* pParam);
void*  shellsimple(void* pParam);
int    dispatch(float cmdary[], int &lastact);
int    dispatchOrg(char token[][STRBUF], int &lastact);
int    kbhit(void);
int    TokenStr(char str[], char str2[][STRBUF]);
void   token2cmdary(char token[][STRBUF], float cmdary[]);
int    Name2Num(char name[]);
void   readDyna(char fname[]);
void   flasksubCallback(const std_msgs::Float32MultiArray& array);
void*  webshell(void* pParam);

float  R2D(float rad); // translate radian to degree
float  D2R(float deg);

int    UserShell(char token[][STRBUF]);
void   UserShellHelp(void);

int CallSetRobotPos(float xyzrpy[], int next);
int CallGetRobotPos(float xyzrpy[], int next);

void flasksubCallback(const std_msgs::Float32MultiArray& array)
{
//  ROS_INFO("Web: (size:%ld) %f %f %f ...", array.data.size(),
//           array.data[0], array.data[1], array.data[2]);
    gCmdAry[0] = array.data[0];
    gCmdAry[1] = array.data[1];
    gCmdAry[2] = array.data[2];
    gCmdAry[3] = array.data[3];
    gCmdAry[4] = array.data[4];
    gCmdAry[5] = array.data[5];

    if (YES == gShellOpen) {
	fprintf(stderr, "Shell processing, try again later.\n");
	return;
    }
    gWebOpen = YES;

    pthread_create(&gTID2, NULL, webshell, NULL); // マルチスレッドで webshell 起動

    gWebOpen = NO;
}

void* webshell(void* pParam) // web 用シェル
{
// ここからの３行でシェルを開いた時点で現在の動作を中断して動きを止める
// シェルを開いた時点で動き続けて構わなければこの３行は実行しなくてよい
// 中断している動作の再開はシェルのコマンド resume を実行する
    if (YES == WEB_LASTACT_SUSPEND) {
	state = ACT_END; // グローバル変数、実体は別ファイル
	gLastAct = act_num; // ローカル変数、webshell() からの復帰のために記憶しておく
	act_num = ACT_SHELLSLEEP; // グローバル変数、実体は別ファイル
    }
    
    int rcode = dispatch(gCmdAry, gLastAct); // 返り値は使わない、受け取るだけ

    pthread_join(gTID2, NULL);

    if (YES == WEB_LASTACT_RESUME) {
	state = ACT_MOVING;
	act_num = gLastAct; // 再開する関数の timer は中断前の値が関数内に残っているので問題ない
	act_next= gLastAct; // 再開する関数の timer は中断前の値が関数内に残っているので問題ない
    }
    return (void*)NULL;
}

void openshell(const ros::TimerEvent&) // シェルの開始、1/100 秒毎にキー入力をチェック
{
    char c;
    
    if (NO == gShellOpen && YES == kbhit()) { // 現在シェルを起動していなくて、キー入力があれば
	if (YES == gWebOpen) {
	    fprintf(stderr, "Web processing, try again later.\n");
	    return;
	}
	c = getchar();
	if ('$' == c) { // '$' がシェルを起動する文字
	    gShellOpen = YES;
	    pthread_create(&gTID, NULL, shell, NULL); // マルチスレッドで shell 起動
	}
	if ('0' == c) { // '0' で原点に移動
	    float xyzrpy[6] = {0, 0, 0.3, 0, 0, 0};
	    act_next = ACT_SETROBOTPOS; // dummy, CallSetRobotPos() を動かすため
	    CallSetRobotPos(xyzrpy, 0);
//	    gShellOpen = YES;
//	    pthread_create(&gTID, NULL, shellsimple, NULL); // マルチスレッドにするとき
	}
	if ('w' == c) { // 'w' で指定した座標に移動、ワープ
	    float xyzrpy[6] = {5, 5, 0.3, 0, 0, 0};
	    act_next = ACT_SETROBOTPOS; // dummy, CallSetRobotPos() を動かすため
	    CallSetRobotPos(xyzrpy, 0);
	}
    }
}

void* shell(void* pParam) // シェルの実体
{
    char cmdstr[256], token[NUMCMDARY][STRBUF];
    int tokens;
    int lastact = 0; // shell 用の lastact はここで多分問題ない

// 次の if 文の処理でシェルを開いた時点で現在の動作を中断して動きを止める。
// シェルを開いた時点で動き続けて構わなければここは実行しなくてよい
// 中断している動作の再開はシェルのコマンド resume を実行する
    if (YES == SHELL_LASTACT_SUSPEND) {
	state = ACT_END; // グローバル変数、実体は別ファイル
	lastact = act_num; // ローカル変数、復帰のために記憶しておく
	act_num = ACT_SHELLSLEEP; // グローバル変数、実体は別ファイル
	act_next = ACT_SHELLSLEEP; // グローバル変数、実体は別ファイル
    }
    
    printf("\nshell opened\n");
    while (TRUE) {
	printf("prg4$ "); // プロンプトの表示
	fgets(cmdstr, 256, stdin);
        tokens = TokenStr(cmdstr, token); // 文字列をスペース区切りの単語に分解
	if (0 == tokens) continue; // 入力なしなら再入力へ
	
	static float cmdary[NUMCMDARY] = {0}; // ループの実行中前の値を保持しておく
	token2cmdary(token, cmdary); // 文字列を数値の並びに変換
	if (SHELLBREAK == dispatch(cmdary, lastact)) { // 入力されたコマンドの動作を実行する
	    break;
	}
// 以下は動作を文字列で指定する旧 version
/*	if (SHELLBREAK == dispatchOrg(token, lastact)) { // 今までの処理
	    break;
	} */
    }
    gShellOpen = NO;
    printf("\nshell closed\n");
    pthread_join(gTID, NULL);
    
// シェルから抜ける時に、シェルに入る前に実行中だった動作を再開するか
    if (YES == SHELL_LASTACT_RESUME) { 
	state = ACT_MOVING;
	act_num = lastact; // 再開する関数の timer は中断前の値が関数内に残っているので問題ない
	act_next = lastact; // 再開する関数の timer は中断前の値が関数内に残っているので問題ない
    }

    return (void*)NULL;
}

void* shellsimple(void* pParam) // シェルの実体
{
    float xyzrpy[6] = {{0.0}}; xyzrpy[2] = 0.3;
    act_next = ACT_SETROBOTPOS; // dummy, CallSetRobotPos() を動かすため
    CallSetRobotPos(xyzrpy, 0);
    act_next = ACT_GETROBOTPOS; // dummy, CallSetRobotPos() を動かすため
    CallGetRobotPos(xyzrpy, 0);
    
    gShellOpen = NO;
    pthread_join(gTID, NULL);
    
    return (void*)NULL;
}

int dispatch(float cmdary[], int &lastact)
{
//  printf("%d ", (int)cmdary[0]);
    if (ACT_SLEEP == (int)cmdary[0]) { // 現在の動作を中断して動きを止める
	    state = ACT_END;
	    lastact = act_num; // lastact の実体はこのファイル冒頭部分の gLastAct
	    act_num = ACT_SHELLSLEEP;
	    act_next= ACT_SHELLSLEEP;
	    return SHELLCONTINUE;
	}
    else if (ACT_RESUME == (int)cmdary[0]) { // 中断した動作を再開する
	    printf("Resume last action, wait a moment\n");
            state = ACT_MOVING;
            act_num = lastact; // 再開する関数の timer は中断前の値が関数内に残っているので問題ない
            act_next= lastact; // 再開する関数の timer は中断前の値が関数内に残っているので問題ない
	    return SHELLCONTINUE;
        }
    else if (ACT_QUIT == (int)cmdary[0]) {
	    return SHELLBREAK;
	}
    else if (ACT_HELP == (int)cmdary[0]) {
	    printf("$              : open shell\n");
	    printf("quit (q, Q)    : close shell\n");
	    printf("!!             : repeat last command\n");
	    printf("sleep (S)      : suspend current motion\n");
	    printf("resume (R)     : resume last motion before entering shell\n");
	    printf("help (H, h, ?) : print help messages\n");
	    printf("angles         : print angles(rad) for Dynamixel\n");
	    printf("allangles (A)  : print angles(deg) of all joints\n");
	    printf("bodyshift (B) x y z sw : bodyshift (ex. B 0.1 0 0 63)\n");
	    printf("dance (D) sec  : dance (ex. dance 10)\n");
	    printf("init (I)       : init pose\n");
	    printf("ik nl x y z    : move leg by ik (ex. ik a1 0.5 0.5 0.5)\n");
	    printf("jump           : jump\n");
	    printf("mjda nj deg    : move joint degree absolute (ex. mjda a1j1 10)\n");
	    printf("mjra nj rad    : move joint radian absolute (ex. mjra a1j1 0.1)\n");
	    printf("mjdi nj deg    : move joint degree increment (ex. mjdi a1j1 10)\n");
	    printf("mjri nj rad    : move joint radian increment (ex. mjri a1j1 0.1)\n");
	    printf("pjd nj         : print joint degree (ex. pjd a2j4)\n");
	    printf("pjr nj         : print joint radian (ex. pjr a4j6)\n");
	    printf("rdyna filename : read dynamixel data\n");
	    printf("relax          : relax, reset all angles to 0 deg\n");
	    printf("stand          : stand up\n");
	    printf("stand2         : stand up another version\n");
	    printf("tilt p r y sw  : body tilt (ex. tilt 0.1 0 0 63)\n");
	    printf("turn (T) l/r iter : turn (ex. turn r 3)\n");
	    printf("vision (V)     : vision, image processing and display\n");
	    printf("walk (W) f/b iter : walk (ex. walk f 5)\n");
	    UserShellHelp();
	    return SHELLCONTINUE;
	}

/////////////////////////////////////////////////////////////////////////////////////////

	else if (ACT_DANCE == (int)cmdary[0]) {
	    for (int i = 0; i < (int)cmdary[1] * 100; i++) {
		Dance(gMotor);
                usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (ACT_IK == (int)cmdary[0]) {
	    float ikparam[IKNUMPARAM];
	    ikparam[0] = 1.0;
	    ikparam[1] = IKHAND;
	    ikparam[2] = IKELBOWUP;
	    ikparam[3] = (float)MODENONE;
	    float target[XYZ] = {0};
	    target[0] = cmdary[2];
	    target[1] = cmdary[3];
	    target[2] = cmdary[4];
	    IKMotions(gMotor, (int)cmdary[1], target, ikparam); 
	    return SHELLCONTINUE;
        }
	else if (ACT_BODYSHIFT == (int)cmdary[0]) {
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_TIMETOTAL] = 3.0; // 合計動作時間 (sec)
	    act_param[AP_BSHIFTX] = cmdary[1];
	    act_param[AP_BSHIFTY] = cmdary[2];
	    act_param[AP_BSHIFTZ] = cmdary[3];
	    act_param[AP_BSHIFTFLAG] = (int)cmdary[4]; // 動かす脚のオンオフの設定, 6bits
//	    act_param[AP_BSHIFTFLAG] = (float)encode_leg(bsflag); // 動かす脚のオンオフの設定
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActBodyShift(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
        }
	else if (ACT_TILT == (int)cmdary[0]) {
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_TIMETOTAL] = 3.0; // 合計動作時間 (sec)
	    act_param[AP_BSHIFTX] = cmdary[1]; // pitch
	    act_param[AP_BSHIFTY] = cmdary[2]; // roll
	    act_param[AP_BSHIFTZ] = cmdary[3]; // ヨー
	    act_param[AP_BSHIFTFLAG] = (int)cmdary[4]; // 動かす脚のオンオフの設定, 6bits
//	    act_param[AP_BSHIFTFLAG] = (float)encode_leg(bsflag); // 動かす脚のオンオフの設定
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActTilt(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
        }
	else if (ACT_INITPOSE == (int)cmdary[0]) {
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActInitialPose(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (ACT_STAND == (int)cmdary[0]) {
	    char n2[] = "n2";
	    gMotor[Name2Num(n2)].targetangle.data = 0.0; // 首をまっすぐに 
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActStandup(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (ACT_STAND2 == (int)cmdary[0]) {
//	    gMotor[Name2Num("n2")].targetangle.data = 0.0; // 首をまっすぐに 
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActStandup2(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (ACT_JUMP == (int)cmdary[0]) {
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 10.0; // 動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActJump(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (ACT_MJDA == (int)cmdary[0]) {
	    gMotor[(int)cmdary[1]].ctrlmode = MODENONE;
	    gMotor[(int)cmdary[1]].duration = 1.0;
	    gMotor[(int)cmdary[1]].targetangle.data = D2R(cmdary[2]);
	    return SHELLCONTINUE;
        }
	else if (ACT_MJRA == (int)cmdary[0]) {
	    gMotor[(int)cmdary[1]].ctrlmode = MODENONE;
	    gMotor[(int)cmdary[1]].duration = 1.0;
	    gMotor[(int)cmdary[1]].targetangle.data = cmdary[2];
	    return SHELLCONTINUE;
        }
	else if (ACT_MJDI == (int)cmdary[0]) {
	    gMotor[(int)cmdary[1]].ctrlmode = MODENONE;
	    gMotor[(int)cmdary[1]].duration = 1.0;
	    gMotor[(int)cmdary[1]].targetangle.data += D2R(cmdary[2]);
	    return SHELLCONTINUE;
        }
	else if (ACT_MJRI == (int)cmdary[0]) {
	    gMotor[(int)cmdary[1]].ctrlmode = MODENONE;
	    gMotor[(int)cmdary[1]].duration = 1.0;
	    gMotor[(int)cmdary[1]].targetangle.data += cmdary[2];
	    return SHELLCONTINUE;
        }
	else if (ACT_PJD == (int)cmdary[0]) {
	    printf("%f\n", R2D(gMotor[(int)cmdary[1]].anglenow.data));
	    return SHELLCONTINUE;
        }
	else if (ACT_PJR == (int)cmdary[0]) {
	    printf("%f\n", gMotor[(int)cmdary[1]].anglenow.data);
	    return SHELLCONTINUE;
        }
	else if (ACT_TURNP1 == (int)cmdary[0]) {
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 0.5;         // 最後の着地の動作時間 (sec)
            act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
	    act_param[AP_ITER] = cmdary[2];
            act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
	    act_param[AP_TURN_DIR] = cmdary[1];
            act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActTurn(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (ACT_WALKP1 == (int)cmdary[0]) {
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 0.5;         // 最後の着地の動作時間 (sec)
            act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
	    act_param[AP_ITER] = cmdary[2];
            act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
	    act_param[AP_WALK_DIR] = cmdary[1];
            act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
                                      act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActWalk(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
            }
	    return SHELLCONTINUE;
	}
	else if (ACT_VISION == (int)cmdary[0]) {
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 0.01;
	    act_param[AP_FILTER] = FOC;       // cmdary[1]; 実行時に変更するには入力部を変える
	    act_param[AP_IMGORCOL] = RED;     // cmdary[2];
//	    act_param[AP_FILTER] = LAPLACIAN; // cmdary[1]; 実行時に変更するには入力部を変える
//	    act_param[AP_IMGORCOL] = DEPTH;   // cmdary[2];
	    int state = ActVision(gMotor, gDepthRGB, gDepthD, act_param);
/*	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { // 1回で終わる、繰返し無
		int state = ActVision(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
            }
*/
	    return SHELLCONTINUE;
	}
	else if (ACT_NECK == (int)cmdary[0]) {
	    gMotor[A7J1].ctrlmode = MODENONE;
	    gMotor[A7J1].duration = 2.0;
	    gMotor[A7J1].targetangle.data += D2R(cmdary[1]);
	    gMotor[A7J2].ctrlmode = MODENONE;
	    gMotor[A7J2].duration = 2.0;
	    gMotor[A7J2].targetangle.data += D2R(cmdary[2]);
	    return SHELLCONTINUE;
	}
	else if (ACT_RDYNA == (int)cmdary[0]) {
	    char fname[] = "dyna.dat";
	    readDyna(fname); // cmdary[] ではファイル名を渡せないので
	    return SHELLCONTINUE;
        }
	else if (ACT_RELAX == (int)cmdary[0]) {
	    for (int i = 0; i < NUMALLJOINTS; i++) {
		gMotor[i].ctrlmode = MODEPROFILE;
		gMotor[i].duration = 3.0;
		gMotor[i].targetangle.data = 0.0;
	    }
	    usleep(3*1000*1000); // wait specified sec
	    return SHELLCONTINUE;
        }
	else if (ACT_ANGLES == (int)cmdary[0]) { // これは多分実機用、このまま残せ
	    printf("1.0 %.3f %.3f %.3f %.3f %.3f %.3f ",
	      gMotor[A4J1].anglenow.data, gMotor[A4J2].anglenow.data,
	      gMotor[A4J3].anglenow.data * -1, 
	      gMotor[A1J1].anglenow.data, gMotor[A1J2].anglenow.data,
	      gMotor[A1J3].anglenow.data * -1
	      );
	    printf("%.3f %.3f %.3f %.3f %.3f %.3f ",
	      gMotor[A5J1].anglenow.data, gMotor[A5J2].anglenow.data,
	      gMotor[A5J3].anglenow.data * -1, 
	      gMotor[A2J1].anglenow.data, gMotor[A2J2].anglenow.data,
	      gMotor[A2J3].anglenow.data * -1
	      );
	    printf("%.3f %.3f %.3f %.3f %.3f %.3f\n",
	      gMotor[A6J1].anglenow.data, gMotor[A6J2].anglenow.data,
	      gMotor[A6J3].anglenow.data * -1, 
	      gMotor[A3J1].anglenow.data, gMotor[A3J2].anglenow.data,
	      gMotor[A3J3].anglenow.data * -1
	      );
	    return SHELLCONTINUE;
	}
	else if (ACT_ALLANGLES == (int)cmdary[0]) { // こちらは普通の表示
	    printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
		   R2D(gMotor[A1J1].anglenow.data), R2D(gMotor[A1J2].anglenow.data),
		   R2D(gMotor[A1J3].anglenow.data), R2D(gMotor[A1J4].anglenow.data),
		   R2D(gMotor[A1J5].anglenow.data), R2D(gMotor[A1J6].anglenow.data),
		   R2D(gMotor[A1J7].anglenow.data), R2D(gMotor[A1J8].anglenow.data)
		   );
	    printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
		   R2D(gMotor[A2J1].anglenow.data), R2D(gMotor[A2J2].anglenow.data),
		   R2D(gMotor[A2J3].anglenow.data), R2D(gMotor[A2J4].anglenow.data),
		   R2D(gMotor[A2J5].anglenow.data), R2D(gMotor[A2J6].anglenow.data),
		   R2D(gMotor[A2J7].anglenow.data), R2D(gMotor[A2J8].anglenow.data)
		   );
	    printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
		   R2D(gMotor[A3J1].anglenow.data), R2D(gMotor[A3J2].anglenow.data),
		   R2D(gMotor[A3J3].anglenow.data), R2D(gMotor[A3J4].anglenow.data),
		   R2D(gMotor[A3J5].anglenow.data), R2D(gMotor[A3J6].anglenow.data),
		   R2D(gMotor[A3J7].anglenow.data), R2D(gMotor[A3J8].anglenow.data)
		   );
	    printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
		   R2D(gMotor[A4J1].anglenow.data), R2D(gMotor[A4J2].anglenow.data),
		   R2D(gMotor[A4J3].anglenow.data), R2D(gMotor[A4J4].anglenow.data),
		   R2D(gMotor[A4J5].anglenow.data), R2D(gMotor[A4J6].anglenow.data),
		   R2D(gMotor[A4J7].anglenow.data), R2D(gMotor[A4J8].anglenow.data)
		   );
	    printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
		   R2D(gMotor[A5J1].anglenow.data), R2D(gMotor[A5J2].anglenow.data),
		   R2D(gMotor[A5J3].anglenow.data), R2D(gMotor[A5J4].anglenow.data),
		   R2D(gMotor[A5J5].anglenow.data), R2D(gMotor[A5J6].anglenow.data),
		   R2D(gMotor[A5J7].anglenow.data), R2D(gMotor[A5J8].anglenow.data)
		   );
	    printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
		   R2D(gMotor[A6J1].anglenow.data), R2D(gMotor[A6J2].anglenow.data),
		   R2D(gMotor[A6J3].anglenow.data), R2D(gMotor[A6J4].anglenow.data),
		   R2D(gMotor[A6J5].anglenow.data), R2D(gMotor[A6J6].anglenow.data),
		   R2D(gMotor[A6J7].anglenow.data), R2D(gMotor[A6J8].anglenow.data)
		   );
	    printf("%.2f %.2f\n",
		   R2D(gMotor[A7J1].anglenow.data), R2D(gMotor[A7J2].anglenow.data)
		   );
	    return SHELLCONTINUE;
	}

/////////////////////////////////////////////////////////////////////////////////////////
/*
	else if (YES == UserShell(token)) { // 自分のシェルで独自のコマンドを追加可能
	    NOTHING; // UserShell() を実行した後はここでは何もすることはない
	    return SHELLCONTINUE;
        }
*/
	else if (ACT_NONE == (int)cmdary[0]) {
//          Do nothing;
	    return SHELLCONTINUE;
	}
	else {
	    printf("invalid command\n");
	    return SHELLCONTINUE;
	}
}

int kbhit(void) // リアルタイムキー入力
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return YES;
    }
    return NO;
}

void token2cmdary(char token[NUMCMDARY][STRBUF], float cmdary[NUMCMDARY])
{
    float lastcmdary[NUMCMDARY];

    for (int i = 0; i < NUMCMDARY; i++) lastcmdary[i] = cmdary[i]; // 前回のコマンドの保存

    if (0 == strcmp("sleep", token[0]) || 'S' == token[0][0]) { // 現在の動作を中断して動きを止める
	cmdary[0] = (float)ACT_SLEEP;
    }
    else if (0 == strcmp("resume", token[0]) || 'R' == token[0][0]) { // 中断した動作を再開する
	cmdary[0] = (float)ACT_RESUME;
    }
    else if (0 == strcmp("quit", token[0]) || 'q' == token[0][0] ||
	     'Q' == token[0][0]) {
	cmdary[0] = (float)ACT_QUIT;
    }
    else if (0 == strcmp("help", token[0]) || 'H' == token[0][0] ||
	     'h' == token[0][0] || '?' == token[0][0]) {
	cmdary[0] = (float)ACT_HELP;
    }

/////////////////////////////////////////////////////////////////////////////////////////

    else if (0 == strcmp("bodyshift", token[0]) || 'B' == token[0][0]) { // body shift
	cmdary[0] = (float)ACT_BODYSHIFT;
	cmdary[1] = atof(token[1]); // 目標座標 X
	cmdary[2] = atof(token[2]); // 目標座標 Y
	cmdary[3] = atof(token[3]); // 目標座標 Z
	cmdary[4] = atof(token[4]); // 動かす脚のオンオフ
    }
    else if (0 == strcmp("tilt", token[0])) { // body tilt
	cmdary[0] = (float)ACT_TILT;
	cmdary[1] = atof(token[1]); // ピッチ
	cmdary[2] = atof(token[2]); // ロール
	cmdary[3] = atof(token[3]); // ヨー
	cmdary[4] = atof(token[4]); // 動かす脚のオンオフ
    }
    else if (0 == strcmp("dance", token[0]) || 'D' == token[0][0]) { // dance
	cmdary[0] = (float)ACT_DANCE;
	cmdary[1] = atof(token[1]); // 秒
    }
    else if (0 == strcmp("ik", token[0])) { // move leg by inverse kinematics
	cmdary[0] = (float)ACT_IK;
	cmdary[1] = (float)Name2Num(token[1]); // 脚の名前から番号に変換
	cmdary[2] = atof(token[2]); // 目標座標 X
	cmdary[3] = atof(token[3]);	// 目標座標 Y
	cmdary[4] = atof(token[4]);	// 目標座標 Z
    }
    else if (0 == strcmp("init", token[0]) || 'I' == token[0][0]) { // init
	cmdary[0] = (float)ACT_INITPOSE;
    }
    else if (0 == strcmp("stand", token[0]) ) { // stand up
	cmdary[0] = (float)ACT_STAND;
    }
    else if (0 == strcmp("stand2", token[0]) ) { // stand up another version
	cmdary[0] = (float)ACT_STAND2;
    }
    else if (0 == strcmp("jump", token[0]) || 'J' == token[0][0]) { // jump
	cmdary[0] = (float)ACT_JUMP;
    }
    else if (0 == strcmp("mjda", token[0])) { // move joint degree absolute
	cmdary[0] = (float)ACT_MJDA;
	cmdary[1] = (float)Name2Num(token[1]);
	cmdary[2] = atof(token[2]);
    }
    else if (0 == strcmp("mjra", token[0])) { // move joint radian absolute
	cmdary[0] = (float)ACT_MJRA;
	cmdary[1] = (float)Name2Num(token[1]);
	cmdary[2] = atof(token[2]);
    }
    else if (0 == strcmp("mjdi", token[0])) { // move joint degree increment
	cmdary[0] = (float)ACT_MJDI;
	cmdary[1] = (float)Name2Num(token[1]);
	cmdary[2] =	atof(token[2]);
    }
    else if (0 == strcmp("mjri", token[0])) { // move joint radian increment
	cmdary[0] = (float)ACT_MJRI;
	cmdary[1] = (float)Name2Num(token[1]);
	cmdary[2] =	atof(token[2]);
    }
    else if (0 == strcmp("pjd", token[0])) { // print joint degree
	cmdary[0] = (float)ACT_PJD;
	cmdary[1] = (float)Name2Num(token[1]);
    }
    else if (0 == strcmp("pjr", token[0])) { // print joint radian
	cmdary[0] = (float)ACT_PJR;
	cmdary[1] = (float)Name2Num(token[1]);
    }
    else if (0 == strcmp("turn", token[0]) || 'T' == token[0][0]) { // turn
	cmdary[0] = (float)ACT_TURNP1;
	if ('l' == token[1][0]) cmdary[1] = (float)TURNCCW;
	else                    cmdary[1] = (float)TURNCW;
	cmdary[2] = atof(token[2]);
    }
    else if (0 == strcmp("walk", token[0]) || 'W' == token[0][0]) { // walk
	cmdary[0] = (float)ACT_WALKP1;
	if ('f' == token[1][0]) cmdary[1] = (float)FORWARD;
	else                    cmdary[1] = (float)BACKWARD;
	cmdary[2] = atof(token[2]);
    }
    else if (0 == strcmp("vision", token[0]) || 'V' == token[0][0]) { // vision
	cmdary[0] = (float)ACT_VISION;
    }
    else if (0 == strcmp("rdyna", token[0])) { // read dynamixel data
	cmdary[0] = (float)ACT_RDYNA;
	cmdary[1] = atof(token[1]); // ファイル名なのでとりあえず値だけ渡す
    }
    else if (0 == strcmp("relax", token[0])) { // relax
	cmdary[0] = (float)ACT_RELAX;
    }
    else if (0 == strcmp("angles", token[0])) { // print angles of joints
	cmdary[0] = (float)ACT_ANGLES;
    }
    else if (0 == strcmp("allangles", token[0]) || 'A' == token[0][0]) { // print all angles of joints
	cmdary[0] = (float)ACT_ALLANGLES;
    }
    else if (0 == strcmp("!!", token[0])) { // repeat last command
	for (int i = 0; i < NUMCMDARY; i++) cmdary[i] = lastcmdary[i]; 
    }

/////////////////////////////////////////////////////////////////////////////////////////

    else {
	printf("invalid command\n");
	cmdary[0] = (float)ACT_NONE; // 何もしない
    }
}

int TokenStr(char str[], char str2[][STRBUF])
{
    char        *ptr, *ptr2;
    int         j, n;

    n = 0;
    ptr = str;

    while (TRUE) {
        while ((SPC == *ptr) || (TAB == *ptr)) ptr++;
        if (((char)0 == *ptr) || (RET == *ptr) || (CMT == *ptr)) 
            return(n);
        ptr2 = ptr;
        j = 0;
        while ((SPC != *ptr) && (TAB != *ptr) && (CMT != *ptr) &&
               ((char)0 != *ptr) && (RET != *ptr)) {
            ptr++;
            j++;
        }
        strncpy(str2[n], ptr2, j);
        str2[n][j] = (char)0;
        n++;
	if (NUMCMDARY == n) {
	    fprintf(stderr, "Note: Many tokens, stop parsing.\n");
	    return(n);
	}
    }
}

int Name2Num(char name[])
{
    if ('0' <=  name[0] && name[0] <= '9') return atoi(name);
    
    if (2 == strlen(name)) {
	if ('a' == name[0] && '0' <=  name[1] && name[1] <= '9') {
	    if ('1' == name[1]) return 1;
	    if ('2' == name[1]) return 2;
	    if ('3' == name[1]) return 3;
	    if ('4' == name[1]) return 4;
	    if ('5' == name[1]) return 5;
	    if ('6' == name[1]) return 6;
	}
	else if ('n' == name[0]) {
	    if ('1' == name[1]) return 48;
	    if ('2' == name[1]) return 49;
	}
    }
    if (4 == strlen(name) && 'a' == name[0] && 'j' == name[2]) {
	if ('0' <=  name[1] && name[1] <= '9' &&
	    '0' <=  name[3] && name[3] <= '9') {
	    return (name[1] - '1') * 8 + (name[3] - '1');
	}
    }

    fprintf(stderr, "invalid arm/joint name\n");
    return 0;
}

void readDyna(char fname[])
// shell() の仕様変更に伴い、fname[] が機能しなくなっているので
// この関数を使う場合はファイル名に注意する
{
    int tokens = 0;
    int id[18] = {A4J1, A4J2, A4J3, A1J1, A1J2, A1J3, 
		  A5J1, A5J2, A5J3, A2J1, A2J2, A2J3, 
		  A6J1, A6J2, A6J3, A3J1, A3J2, A3J3};
    const int BUF = 256;
    char str[BUF], str2[64][STRBUF];
    FILE *fp1;

    if ((FILE *)NULL == (fp1 = fopen(fname, "r"))) {
        fprintf(stderr, "Error: [%s] can not open\n", fname);
	return;
    }
    while (NULL != fgets(str, BUF, fp1)) {
	tokens = TokenStr(str, str2);
	for (int i = 1; i < 19; i++) {
	    gMotor[id[i-1]].ctrlmode = MODENONE;
	    gMotor[id[i-1]].duration = atof(str2[0]);
	    gMotor[id[i-1]].targetangle.data = atof(str2[i]);
	}
	gMotor[id[2]].targetangle.data *= -1; // 各脚の j3 は Gazebo と Dynamixel で回転方向が逆
	gMotor[id[5]].targetangle.data *= -1;
	gMotor[id[8]].targetangle.data *= -1;
	gMotor[id[11]].targetangle.data *= -1;
	gMotor[id[14]].targetangle.data *= -1;
	gMotor[id[17]].targetangle.data *= -1;
	usleep(atof(str2[0])*1000*1000); // wait specified sec
    }
    fclose(fp1);
}

// 以下は旧 version, 現在は dispatch() を使用している
// 必要なくなったら消しても構わない
/*
int dispatchOrg(char token[][STRBUF], int &lastact)
{
	if (0 == strcmp("sleep", token[0]) || 'S' == token[0][0]) { // 現在の動作を中断して動きを止める
	    state = ACT_END;
	    lastact = act_num; // 実体は shell() にある 
	    act_num = ACT_SHELLSLEEP;
	    return SHELLCONTINUE;
	}
	else if (0 == strcmp("resume", token[0]) || 'R' == token[0][0]) { // 中断した動作を再開する
	    printf("Resume last action, wait a moment\n");
            state = ACT_MOVING;
            act_num = lastact; // 再開する関数の timer は中断前の値が関数内に残っているので問題ない
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("quit", token[0]) || 'q' == token[0][0] ||
		 'Q' == token[0][0]) {
	    return SHELLBREAK;
	}
	else if (0 == strcmp("help", token[0]) || 'H' == token[0][0] ||
		 'h' == token[0][0] || '?' == token[0][0]) {
	    printf("$              : open shell\n");
	    printf("quit (q, Q)    : close shell\n");
	    printf("(EnterKey)     : repeat last command\n");
	    printf("sleep (S)      : suspend current motion\n");
	    printf("resume (R)     : resume last motion\n");
	    printf("help (H, h, ?) : print help messages\n");
	    printf("init (I)       : init pose\n");
	    printf("ik nl x y z    : move leg by inverse kinematics\n");
	    printf("jump           : jump\n");
	    printf("mjda nj deg    : move joint degree absolute\n");
	    printf("mjra nj rad    : move joint radian absolute\n");
	    printf("mjdi nj deg    : move joint degree increment\n");
	    printf("mjri nj rad    : move joint radian increment\n");
	    printf("pjd nj         : print joint degree\n");
	    printf("pjr nj         : print joint radian\n");
	    printf("rdyna filename : read dynamixel data\n");
	    printf("relax          : relax\n");
	    printf("stand          : stand up\n");
	    printf("stand2         : stand up another version\n");
	    printf("turn (T) l/r iter : turn\n");
	    printf("walk (W) f/b iter : walk\n");
	    printf("angles (A)     : print angles of all leg joints\n");
	    printf("dance (D) sec  : dance\n");
	    UserShellHelp();
	    return SHELLCONTINUE;
	}

/////////////////////////////////////////////////////////////////////////////////////////

	else if (0 == strcmp("dance", token[0]) || 'D' == token[0][0]) { // dance
	    for (int i = 0; i < atoi(token[1]) * 100; i++) {
		Dance(gMotor);
                usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (0 == strcmp("ik", token[0])) { // move leg by inverse kinematics
	    float ikparam[IKNUMPARAM];
	    ikparam[0] = 1.0;
	    ikparam[1] = IKHAND;
	    ikparam[2] = IKELBOWUP;
	    ikparam[3] = (float)MODENONE;
	    float target[XYZ] = {0};
	    target[0] = atof(token[2]);
	    target[1] = atof(token[3]);
	    target[2] = atof(token[4]);
	    IKMotions(gMotor, Name2Num(token[1]), target, ikparam); 
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("init", token[0]) || 'I' == token[0][0]) { // init
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActInitialPose(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (0 == strcmp("stand", token[0]) ) { // stand up
	    char n2[] = "n2";
	    gMotor[Name2Num(n2)].targetangle.data = 0.0; // 首をまっすぐに 
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActStandup(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (0 == strcmp("stand2", token[0]) ) { // stand up another version
//	    gMotor[Name2Num("n2")].targetangle.data = 0.0; // 首をまっすぐに 
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActStandup2(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (0 == strcmp("jump", token[0]) || 'J' == token[0][0]) { // jump
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIMETOTAL] = 10.0; // 動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActJump(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (0 == strcmp("mjda", token[0])) { // move joint degree absolute
	    gMotor[Name2Num(token[1])].ctrlmode = MODENONE;
	    gMotor[Name2Num(token[1])].duration = 1.0;
	    gMotor[Name2Num(token[1])].targetangle.data = D2R(atof(token[2]));
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("mjra", token[0])) { // move joint radian absolute
	    gMotor[Name2Num(token[1])].ctrlmode = MODENONE;
	    gMotor[Name2Num(token[1])].duration = 1.0;
	    gMotor[Name2Num(token[1])].targetangle.data = atof(token[2]);
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("mjdi", token[0])) { // move joint degree increment
	    gMotor[Name2Num(token[1])].ctrlmode = MODENONE;
	    gMotor[Name2Num(token[1])].duration = 1.0;
	    gMotor[Name2Num(token[1])].targetangle.data += D2R(atof(token[2]));
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("mjri", token[0])) { // move joint radian increment
	    gMotor[Name2Num(token[1])].ctrlmode = MODENONE;
	    gMotor[Name2Num(token[1])].duration = 1.0;
	    gMotor[Name2Num(token[1])].targetangle.data += atof(token[2]);
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("pjd", token[0])) { // print joint degree
	    printf("%f\n", R2D(gMotor[Name2Num(token[1])].anglenow.data));
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("pjr", token[0])) { // print joint radian
	    printf("%f\n", gMotor[Name2Num(token[1])].anglenow.data);
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("turn", token[0]) || 'T' == token[0][0]) { // turn
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 0.5;         // 最後の着地の動作時間 (sec)
            act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
	    act_param[AP_ITER] = atoi(token[2]);
            act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
	    if ('l' == token[1][0]) act_param[AP_TURN_DIR] = TURNCCW;
	    else                    act_param[AP_TURN_DIR] = TURNCW;
            act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
		                      act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActTurn(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
	    }
	    return SHELLCONTINUE;
	}
	else if (0 == strcmp("walk", token[0]) || 'W' == token[0][0]) { // walk
	    float act_param[AP_PARAMS] = {0.0};
	    act_param[AP_STATE] = ACT_START;
	    act_param[AP_TIME1] = 0.5;         // 動き１の動作時間 (sec)
	    act_param[AP_TIME2] = 0.5;         // 最後の着地の動作時間 (sec)
            act_param[AP_PHASE] = 4;           // １組の行動に含まれる動きの種類
	    act_param[AP_ITER] = atoi(token[2]);
            act_param[AP_CTRL]  = MODEPROFILE; // 制御のアルゴリズム
	    if ('f' == token[1][0]) act_param[AP_WALK_DIR] = FORWARD;
	    else                    act_param[AP_WALK_DIR] = BACKWARD;
            act_param[AP_TIMETOTAL] = act_param[AP_TIME1] * act_param[AP_PHASE] *
                                      act_param[AP_ITER] + act_param[AP_TIME2]; // 合計動作時間 (sec)
	    for (int i = 0; i < act_param[AP_TIMETOTAL] * 100; i++) { 
		int state = ActWalk(gMotor, act_param);
		usleep(10*1000); // sleep 10 msec, emulates 100hz of intelligence()
            }
	    return SHELLCONTINUE;
	}
	else if (0 == strcmp("rdyna", token[0])) { // read dynamixel data
	    readDyna(token[1]);
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("relax", token[0])) { // relax
	    for (int i = 0; i < NUMALLJOINTS; i++) {
		gMotor[i].ctrlmode = MODEPROFILE;
		gMotor[i].duration = 3.0;
		gMotor[i].targetangle.data = 0.0;
	    }
	    usleep(3*1000*1000); // wait specified sec
	    return SHELLCONTINUE;
        }
	else if (0 == strcmp("angles", token[0]) || 'A' == token[0][0]) { // print angles of joints
	    printf("1.0 %.3f %.3f %.3f %.3f %.3f %.3f ",
	      gMotor[A4J1].anglenow.data, gMotor[A4J2].anglenow.data,
	      gMotor[A4J3].anglenow.data * -1, 
	      gMotor[A1J1].anglenow.data, gMotor[A1J2].anglenow.data,
	      gMotor[A1J3].anglenow.data * -1
	      );
	    printf("%.3f %.3f %.3f %.3f %.3f %.3f ",
	      gMotor[A5J1].anglenow.data, gMotor[A5J2].anglenow.data,
	      gMotor[A5J3].anglenow.data * -1, 
	      gMotor[A2J1].anglenow.data, gMotor[A2J2].anglenow.data,
	      gMotor[A2J3].anglenow.data * -1
	      );
	    printf("%.3f %.3f %.3f %.3f %.3f %.3f\n",
	      gMotor[A6J1].anglenow.data, gMotor[A6J2].anglenow.data,
	      gMotor[A6J3].anglenow.data * -1, 
	      gMotor[A3J1].anglenow.data, gMotor[A3J2].anglenow.data,
	      gMotor[A3J3].anglenow.data * -1
	      );
	    return SHELLCONTINUE;
	}

/////////////////////////////////////////////////////////////////////////////////////////

	else if (YES == UserShell(token)) { // 自分のシェルで独自のコマンドを追加可能
	    NOTHING; // UserShell() を実行した後はここでは何もすることはない
	    return SHELLCONTINUE;
        }
	else {
	    printf("invalid command\n");
	    return SHELLCONTINUE;
	}
}
*/
