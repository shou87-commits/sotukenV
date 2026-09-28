//fttest.cpp
//ftセンサの値を確かめるためのプログラム

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

#include <geometry_msgs/WrenchStamped.h> //いれわすれに注意

// 様々な定数の設定は const.h に移動した
#include "const.h"
#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h"
#include "lidarclass.h"
#include "ftclass.h"

// user.h にはこれまでに作成した Act?????(); のプロトタイプ宣言が
// 記述されている。このヘッダを include することで、それらの関数を
// 利用することができる。
#include "user.h"

// LIDAR 測距センサのデータにアクセスするためのポインタ
// LIDAR を使わなければ消せる
extern LIDAR *gLIDAR;
extern FTSENSOR *gFT;

// 自分が作った動作を表す数値、const.h 内の他の数値と被らないか要確認
const int ACT_FTTEST = 101;
// const int ACT_FTTEST2 = 102; // 必要なら何個でも定義できる
// const int ACT_FTTEST3 = 103;
const int ACT_FTTESTVISION = 104;
const int ACT_FTTESTGRAB = 105;

// 自分が作った独自の動きを定義する関数のプロトタイプ宣言、
// 他のファイルから参照しないならここに(だけ)書けばよい
int ActFttest(C1 m[], float p[]);
// int ActFttest2(C1 m[], float p[]); // 必要なら何個でも作れる
// int ActFttest3(C1 m[], float p[]);
int ActFttestVision(C1 m[], RGBCam *c, DepthCam *d, float p[]);
int ActFttestGrab(C1 m[], float p[]);
int FttestVFindColorObj(RGBCam *c, DepthCam *d, int *x, int *y, int color);
int FttestVSobel(RGBCam *c, DepthCam *d, int *x, int *y);
int FttestVLaplacian(RGBCam *c, DepthCam *d, int *x, int *y);

// FttestMotion() のプロトタイプ宣言は user.cpp に記述するか、
// または user.hに存在している必要あり

// 以下の変数の実体は user.cpp にある

extern int act_num;       // 次の行動を決める値、重要
extern int act_last;      // 直前の行動を覚えておく
extern int state;         // 動作の状態を表す、開始、継続中、終了
extern float act_param[]; // パラメータの受け渡し用配列

int flag=0;

void FttestMotion(C1 m[], RGBCam *c, DepthCam *d)
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
		act_param[AP_NECK_VA] = -0.2;
	    act_num = ACT_NECK;
	}
	if (ACT_BODYSHIFT == act_last) {
		act_param[AP_NECK_VA] = -0.2;
	    act_num = ACT_NECK;
	}
	if (ACT_NECK == act_last) act_num = ACT_FTTESTVISION;
	if (ACT_FTTESTVISION == act_last) {
	    if (YES == act_param[AP_OBJFIND]) {
		act_param[AP_TIMETOTAL] = 19;
		act_num = ACT_FTTEST; // 何かものを見つけたら掴む(FT)
	    }
	    else {
		act_param[AP_TIMETOTAL] = 1;
		act_num = ACT_STAY;       // 無ければおやすみ
	    }
	}
	if (ACT_FTTEST == act_last) {
		if(act_param[AP_FTHOLD] == FTON){
        	act_param[AP_TIMETOTAL] = 10; //手に当たっていれば掴む
        	act_num = ACT_FTTESTGRAB;
		}
		else{
			act_param[AP_TIMETOTAL] = 1; //手に当たっていなければ見るところから
        	act_num = ACT_FTTESTVISION;
		}
    }
	if (ACT_FTTESTGRAB == act_last) {
		act_param[AP_TIMETOTAL] = 9; //また見る動きに戻る
		flag+=1;
		act_num = ACT_FTTESTVISION;
	}
	if (ACT_STAY == act_last) act_num = ACT_FTTESTVISION;
	if (ACT_SHELL == act_last) {
//	    act_num = ACT_FTTESTVISION; // 再開する動作は shell() で決める、ここでは不要
	}
	if (ACT_SHELLSLEEP == act_last) {
	    act_num = ACT_SHELLSLEEP;
	}
// もともと user.cpp dで定義されていた関数は action.cpp, action2.cpp に移された。
// 次行のようにaction.cpp, action2.cpp にある関数はこれまでと同様に利用可能
//	if (ACT_WALK == act_last) act_num = ACT_TURN;
//	if (ACT_TURN == act_last) act_num = ACT_WALK;
	if (ACT_ERROR == act_last) act_num = ACT_INITPOSE; // エラー時の次の行動を決める
    }
    if (ACT_ERR == state) act_num = ACT_ERROR; // 状態がエラーの時はエラー処理へ

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
	    act_param[AP_TIMETOTAL] = 5.0; // 動作時間 (sec)
	    state = ActInitialPose(m, act_param);
	    break;
	case ACT_FTTESTVISION : // 画像処理
	    act_param[AP_TIMETOTAL] = 0.1; // 動作時間 (sec)
	    state = ActFttestVision(m, c, d, act_param);
	    break;
	case ACT_FTTESTGRAB : // ものをつかむ
	    act_param[AP_TIMETOTAL] = 9.0; // 動作時間 (sec)
	    act_param[AP_TIME1] = 4.0; // 動作時間 (sec)
		act_param[AP_TIME2] = 5.0;
	    state = ActFttestGrab(m, act_param);
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
	case ACT_FTTEST : // 動きの例、前の腕を交互に動かす
	    act_param[AP_TIMETOTAL] = 19.0; // 動作時間 (sec)
	    act_param[AP_TIME1] = 3.0; // 動作時間 (sec)
	    act_param[AP_TIME2] = 5.0; // 動作時間 (sec)
	    act_param[AP_TIME3] = 10.0; // 動作時間 (sec)
	    state = ActFttest(m, act_param);
	    break;
        case ACT_STAY : // 現状維持
//	    act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
	    state = ActStay(m, act_param);
	    break;
        case ACT_SHELL : // shell() による操作のためにここでは何もしない
	    act_param[AP_STATE] = ACT_MOVING;
	    state = ACT_MOVING;
	    break;
        case ACT_SHELLSLEEP : // shell() による操作のためにロボットの動きを静止させて待つ
	    act_param[AP_TIMETOTAL] = 60.0; // 合計動作時間 (sec)
	    state = ActStay(m, act_param);
	    break;
        case ACT_ERROR : // エラーが発生したとき
        default : // どの case にもマッチしないとき、本来ここにはマッチしないはず
	    fprintf(stderr, "Error: Bad act_num specified, or other error occured.\n");
	    state = ACT_END;
            break;
    }
}

int ActFttestVision(C1 m[], RGBCam *c, DepthCam *d, float p[]) // 
{
    static int timer = 0;
    static int count = 0;
    static float ikparam[IKNUMPARAM];
    char fname[64];
    float duration = p[AP_TIMETOTAL]; 
    int cycletotal = (int)(duration*100);

// 動作の指示はここ★★ ★から ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
#ifdef ACTEXEC
	fprintf(stderr, "start ActFttestVision()\n");
#endif
        timer = 0;
    }
    if (0 == (timer % cycletotal)) {
	int ix = 319, iy = 239;   // 画像の注目座標、最初は適当な初期値なので意味なし
	float x, y, z;            // 計算後のローカル座標を記憶する変数
	float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要
	float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要

	Hand(m, LF, HANDOPEN, RAD60);//開く

//	float ha = m[A7J1].targetangle.data; // 目標角度
//	float va = m[A7J2].targetangle.data; // 目標角度
//	float ha = p[AP_NECK_HA]; // 首の水平回転角度(radian)、正しくセットされてない可能性がある
//	float va = p[AP_NECK_VA]; // 首の垂直回転角度(radian)、正しくセットされてない可能性がある

//////////// 画像処理の例 /////////////////////////////////
// 画像処理をして、必要なら更に自分で画像を調べて、注目座標を ix, iy にセットする
	
//	if (0 < FttestVSobel(c, d, &ix, &iy)) {            // Sobel フィルタ、輪郭を求める
//	if (0 < FttestVLaplacian(c, d, &ix, &iy)) {        // Laplacian フィルタ、輪郭を求める

	int color;
	if (flag %2==0) color = RED;
	else color = BLUE;
	if (0 < FttestVFindColorObj(c, d, &ix, &iy, RED)) { // 指定色の物体を見つける
	    
// カメラ画像の window で注目する画素の位置に + マークを描画する、不要なら実行しなくて可
	    int a[2] = {ix, iy};
	    c -> SetMark(1, a);
//	    c -> ResetMark(); // 登録したマークを削除して描画をやめるとき

// ix, iy から３次元のローカル座標を求める、x, y, z はアームを動かす目標座標などで使える
	    d -> LocalXyz(ix, iy, &x, &y, &z, ha, va);
	    p[AP_LOCALX] = x; // p[] に入れて計算結果を呼び出し元に返す
	    p[AP_LOCALY] = y;
	    p[AP_LOCALZ] = z;
	    p[AP_OBJFIND] = YES; // 物体発見のフラグ、見つけたか否かで次の動作を変えられる
	}
	else {
	    p[AP_LOCALX] = 99;
	    p[AP_LOCALY] = 99;
	    p[AP_LOCALZ] = 99;
	    p[AP_OBJFIND] = NO;  // 物体を見つけなかった
	}
// 以下を実行すると画像をファイルに書き出す、不要なら実行しなくて可
//	int a[] = {319, 239, 319, 357, 198, 239, 198, 357, 190, 116}; // ５個の座標
//	sprintf(fname, "rgb%d.ppm", count++); // ファイル名の生成
//	c -> RGBsavemark(fname, 5, a); // マーク付きでRGB画像を保存
//	sprintf(fname, "d%d.pgm", count++); // ファイル名の生成
//	d -> Dsave(fname); // depth画像を保存
    }
// ここ★★★ までの間に書く //////////////////////////////////

    timer++;
    if (timer < cycletotal) { // 動作継続中の処理
//  if (timer <= cycletotal) { // これは間違い、１回多く実行される
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // 動作終了、タイマをクリアして ACT_END を返り値に
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC
	fprintf(stderr, "end ActFttestVision()\n");
#endif
	return ACT_END;
    }
}

int ActFttestGrab(C1 m[], float p[]) // 物体を掴む、持ち上げる、落とす
{
    static int timer  = 0; // 関数が呼び出された回数をカウントする
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
    float time1 = p[AP_TIME1]; // 動き１の時間、秒
    int cycletotal = (int)(duration*100); // 動作終了までに関数が呼び出される回数

    float x, y, z; // 掴む対象となる物体の座標
	float avex[2];
	float avey[2];
	float avez[2]; 		//[0]=ftセンサの平均値 [1]=ONかOFFか
	float ft[3][500]; 	//[3]=xyz, [500]=5秒間
	int a; 				//ONかOFFか
    
    x = p[AP_LOCALX]; // 掴む対象となる物体の座標、p[]経由で受け取る
    y = p[AP_LOCALY];
    z = p[AP_LOCALZ];
    
// 動作の指示はここから ////////////////////////////////

// 初期設定は最初に１回
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { 
#ifdef ACTEXEC
	fprintf(stderr, "start ActGrab()\n");
#endif
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
	m[A1J7].ctrlmode = m[A1J8].ctrlmode = MODENONE;
	m[A1J7].duration = m[A1J8].duration = time1;
	m[A4J7].ctrlmode = m[A4J8].ctrlmode = MODENONE;
	m[A4J7].duration = m[A4J8].duration = time1;
        timer = 0;
    }
// 掴む
    if (0 == (timer % cycletotal)) {
		ikparam[0] = time1; // 動きによって時間が異なる場合
    	Hand(m, 0.0<y?LF:RF, HANDCLOSE, 0); // ハンドを閉じる
		p[AP_FTHOLD] = FTOFF; //OFFを入れておく
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
	fprintf(stderr, "end ActGrab()\n");
#endif
	return ACT_END;
    }
}

int ActFttest(C1 m[], float p[]) // 3種類の異なる動きを組み合わせる例
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

	float x, y, z; // 掴む対象となる物体の座標
	float decix[2]={0,0};
    float deciy[2]={0,0};      
    float deciz[2]={0,0};      //[0]=ftセンサの平均値 [1]=ONかOFFか
    float ft[3][200];   //[3]=xyz, [300]=3秒間
	float maxx[1],maxy[1],maxz[1];
	float avex[1],avey[1],avez[1];
    int a;

	x = p[AP_LOCALX]; // 掴む対象となる物体の座標、p[]経由で受け取る
    y = p[AP_LOCALY];
    z = p[AP_LOCALZ];

// 動作の指示はここから ////////////////////////////////

// 複数の動きを順番に行わせるときの書き方
// 初期設定は最初に１回
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { 
#ifdef ACTEXEC
	fprintf(stderr, "start ActFttest()\n");
#endif
    ikparam[1] = IKHAND;
    ikparam[2] = IKELBOWUP;
    ikparam[3] = (float)MODEPROFILE;
    m[A1J7].ctrlmode = m[A1J8].ctrlmode = MODENONE;
    m[A1J7].duration = m[A1J8].duration = time1;
    m[A4J7].ctrlmode = m[A4J8].ctrlmode = MODENONE;
    m[A4J7].duration = m[A4J8].duration = time1;
        timer = 0;

    }
// １番目の動き、物体の上にハンドを移動させる
    if (0 == (timer % cycletotal)) {
        float zz = z + 0.25, target1[XYZ] = {x, y, zz};
        ikparam[0] = time1;
        IKMotions(m, 0.0<y?LF:RF, target1, ikparam); // アーム、物体の 250mm 上に
        Hand(m, 0.0<y?LF:RF, HANDOPEN, 1.57); // ハンドを開く
        float targetrest[XYZ] = {0.6, 0.2, 0.3};
        targetrest[1] *= 0.0<y?-1:1;
        IKMotions(m, 0.0<y?RF:LF, targetrest, ikparam); // 使わないアーム、おやすみ
    }

// ２番目の動き、腕を下ろす
    if (cycle1 == (timer % cycletotal)) {
        ikparam[0] = time1; // 動きによって時間が異なる場合
        float xx = x + 0.1, yy = y * 1.3, zz = z - 0.03, target1[XYZ] = {xx, yy, zz};
        // 物体が左右どちらにあるかで動かす腕を変える
        IKMotions(m, 0.0<y?LF:RF, target1, ikparam); // 物体の 50mm 上
    }
//ここでhandにあたっているかtime3秒停止してデータをとる
//この値は現在3秒　変える場合はftclass.hのdefine BYOU 3　の3を変えること
	if (cycle1*2 == (timer % cycletotal)){
		ikparam[0] = time3;
		printf("計測中\n");
	}
// 3番目、FT
    if (cycle1*2 + cycle3 == (timer % cycletotal)) { // cycle1 回呼び出されたら次の動きに移る
        ikparam[0] = time1;
		int id= (0.0<y)?FTA1J2:FTA4J2; //掴む方の手のIDをFTIDに代入
		p[AP_FTID]=id;
		//gFT[id].Ftyomu(ft);
		//printf("cpp id=%p tail=%p f=%p \n",&(gFT[id].id), &(gFT[id].tail), gFT[id].f);

		/*
		gFT[id].Ftmax(maxx, maxy, maxz); //最大値
		gFT[id].Ftmax(avex, avey, avez); //平均
        fprintf(stderr,"prg4 max x:%.3f y:%.3f z:%.3f \n", maxx[0],maxy[0],maxz[0]);
		fprintf(stderr,"prg4 deci x:%.3f y:%.3f z:%.3f \n", avex[0],avey[0],avez[0]);
		*/

		a = gFT[id].Ftdecision(decix, deciy, deciz); //関数をFTIDで指定、配列を渡す
        gFT[id].Ftyomu(ft); //ftclass.h内のFtyomuの中身を行う　現在は特に何もしてない
        //fprintf(stderr, "ON(1)OFF(0):%d \n x:%f y:%f z:%f \n", a, avex[0], avey[0], avez[0]); //ave[0]=平均値 ave[1]=ONOFF
		fprintf(stderr, "ON(1)OFF(0):%d \n", a); //ave[0]=平均値 ave[1]=ONOFF

		p[AP_FTHOLD] = a; //ONかOFFか入れる
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
	fprintf(stderr, "end ActFttest()\n");
#endif
	return ACT_END;
    }
}

int FttestVFindColorObj(RGBCam *c, DepthCam *d, int *x, int *y, int color)
// 指定した色 color の物体(画素の集まり)を見つけて、重心座標を *x, *y で返す。
// 返り値は見つけた画素の個数、1以上なら見つかった、0 なら見つからなかった。
// 画像中の物体の個数は１個を前提としている。
// ２個以上の場合は正しく動作しないので、別の機能を実装する必要がある。
{
	cv_bridge::CvImagePtr p = c -> cv_ptr;

// 指定した色の画素を抽出する処理の例
	int sumx = 0, sumy = 0, sum  = 0; // 見つけた画素の座標と個数
	cv::Mat img(p -> image.rows, p -> image.cols, CV_8UC1); // 結果を画像として保存する場所
	for (int y = 0; y < p -> image.rows; y++) { // 全画素を調べる二重ループの例
	    for (int x = 0; x < p -> image.cols; x++) {
		Uchar b = p -> image.at<cv::Vec3b>(y, x)[0];
		Uchar g = p -> image.at<cv::Vec3b>(y, x)[1];
		Uchar r = p -> image.at<cv::Vec3b>(y, x)[2];
		if (YES == c -> TestColor(r, g, b, color)) { // 色の条件に合ったら
		    sumx += x; sumy += y; sum++;
		    img.at<Uchar>(y, x) = 255;
		}
		else {
		    img.at<Uchar>(y, x) = 0;
		}
	    }
	}
	if (0 < sum) { // 指定した色の画素が見つかったとき
	    *x = sumx/sum;
	    *y = sumy/sum;
//	    fprintf(stderr, "(%d,%d) ", *x, *y);
//	    imwrite("/tmp/detect.pbm", img); // 検出結果の画像、記録して見る必要なければ実行しない
	}
	else { // 見つからなかったとき
	    *x = 0; 
	    *y = 0;
	}
	cv::imshow("ColorObj", img); // 検出結果を画像として表示してみる
	cv::waitKey(5); // 5ms 待つ、消してはいけない

	return sum; // 見つかった画素の個数を返り値としている
}

int FttestVSobel(RGBCam *c, DepthCam *d, int *x, int *y)
// 画像処理の例、Sobel フィルタで輪郭を求める。
// 次の動作で必要となる何らかの座標を求めて *x, *y で返す。
// 処理結果を表す何らかの値を返り値とする。
{
    cv_bridge::CvImagePtr p = c -> cv_ptr; // 原画像の構造体へのポインタ

    cv::Mat input = p -> image; // カメラで撮影した原画像
    cv::Mat output, out2; // 配列の大きさは指定しなくてもよい、自動で決まる
//  cv::Mat img(p -> image.rows, p -> image.cols, CV_8UC1); // 大きさを指定して構造体を確保

// 深度画像 d は今のところ活用していないが、必要なら利用できる
    
// 輪郭を抽出する処理の例
// Sobelフィルタ(入力, 出力, 出力タイプ, x方向の微分次数, y方向の微分次数, フィルタサイズ)
    cv::Sobel(input, output, CV_32F, 1, 1, 3);          // x, y 両方向の微分フィルタ
//  cv::Sobel(input, output, CV_32F, 0, 1, 3);          // 参考：y方向のみの微分フィルタ
//  convertScaleAbs（＝スケーリング後に絶対値を計算し，結果を8ビットに変換）
    cv::convertScaleAbs(output, out2, 1, 0);

// 処理結果 out2 の全画素を調べる二重ループの例
    int sum = 0;
    for (int y = 0; y < out2. rows; y++) {
	for (int x = 0; x < out2. cols; x++) {
	    float b = out2. at<cv::Vec3b>(y, x)[0];
	    float g = out2. at<cv::Vec3b>(y, x)[1];
	    float r = out2. at<cv::Vec3b>(y, x)[2];
	    if (300 < (r + g + b)) { // 指定した条件に合ったら特定の処理をする
		sum++; // とりあえず数えてみた
	    }
	}
    }
    fprintf(stderr, "%d ", sum);
    
    cv::imshow("Sobel", out2); // 検出結果を画像として表示する
    cv::waitKey(5); // 5ms 待つ、消してはいけない

// 閾値以上の場合にエッジ（＝白）と見做して値変換 (入力, 出力, 閾値, 最大値, 閾値タイプ)
    cv::threshold(out2, out2, 64, 255, cv::THRESH_BINARY);
    cv::imshow("SobelBin", out2); // 検出結果を画像として表示する
    cv::waitKey(5); // 5ms 待つ、消してはいけない

    *x = 0; // 必要なら何らかの座標を呼び出し元に返す
    *y = 0;

    return 0; // 何らかの処理結果の値を返り値とする
}

int FttestVLaplacian(RGBCam *c, DepthCam *d, int *x, int *y)
// 画像処理の例、Laplacian フィルタで輪郭を求める。
{
    cv_bridge::CvImagePtr p = c -> cv_ptr; // 原画像の構造体へのポインタ

    cv::Mat input = p -> image; // カメラで撮影した原画像
    cv::Mat output, out2; // 配列の大きさは指定しなくてもよい、自動で決まる

    cv::Laplacian(input, output, CV_32F, 1, 1, 3); // Laplacian フィルタ、輪郭

    cv::convertScaleAbs(output, out2, 1, 0);
// 処理結果 out2 の全画素を調べる二重ループの例
/*
    int sum = 0;
    for (int y = 0; y < out2. rows; y++) {
	for (int x = 0; x < out2. cols; x++) {
	    float b = out2. at<cv::Vec3b>(y, x)[0];
	    float g = out2. at<cv::Vec3b>(y, x)[1];
	    float r = out2. at<cv::Vec3b>(y, x)[2];
	    if (300 < (r + g + b)) { // 指定した条件に合ったら特定の処理をする
		sum++;
	    }
	}
    }
    fprintf(stderr, "%d ", sum);
*/    
    cv::imshow("Lapracian", out2); // 処理結果を画像として表示する
    cv::waitKey(5); // 5ms 待つ、消してはいけない

    cv::threshold(out2, out2, 32, 255, cv::THRESH_BINARY);
    cv::imshow("LapracianBin", out2); // 処理結果を画像として表示する
    cv::waitKey(5); // 5ms 待つ、消してはいけない

    *x = 0; // 必要なら何らかの座標を呼び出し元に返す
    *y = 0;

    return 0; // 何らかの処理結果の値を返り値とする
}
