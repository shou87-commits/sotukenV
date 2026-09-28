// user.h

#ifndef HEADER_USERH
#define HEADER_USERH

#include "motorclass.h"
#include "imageclass.h"
#include "depthclass.h"
#include "ftclass.h"

// UserMotion() は intelligence() から AIITVL [sec] 毎に呼び出される
// LessonMotion(), SuzukiMotion(), TanakaMotion() の原型
void UserMotion(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[]); 

// action.cpp で定義されている
// 次の関数はユーザが必要に応じて書き換え可能、自分で関数を追加することもできる
// そのときは UserMotion(), 又は自作の ????Motion() に呼び出す記述を追加する
void MyMotion(C1 m[]);
int ActInitialPose(C1 m[], float p[]); // 初期姿勢の例
int ActStandup(C1 m[], float p[]); // ひっくり返ったときに立ち上がる
int ActStandup2(C1 m[], float p[]); // ひっくり返ったときに立ち上がる、その２
int ActJump(C1 m[], float p[]); // ジャンプ
int ActBodyShift(C1 m[], float p[]); // 脚先を固定して本体を移動、平行移動
int ActTilt(C1 m[], float p[]); // 脚先を固定して本体を移動、回転
int ActStay(C1 m[], float p[]); // 停止
int ActRelax(C1 m[], float p[]); // 脱力
int ActLidar(C1 m[], float p[]); // LIDAR で距離を測定する
int ActLidarVScan(C1 m[], float p[]); // LIDAR で縦方向に scan する、未完成
int ActWalk(C1 m[], float p[]); // 歩行
int ActLegs(C1 m[], float p[], float target[NUMARMS][MAXPHASE][XYZ], int onflag[NUMARMS]);
int ActTurn(C1 m[], float p[]); // その場で旋回
int ActNeck(C1 m[], float p[]); // 首を動かす
int ActHatch(C1 m[], float p[]); // 腹下のハッチを開閉する
int ActHand(C1 m[], float p[]); // Hand を開く、閉じる
int ActVision(C1 m[], RGBCam *c, DepthCam *d, float p[]); // カラー画像、距離画像から検出した
                                                          // 物体の3次元ローカル座標を求める
int ActGrabRelease(C1 m[], DepthCam *d, float p[]); // 指定座標の物体を掴む、持上げる、落とす


// 次の関数は目標座標に対して逆運動学を計算して、結果に従い脚を動かす
int IKIncrementMotions(C1 m[], int arm, float target[], float ikparam[]);
int IKMotions(C1 m[], int arm, float target[], float ikparam[]);

// ハンド、首を動かす関数
void Hand(C1 m[], int place, int move, float rad); // 角度を radian で指定する
void Neck(C1 m[], float hrad, float vrad);
void HandDeg(C1 m[], int place, int move, float deg); // 角度を degree で指定する
void NeckDeg(C1 m[], float hdeg, float vdeg);

// 従来の動きの例、UserMotion() から呼び出せば今でも使える
void UserMotion1(C1 m[]); // シンプルな動き
void UserWalkMotion(C1 m[]); // 歩行の例
void Dance(C1 m[]); // ダンスは一度呼出すと動きが終わるまで戻らないので注意

// 練習用の動きを記述する例 action2.cpp
int Act1(C1 m[], float p[]); // 動きの例
int Act2(C1 m[], float p[]);
int Act12(C1 m[], float p[]); // 複数の動きをまとめて指定する例
int ActEX1(C1 m[], float p[]);
int ActEX2(C1 m[], float p[]);

// 個別ユーザによる動作の関数の例
// void SuzukiMotion(C1 m[]);
// void TanakaMotion(C1 m[]);
// void LessonMotion(C1 m[]); // 練習用 lesson.cpp

// void LessonMotion1(C1 m[]); // LessonMotion() から呼び出される
// void LessonMotion2(C1 m[]); // LessonMotion() から呼び出される
// void LessonMotion3(C1 m[]); // LessonMotion() から呼び出される

// その他
int encode_leg(int flag[]); // BodyShift 用のユーティリティ関数
void decode_leg(int val, int flag[]);

#endif // HEADER_USERH
