汎用で利用可能な何らかの動きを実現する関数

motion.cpp

void HandDeg(C1 m[], int place, int move, float deg);
角度(degree)を指定してハンドを開閉する

void Hand(C1 m[], int place, int move, float rad);
角度(radian)を指定してハンドを開閉する

void NeckDeg(C1 m[], float hdeg, float vdeg);
角度(degree)を指定して首を曲げる

void Neck(C1 m[], float hrad, float vrad);
角度(radian)を指定して首を曲げる

action.cpp, action2.cpp

int ActInitialPose(C1 m[], float p[]);
初期姿勢、立ち上がる

int ActStay(C1 m[], float p[]);
停止、現在の姿勢で静止する

int ActLidar(C1 m[], float p[]);
LIDAR センサで対象物までの距離を測定する

int ActWalk(C1 m[], float p[]);
前進または後退歩行する

int ActTurn(C1 m[], float p[]);
その場で旋回する、回転方向を指定可能

int ActNeck(C1 m[], float p[]);
首を動かす、水平垂直方向の回転角度を指定可能

int ActBodyShift(C1 m[], float p[]);
指定した脚先を固定して本体を動かす

int ActVision(C1 m[], RGBCam *c, DepthCam *d, float p[]);
画像中の物体を探してローカル座標を求める

int ActGrabRelease(C1 m[], DepthCam *d, float p[]);
座標を指定してそこにある物体を掴んで持ち上げる
