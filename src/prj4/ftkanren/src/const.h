// const.h

#ifndef	HEADER_CONST
#define	HEADER_CONST

// 次の２行のうち、自分が使わないモデルをコメントアウトする
#define MODEL_6LEG
// #define MODEL_NEXTAGE

//#define ACTEXEC // コメントを外すとAct????() 開始と狩終了時に画面出力する

typedef unsigned char Uchar;

const int TRUE  = 1;
const int FALSE = 0;

const int YES = 1;
const int NO  = 0;

const int ON  = 1;
const int OFF = 0;

const float FTON = 1;
const float FTOFF = 0;

const int ERROR  = 1;
const int NORMAL = 0;

const int NUMARMS = 6; // Arm の総数、首を含まない
const int NUMLINKS = 6; // 1 本の Arm に存在する link の数、ハンドは除く
const int NUMJOINTS = 6; // 各 Arm が備える Joint の数、ハンドは含まない
const int NUMHANDJOINTS = 2; // ハンドが備える Joint の数
const int NUMALLJOINTS = 50; // Joint の総数、手首や首を含む
const int NUMFTSENSORS = 4; //ftsensorの数


const int XYZ = 3;  // ３次元空間の次元数、XYZはこの並び順

const int NUMPARAM = 4; // param[] の配列の要素数
const int IKNUMPARAM = 4; // ikparam[] の配列の要素数

// 次の定数は本来 int でよいが、配列の他要素に float が必要なのでそれに合わせる 
const float IKHAND      = 0.0; // IK 計算時にハンド先端を目標座標に持っていく
const float IKWRIST     = 1.0; // IK 計算時に手首を目標座標に持っていく
const float IKELBOWUP   = 0.0; // IK 計算時に肘を上に曲げる
const float IKELBOWDOWN = 1.0; // IK 計算時に肘を下に曲げる

const int LF = 1; // 脚の位置、Left Front
const int LM = 2; // 脚の位置、Left Middle
const int LB = 3; // 脚の位置、Left Back
const int RF = 4;
const int RM = 5;
const int RB = 6;

const int A1J1 = 0; // id number of joint, 各ジョイントにID番号を付ける。
const int A1J2 = 1; // 連番にすること、読み出した角度がこの順番で配列に入る。
const int A1J3 = 2; // class　の初期化は必ずこの順番で行う。
const int A1J4 = 3; // この値は読み出した角度の配列の添字に使える。
const int A1J5 = 4;
const int A1J6 = 5;
const int A1J7 = 6;
const int A1J8 = 7;

const int A2J1 = 8;
const int A2J2 = 9;
const int A2J3 = 10;
const int A2J4 = 11;
const int A2J5 = 12;
const int A2J6 = 13;
const int A2J7 = 14;
const int A2J8 = 15;

const int A3J1 = 16; // 3番のアームの1番のジョイント
const int A3J2 = 17;
const int A3J3 = 18;
const int A3J4 = 19;
const int A3J5 = 20;
const int A3J6 = 21;
const int A3J7 = 22;
const int A3J8 = 23;

const int A4J1 = 24;
const int A4J2 = 25;
const int A4J3 = 26;
const int A4J4 = 27;
const int A4J5 = 28;
const int A4J6 = 29;
const int A4J7 = 30;
const int A4J8 = 31;

const int A5J1 = 32;
const int A5J2 = 33;
const int A5J3 = 34;
const int A5J4 = 35;
const int A5J5 = 36;
const int A5J6 = 37;
const int A5J7 = 38;
const int A5J8 = 39;

const int A6J1 = 40;
const int A6J2 = 41;
const int A6J3 = 42;
const int A6J4 = 43;
const int A6J5 = 44;
const int A6J6 = 45;
const int A6J7 = 46;
const int A6J8 = 47;

const int A7J1 = 48; // 首、水平回転
const int A7J2 = 49; // 首、垂直回転

const int FTA1J1 = 0; // ft lf1
const int FTA1J2 = 1; // ft lf2
const int FTA4J1 = 2; // ft rf1
const int FTA4J2 = 3; // ft rf2

const int HANDOPEN = 1;
const int HANDCLOSE= 2;

const float RAD30 = M_PI / 6; // 30 度を表す radian
const float RAD45 = M_PI / 4; // 45 度を表す radian
const float RAD60 = M_PI / 3; // 60 度を表す radian
const float RAD90 = M_PI / 2; // 90 度を表す radian
const float RAD180= M_PI;     // 180度を表す radian

const float ANGLEKEEP = -99.0; // 現在の角度を維持することを表す定数

const int MODENONE     = 0; // 制御の種類、シンプルなPID制御
const int MODEPROFILE  = 1; // プロファイルを使った滑らかな動き
const int MODEPROFILE2 = 2; // プロファイルを使った滑らかな動き

const int MOVING = 0; // モータの状態、動作中
const int DONE   = 1; // モータの状態、目標角度に達した

const int PROFSIZE = 1024; // モータの角度指定プロファイルの配列サイズ、10.24秒

const int IDHEADCAM = 1;       // ID of Head Camera
const int IDHANDCAM = 2;       // ID of Hand Camera
const int IDDEPTHCAMCOLOR = 3; // ID of depth camera (color image)
const int IDDEPTHCAMDEPTH = 4; // ID of depth camera (depth value)

const float MotorSenseITVL = 0.01; // モータの角度を読み出す間隔、秒
const float MotorCtrlITVL  = 0.01; // モータの角度を指示する間隔、秒
const float AIITVL = 0.01;         // 行動計画 AI の計算をする間隔、秒

// ここから user.cpp 用の定数

// 以下の定数は内容を理解していなければ変更してはいけない
const int ACT_START    = 0; // 動作開始、返り値と act_param[AP_STATE] の両方で使うので注意
const int ACT_MOVING   = 1; // 動作の継続中
const int ACT_END      = 2; // 動作終了
const int ACT_ERR      = 3; // 何らかのエラー発生

// 以下の定数は個々の行動に割り当てた値で、必要に応じて追加可能
const int ACT_INITPOSE = 0; // 初期姿勢
const int ACT_WALK     = 1; // 歩行
const int ACT_FASTWALK = 2; // 高速歩行
const int ACT_TURN     = 3; // その場で旋回
const int ACT_NECK     = 4; // その場で旋回
const int ACT_BODYSHIFT= 5; // 脚先を固定して本体を移動
const int ACT_SHELL    = 6; // shell() による操作
const int ACT_SHELLSLEEP=7; // shell() による操作のためにロボットを静止

const int ACT_LIDAR    = 26; // LIDAR test 動作
const int ACT_LIDAR2   = 27; // LIDAR test 動作
const int ACT_LIDAR3   = 28; // LIDAR test 動作
const int ACT_LIDARVSCAN=29; // LIDAR で縦方向の SCAN 動作
const int ACT_1        = 30; // 逆運動学の動作の例１
const int ACT_2        = 31; // 逆運動学の動作の例２
const int ACT_12       = 32; // ACT_1, ACT2 の組み合わせ
const int ACT_EX1      = 33; // 関節動作の例１
const int ACT_EX2      = 34; // 関節動作の例２

const int ACT_STAY     = 98;// 現状維持、停止
const int ACT_ERROR    = 99;// エラー処理

// 以下の定数は act_param[] の要素数
const int AP_PARAMS    = 32;// パラメータの受け渡し用配列のサイズ

// 以下の定数は act_param[] 中の個々の値の位置(配列の添字)
const int AP_STATE     = 0; // 必須、動作開始時 ACT_START か、途中 ACT_MOVING か

const int AP_TIMETOTAL = 1; // 必須、動作時間、秒 float
const int AP_TIME1     = 2; // 動き１の時間、秒 float
const int AP_TIME2     = 3; // 動き２の時間、秒 float
const int AP_TIME3     = 4; // 動き３の時間、秒 float
const int AP_TIME4     = 5; // 動き４の時間、秒 float
const int AP_TIME5     = 6; // 動き４の時間、秒 float
const int AP_TIME6     = 7; // 動き４の時間、秒 float

const int AP_CTRL      = 10; // 制御の方法、MODENONE, MODEPROFILE, ...
const int AP_PHASE     = 11; // 動きの種類、歩行と旋回は４種類の動きの組み合わせ
const int AP_ITER      = 12; // 繰り返し回数

const int AP_NECK_HA   = 15; // 首振り水平角度
const int AP_NECK_VA   = 16; // 首振り垂直角度

const int AP_WALK_DIR  = 17; // 歩行で進む向き、FORWARD, BACKWARD
const int AP_TURN_DIR  = 18; // 回転方向、TURNCW 時計回り, TURNCCW 反時計回り

const int AP_LOCALX    = 19; // ローカル座標 X
const int AP_LOCALY    = 20; // ローカル座標 X
const int AP_LOCALZ    = 21; // ローカル座標 X

const int AP_OBJFIND   = 22; // Vision で物体を見つけたか否か

const int AP_BSHIFTX    = 23; // シフト距離 X
const int AP_BSHIFTY    = 24; // シフト距離 Y
const int AP_BSHIFTZ    = 25; // シフト距離 Z
const int AP_BSHIFTFLAG = 26;

const int AP_LIDAR_VUP   = 27; // LIDAR VSCAN の上の角度
const int AP_LIDAR_VDOWN = 28; // LIDAR VSCAN の下の角度

const int AP_FTID		= 29; //FTセンサの番号
const int AP_FTHOLD		= 30; //掴んだか否か
// その他、act_param[] 関連の定数

const int FORWARD      = 1; // 前進
const int BACKWARD     =-1; // 後退
const int TURNCW       =-1; // 時計回り
const int TURNCCW      = 1; // 反時計回り

// 色を表す定数

const int RED      = 1;
const int GREEN    = 2;
const int BLUE     = 3;
const int YELLOW   = 4;
const int CYAN     = 5;
const int PURPLE   = 6;
const int WHITE    = 7;
const int DURK     = 8;


#endif // HEADER_CONST
