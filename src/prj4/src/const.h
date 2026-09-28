// const.h

#ifndef	HEADER_CONST
#define	HEADER_CONST

// 次の２行のうち、自分が使わないモデルをコメントアウトする
#define MODEL_6LEG
// #define MODEL_NEXTAGE

// #define ACTEXEC // コメントを外すと Act????() 開始と終了時に画面出力する
#define CALLMSG // コメントを外すと Call????() 開始時に画面出力する

#define NOTHING // 何もすることがないことを表す

typedef unsigned char Uchar;

const int TRUE  = 1;
const int FALSE = 0;

const int YES = 1;
const int NO  = 0;
const int OK = YES;

const int ON  = 1;
const int OFF = 0;

const float FTON = 1;
const float FTOFF = 0;

const int ERROR  = 1;
const int NORMAL = 0;

const int STRBUF = 32; // 短い文字列を入れる配列の大きさ
const int EOLV = -99;  // end of list val, 可変長リストの終端を表す値

// 制御の周期など、ITVL は CYCLE の逆数

const float MotorSenseCYCLE = 100; // モータの角度を読み出す頻度/秒 config/prg4.yaml で決まる
const float MotorCtrlCYCLE  = 100; // モータの角度を指示する頻度/秒
const float AICYCLE = 100;         // 行動計画 AI の計算をする頻度/秒
const float MotorSenseITVL = 1.0/MotorSenseCYCLE; // モータの角度を読み出す間隔、秒
const float MotorCtrlITVL  = 1.0/MotorCtrlCYCLE;  // モータの角度を指示する間隔、秒
const float AIITVL = 1.0/AICYCLE;  // 行動計画 AI の計算をする間隔、秒

// 以下はRGBカメラとDepthカメラの画像取得周期
// Depthカメラはカラー画像と深度画像の両方を出力する
// 設定は URDF ファイルの以下の記述で決まる
// <sensor type="camera" name="camera1">
//    <update_rate>30.0</update_rate>
//  <sensor type="depth" name="depthcamera1">
//    <update_rate>30.0</update_rate>
// Depthカメラは update_rate を設定しないと 100 になるようだ

const float CAMERACYCLE = 30; // <update_rate>30.0</update_rate>
const float DEPTHCYCLE  = 30; // <update_rate>30.0</update_rate>

const int NUMARMS = 6; // Arm の総数、首を含まない
const int NUMLEGS = NUMARMS; // 脚の総数、首を含まない
const int NUMLINKS = 6; // 1 本の Arm に存在する link の数、ハンドは除く
const int NUMJOINTS = 6; // 各 Arm が備える Joint の数、ハンドは含まない
const int NUMHANDJOINTS = 2; // ハンドが備える Joint の数
const int NUMALLJOINTS = 52; // Joint の総数、手首や首、ハッチを含む、ftsensor は動かさないので除く
const int NUMARMFT = 4; // Arm の停止を判定するFTセンサの最大個数、必要なら増やしてよい
const int NUMMOTORFT = 4; // Motor の停止を判定するFTセンサの最大個数、必要なら増やしてよい

const int NUMRGBCAMS = 3;   // カラー画像カメラの数
const int NUMDEPTHCAMS = 3; // 深度カメラの数
const int NUMFTSENSORS = 11; // ftsensorの数

const int IDHEADCAM = 1;       // ID of Head Camera 202312月は未使用
const int IDHANDCAM = 2;       // ID of Hand Camera 202312月は未使用 

const int IDDEPTHCAMCOLOR   = 0; // ID of depth camera (color image)
const int IDDEPTHCAMCOLORSL = 1; // ID of depth camera side left (color image)
const int IDDEPTHCAMCOLORSR = 2; // ID of depth camera side right (color image)

const int IDDEPTHCAMDEPTH   = 0; // ID of depth camera (depth value)
const int IDDEPTHCAMDEPTHSL = 1; // ID of depth camera side left (depth value)
const int IDDEPTHCAMDEPTHSR = 2; // ID of depth camera side right (depth value)

// 以下は画像処理と座標関係の定数
const int XY   = 2; // 画像データの次元数 (==２次元)
const int XYZ  = 3; // ３次元空間の次元数、XYZはこの並び順
const int XYZW = 4; // ４元数 Quaternion の次元数、XYZWはこの並び順
const int RPY  = 3; // ロール、ピッチ、ヨーの次元数、RPYはこの並び順

const int X    = 0; // 配列の添字として使う時の値
const int Y    = 1; // 配列の添字として使う時の値
const int Z    = 2; // 配列の添字として使う時の値
const int W    = 3; // 配列の添字として使う時の値
const int R    = 0; // 配列の添字として使う時の値
const int P    = 1; // 配列の添字として使う時の値
const int YAW  = 2; // 配列の添字として使う時の値

const int LURL = 2; // 左上右下角の座標 Left Upper, Right Lower corners
const int MAXOBJS = 256; // 画像処理で見つけた物体の扱える最大個数、必要なら増やせる
                         // *** stack smashing detected ***: terminated 中止 (コアダンプ)
                         // のエラーが出るときはこの値が原因かも。大きくしてみる

const int NUMPARAM = 4; // param[] の配列の要素数
const int IKNUMPARAM = 4; // ikparam[] の配列の要素数
const int MAXPHASE = 4; // 連続した動きを指定する時の位相の最大個数

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

// 関節の名前と通し番号、値を変更してはいけない

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

const int A3J1 = 16; // 3番の脚またはアーム(左後)の1番のジョイント
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

const int A8J1 = 50; // 腹下のハッチ左
const int A8J2 = 51; // 腹下のハッチ右、520個分で最後の値は51

/////////////////////// ARM

// NUMLINK, NUMJOINT が既に定義済み
//const int NUMLINKSARM = 6;  // arm １本あたりのリンク数
//const int NUMJOINTSARM = 6; // arm １本あたりのジョイント数

/////////////////////// FT sensor

const int FTA1J1 = 0; // ft lf1 finger1 力覚、トルクセンサ
const int FTA1J2 = 1; // ft lf1 finger2
const int FTA4J1 = 2; // ft rf1 finger1
const int FTA4J2 = 3; // ft rf1 finger2
const int FT5    = 4; // ft 5 背中
const int FTA1S1 = 5; // ft sensor1 lf1
const int FTA2S1 = 6; // ft sensor1 lf1
const int FTA3S1 = 7; // ft sensor1 rf1
const int FTA4S1 = 8; // ft sensor1 lf1
const int FTA5S1 = 9; // ft sensor1 lf1
const int FTA6S1 = 10; // ft sensor1 lf1

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

// ここから user.cpp 用の定数

// 以下の状態を表す定数は内容を理解していなければ変更してはいけない
// 以下の4個の定数は intelligence() から実行される動作や処理の進み具合を示す
// 特に重要な概念を表すので理解しておくことが必要

const int MAX_ACTS     = 1024; // ACT_??? の上限値、配列のサイズに一致、必要なら大きくする

const int ACT_START    = 0; // 動作開始、返り値と act_param[AP_STATE] の両方で使うので注意
const int ACT_MOVING   = 1; // 動作の継続中
const int ACT_END      = 2; // 動作終了
const int ACT_ERR      = 3; // 何らかのエラー発生
const int ACT_NONE     = 4; // 動作が何もない状態 Call????() 用に追加

// 以下の定数は個々の行動に割り当てた値で、必要に応じて追加可能
// 次に実行する動作や処理をこれらの値で指定する
const int ACT_INITPOSE = 0; // 初期姿勢
const int ACT_WALKP1   = 1; // 通常歩行
const int ACT_WALKP2   = 2; // 高速歩行
const int ACT_WALKP3   = 3; // 精密歩行
const int ACT_WALKP4   = 4; // 通常歩行、後進

const int ACT_BODYSHIFT= 5; // 脚先を固定して本体を移動
const int ACT_SHELL    = 6; // shell() による操作
const int ACT_SHELLSLEEP=7; // shell() による操作のためにロボットを静止
const int ACT_SLEEP  = 8;
const int ACT_RESUME = 9;
const int ACT_QUIT   = 10;
const int ACT_HELP   = 11;
const int ACT_DANCE  = 12;
const int ACT_IK     = 13;
// const int ACT_INIT   = 14; // 正しくは ACT_INITPOSE 
const int ACT_STAND  = 15;
const int ACT_STAND2 = 16;
const int ACT_JUMP   = 17;
const int ACT_MJDA   = 18; // このあたりの値は shell() から呼び出すため
const int ACT_MJRA   = 19;
const int ACT_MJDI   = 20;
const int ACT_MJRI   = 21;
const int ACT_PJD    = 22;
const int ACT_PJR    = 23;
const int ACT_RDYNA  = 24;
const int ACT_RELAX  = 25;
const int ACT_ANGLES = 26; // 実機用の角度出力
const int ACT_ALLANGLES = 27;
const int ACT_HAND   = 28; // Hand を開く、閉じる
const int ACT_STAY   = 29; // 状態ではなく動きとしての現状維持、静止

const int ACT_VISION = 30; // カメラ画像を処理する
const int ACT_VISIONFRONT = 31; // カメラ画像を処理する
const int ACT_VISIONLEFT = 32; // カメラ画像を処理する
const int ACT_VISIONRIGHT = 33; // カメラ画像を処理する
const int ACT_VISION3CAM = 34; // カメラ画像を処理する
const int ACT_VISIONDEMO = 35; // カメラ画像を処理するデモ
const int ACT_GRABRELEASE = 36; // モノを掴んで移動させて落とす
const int ACT_FTDEMO = 37; // FTセンサのデータにアクセスするデモ
const int ACT_BRANCH = 38; // 条件分岐
const int ACT_COMPLEX = ACT_BRANCH; // 以前の名前
const int ACT_TILT   = 39; // ボディを回転させる

const int ACT_NVFOC = 40; // カメラ画像を処理する NVFindObjsColor
const int ACT_NVFOD = 41; // カメラ画像を処理する NVFindObjsDepth

const int ACT_NECK  = 42; // 首関節の上下左右の回転
const int ACT_TURNP1  = 43; // 旋回、時計回り
const int ACT_TURNP2  = 44; // 旋回、反時計回り
const int ACT_TURNP3  = 45; // 精密旋回、時計回り
const int ACT_TURNP4  = 46; // 精密旋回、反時計回り

const int ACT_LEGS     = 47; // 脚の任意の動き
const int ACT_LEGSP1   = 48; // パラメータ設定

const int ACT_SETROBOTPOS = 49; // ロボットの位置設定
const int ACT_GETROBOTPOS = 50; // ロボットの位置読み出し

const int ACT_HATCH  = 51; // 腹下ハッチの開閉
const int ACT_HATCHOPEN  = 52; // 腹下ハッチの開閉
const int ACT_HATCHCLOSE = 53; // 腹下ハッチの開閉

// const int ACT_ERROR  = 99;// エラー処理、ACT_ERR に統一する

// 以下は最近使っていないので大きな値の領域に移動した

const int ACT_LIDAR    = 101; // LIDAR test 動作
const int ACT_LIDAR2   = 102; // LIDAR test 動作
const int ACT_LIDAR3   = 103; // LIDAR test 動作
const int ACT_LIDARVSCAN=104; // LIDAR で縦方向の SCAN 動作
const int ACT_1        = 105; // 逆運動学の動作の例１
const int ACT_2        = 106; // 逆運動学の動作の例２
const int ACT_12       = 107; // ACT_1, ACT2 の組み合わせ
const int ACT_EX1      = 108; // 関節動作の例１
const int ACT_EX2      = 109; // 関節動作の例２

// ACT_???? はあまり使わない機能は 100 番台、
// suzuki.cpp では 200 番台、tanaka.cpp では 300 番台、
// lesson.cpp では 400 番台を割り当てることにする。
// 個人で作った動作は 500 番台以降にすると他の値と被りにくくなる

// 以下は新しい動作起動の仕組み Call????() 用の定数
const int CALLTHRU     = 0; // Call????() から Act????() を起動しなかった
const int CALLMATCH    = 1; // Call????() から Act????() を起動した

// 以下の定数は act_param[] の要素数、不足するときは増やして構わない
const int AP_PARAMS    = 48;// パラメータの受け渡し用配列のサイズ

// 以下の定数は act_param[] 中の個々の値の位置(配列の添字)
// ここで指定される配列中の位置は、同じ場所を複数の動作間で使いまわしても
// 問題ないが、現在は念の為すべて異なる場所を割当てるようにしている

const int AP_STATE     = 0; // 必須、動作開始時 ACT_START か、途中 ACT_MOVING か

const int AP_TIMETOTAL = 1; // 必須、動作時間、秒 float
const int AP_TIME1     = 2; // 動き１の時間、秒 float
const int AP_TIME2     = 3; // 動き２の時間、秒 float
const int AP_TIME3     = 4; // 動き３の時間、秒 float
const int AP_TIME4     = 5; // 動き４の時間、秒 float
const int AP_TIME5     = 6; // 動き５の時間、秒 float
const int AP_TIME6     = 7; // 動き６の時間、秒 float
const int AP_TIME7	   = 8; // 動き７の時間、秒 float
const int AP_TIME8	   = 9; // 動き８の時間、秒 float

const int AP_CTRL      = 10; // 制御の方法、MODENONE, MODEPROFILE, ...
const int AP_PHASE     = 11; // 動きの種類、歩行と旋回は４種類の動きの組み合わせ
const int AP_ITER      = 12; // 繰り返し回数

const int AP_NECK_HA   = 15; // 首振り水平角度
const int AP_NECK_VA   = 16; // 首振り垂直角度

const int AP_WALK_DIR  = 17; // 歩行で進む向き、FORWARD, BACKWARD
const int AP_TURN_DIR  = 18; // 回転方向、TURNCW 時計回り, TURNCCW 反時計回り

// 以下の3個の座標はクラスの変数で受け渡すことに変更、2026年5月名前を変えて復活
const int AP_GRABX     = 19; // 掴む座標 X
const int AP_GRABY     = 20; // 掴む座標 Y
const int AP_GRABZ     = 21; // 掴む座標 Z
const int AP_OBJFIND   = 22; // 別の値に変えたので、機能を変更した

const int AP_BSHIFTX    = 23; // シフト距離 X, ActBodyShift() 用
const int AP_BSHIFTY    = 24; // シフト距離 Y
const int AP_BSHIFTZ    = 25; // シフト距離 Z
const int AP_BSHIFTFLAG = 26; // 動かす脚のオンオフの設定を保持する

const int AP_LIDAR_VUP  = 27; // LIDAR VSCAN の上の角度、2023年は未使用
const int AP_LIDAR_VDOWN= 28; // LIDAR VSCAN の下の角度

const int AP_FTID       = 29; // FTセンサの番号
const int AP_FTHOLD     = 30; // 掴んだか否か

const int AP_HANDNUM    = 31; // LEFT or RIGHT
const int AP_HANDSTATE  = 32; // HANDOPEN or HANDCLOSE
const int AP_HANDRAD    = 33; // radian

// const int AP_OBJFIND    = 34; // 値重複Vision関係、物体を見つけたか、個数
const int AP_FILTER     = 35; // 画像処理で使うフィルタの種類
const int AP_IMGORCOL   = 36; // 画像の種類および色

const int AP_HOHABAX    = 37; // 一歩の歩行距離
const int AP_TURNDEG    = 38; // 旋回一回の角度(deg)

const int AP_HATCH_LEFT = 39; // 首振り水平角度
const int AP_HATCH_RIGHT= 40; // 首振り垂直角度

const int AP_P1         = 41; // 汎用のパラメータ、現在の最後の値

const int AP_ELAPSEDTIME= 42; // Act????() から得る経過時間

// その他、ActWalk(), ActTurn() 関連の定数

const int FORWARD  = 1; // 前進
const int BACKWARD =-1; // 後退
const int TURNCW   =-1; // 時計回り
const int TURNCCW  = 1; // 反時計回り

// image.cpp に含まれる画像処理関連の関数用
// フィルタの種類
const int FOC      = 0;
const int SOBEL    = 1;
const int LAPLACIAN= 2;
// 結果を window で表示するか
const int SHOW     = YES;
const int NOSHOW   = NO;
// 色を表す定数
const int RED      = 11; // COLOR, DEPTH と被らないように
const int GREEN    = 12;
const int BLUE     = 13;
const int YELLOW   = 14;
const int CYAN     = 15;
const int MAGENTA  = 16;
const int BLACK    = 17;
const int WHITE    = 18;

const int CVMALL = 8; // 各カメラに用意する cv::Mat のバッファの総数
const int CVMB0  = 0; // Buffer 0
const int CVMB1  = 1; // Buffer 1
const int CVMIN  = CVMB0; // 入力データ用の標準バッファ
const int CVMOUT = CVMB1; // 出力データ用の標準バッファ
const int CVMB2  = 2; // 2 も使用可, CV Mat Buffer 2
const int CVMB3  = 3; // その他のバッファ, 4, 5, ... も使用可

#endif // HEADER_CONST
