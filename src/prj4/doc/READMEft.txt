FT(Force,Torque) Sensor 力、トルクセンサの使用法

※
FTセンサについての説明は、6脚ロボットでFTセンサを使えるように
実装した伊藤さんによる詳しい解説文書が READMEftsensor として同じ
ディレクトリに、および prj4/ftkanren/READMEftsensor として置いて
あるので、あわせて参照するとよい。
最初の実装時から以下の点が変更され、 READMEftsensor の記述内容から
変わったので注意する。
・prj4/config/prg4.yaml に FT センサとして使うモータのPID制御に
　関する設定を追加する必要がなくなった、すなわち
　書き換える必要がなくなった
・prj4/launch/prg4_*.launch にモータ制御のプログラム起動を追加する
　必要がなくなった、すなわち書き換える必要がなくなった
・prj4/src/prg4.cpp FTセンサに関連するモータ制御のオブジェクトを
　生成する必要がなくなった。ただしFTセンサに関連するサブスクライバの
　生成は引き続き必要である。

FTセンサを新たに追加して利用する場合は
・URDFファイルによるセンサの形状と取付位置の指定　と
・ロボットの動作を制御するプログラム prg4 でのデータの読み出し
を設定する必要がある。
これに関して追加や修正が必ず必要になるファイルが以下である。
(1) prj4/urdf/ft*.urdf
(2) prj4/src/prg4.cpp, const.h, ftclass.h
一方で、最初に記したように以下のファイルは変更不要になった。
prj4/config/prg4.yaml 変更不要
prj4/launch/prg4_gz.launch 変更不要

以降は上記 (1), (2) について具体的な記述内容を示す。
また (3) として、センサで計測された値をどのように読み出して
プログラム中で利用するか、その例をあわせて示す。


(1) URDFファイルについて
prj4/urdf に
ftall.urdf、右手と左手の指の内側に付けてある4個のFTセンサの記述
ft5.urdf、例としてロボットの背中に付けてあるFTセンサ
がある。
新たにセンサを追加する場合は、これらのファイルの内容を参考にする。
ft5.urdf は1個のセンサを追加する場合の例で、記述がなるべく簡潔に
なるようにしている。ft5 の 5 は5個目のセンサの意味で、6個目や7個目
を追加する場合はft5に関するファイルや記述をコピーして名前を変更する
と楽だと思われる。以下が ft5.urdf の記述内容である。

以下の記述で、FT5, ft5 という表記はこのセンサに固有の名前になるので、
新たに追加したセンサにおいてはそのセンサにあわせた名前に変更する。
その他、細かい説明はその行の右端に ##1 のように記号を付して、その後
注釈の説明を付ける。

<!-- ################################################################### -->
<!-- ################################################################### -->

    <joint name="jointNNFT5" type="revolute"> <!-- revolute for ft sensor --> ##0
        <parent link="body"/>                          <!-- senaka -->  ##1
<!--    <parent link="linkNN172"/> -->                 <!-- hand -->    ##1
        <child link="linkNNFT5"/>
        <origin xyz= "0 0 0.08" rpy="0 0 0" />         <!-- senaka -->  ##1
<!--    <origin xyz= "0 0.06 -0.05" rpy="0 0 0" /> --> <!-- hand -->    ##1
        <axis xyz="1 0 0"/>
        <limit effort="0" lower="0" upper="0" velocity="0"/>
    </joint>

    <link name="linkNNFT5">
        <visual>
            <origin xyz="0 0.05 0" rpy="0 0 0"/>                         ##2
            <geometry> <box size="0.01 0.01 0.01"/> </geometry>          ##2
        </visual>
        <collision>
            <origin xyz="0 0.05 0" rpy="0 0 0"/>                         ##2
            <geometry> <box size="0.01 0.01 0.01"/> </geometry>          ##2
        </collision>
        <inertial>
            <origin xyz="0 0.05 0" rpy="0 0 0"/> <mass value="0.005"/>   ##2
            <inertia ixx="1e-2" ixy="0" ixz="0" iyy="1e-2" iyz="0" izz="1e-2"/>
        </inertial>
    </link>

    <gazebo reference="linkNNFT5"> <material>Gazebo/Blue</material> </gazebo> ##3

    <gazebo reference="jointNNFT5">
        <provideFeedback>true</provideFeedback>
<!--    <turnGravityOff>false</turnGravityOff> -->
    </gazebo>

    <gazebo>
        <plugin name="ft5_sensor" filename="libgazebo_ros_ft_sensor.so">
	    <updateRate>100.0</updateRate>
            <topicName>ftsensor_5/raw</topicName>                         ##4
            <jointName>jointNNFT5</jointName>
            <gaussianNoise>0.0</gaussianNoise>
	</plugin>
    </gazebo>

    <transmission name="transNNFT5">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="jointNNFT5">
	    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
	</joint>
        <actuator name="motorNNFT5">
    	    <hardwareInterface>EffortJointInterface</hardwareInterface>
	    <mechanicalReduction>1</mechanicalReduction>
	</actuator>
    </transmission>

解説
##0 FTセンサを使うときにはここを revolute に設定するのが重要、他では動かない
##1 センサを付ける時の親となるリンクを指定する。この例では指先と背中の2種類の
　　例が記述してあり、2行セットでどちらかを有効にする。今は背中が選択されている
##2 センサを取り付ける位置（座標）と、センサの大きさを指定する。URDFにおける
　　一般的なリンクの指定と同じである。
##3 センサの色、ここでは青
##4 Gazebo 上のロボットモデルと、制御プログラム prg4 が通信するためにこの名前が
　　重要で、同じ名前が以下で出てくる prg4.cpp に出てくる。



(2) C++ のファイル群について

制御プログラム prg4 でFTセンサを使えるようにするための記述内容を説明する。
説明の文書はその場に // 説明のコメント
の形で記す。

// prg4.cpp

// 84行目付近、ここでFTセンサを使うための FTSENSOR 型のオブジェクトのインスタンスを
　　生成する。センサは右手と左手に2個ずつ、背中に1個で合計5個ある。追加する場合は
　　ここの配列の要素を追加する。FT5 の定義で "ftsensor_5/raw" が指定されているが、
　　ここの名前を使って、Gazebo上のロボットモデルのどのセンサを表すかを指定する。

// FT(force and torque) Sensor
    FTSENSOR ftsensor[NUMFTSENSORS]= {
        FTSENSOR(FTA1J1, "ftsensor_lf1/raw"),
        FTSENSOR(FTA1J2, "ftsensor_lf2/raw"),
        FTSENSOR(FTA4J1, "ftsensor_rf1/raw"),
        FTSENSOR(FTA4J2, "ftsensor_rf2/raw"),
        FTSENSOR(FT5, "ftsensor_5/raw"),
    };

// 169行目付近、グローバル変数のポインタとして、上で作ったインスタンスの配列の
　　先頭アドレスを入れておく。ここはセンサの個数が増えても変更不要。

gFT = ftsensor;

#########################################################################
#########################################################################
#########################################################################
#########################################################################

以下は様々な定数を定義する const.h に含まれる FTセンサ関連の記述である。

// const.h

// 26 行目付近、FTセンサの状態を表す値、変更不要

const float FTON = 1;
const float FTOFF = 0;

// 60行目付近、ftsensorの個数、増えたらこの値も増やす

const int NUMFTSENSORS = 5; // ftsensorの数

// 145 行目付近、個々のFTセンサのインスタンスの配列中の場所、
// 0から始まって1ずつ増えるようにセンサが増えたらここに定義を追加する

const int FTA1J1 = 0; // ft sensor lf1 力覚、トルクセンサ
const int FTA1J2 = 1; // ft lf2
const int FTA4J1 = 2; // ft rf1
const int FTA4J2 = 3; // ft rf2
const int FT5    = 4; // ft 5

// 290 行目付近、Act????() でFTセンサを扱う時の関連する値の保持用

const int AP_FTID       = 29; // FTセンサの番号
const int AP_FTHOLD     = 30; // 掴んだか否か

#########################################################################
#########################################################################
#########################################################################
#########################################################################



(3) プログラム中でのFTセンサの計測値の読み出しと利用について

以下はロボットが動いている時に、次の動作を選択して起動するための
lessonneo.cpp である。その中のFTセンサに関連する関数を示して、
FTセンサの計測値をどのように読み出して利用するかを示す。
FTセンサに関係しないコメントは消した状態で示す。

// 637行目付近、FTセンサの値を読み出すデモ

int CallFTDemo(C1 m[], FTSENSOR ft[]) // FTセンサのデータにアクセスするデモ
{
    int goflag = NO;
    if (ACT_FTDEMO == act_next) goflag = YES;
    if (NO == goflag) return CALLTHRU;

    if (ACT_START == (int)act_param[AP_STATE]) {
#ifdef CALLMSG
	fprintf(stderr, "CallFTDemo(), FTセンサのデータにアクセスするデモ\n");
	fprintf(stderr, "              10秒以内に、背中の緑色の物を下ろして値の変化を見よう\n");
#endif
	fprintf(stderr, "背中の青いFTセンサの値、直近 0.1秒の平均 ");
	act_param[AP_TIMETOTAL] = 10; // 動作時間 (sec)、この間デモを継続する
	act_num = ACT_FTDEMO;
    }
    state = ActStay(m, act_param);
    if (ACT_MOVING == state) {
	if (0 == (int)act_param[AP_ELAPSEDTIME] % 10) { // 0.1sec に1回表示
///////////////////////////////////////////////////////////////////////////////
// ロボットが動作中に力を調べる書き方
// 3行上にあるように、if (ACT_MOVING == state) の期間に調べると動作中の力が
// わかる。この例のように Call????() でも可能だし、Act????() の方でも同様の処理が
// 可能。
// FTセンサの計測値を読み出しているのは次の fprintf(stderr, ...); の行。
// ロボットの背中に付いているセンサを表すのが ft[FT5] で、それが持つメンバ関数
// Ftlastave() を呼び出すことで、直近の0.1秒間の計測値の平均値を返り値として
// 受け取ることができる。
// 計測値が直接記録されているのは ft[FT5]. f[XYZ][] という2次元配列のメンバ変数、
// [XYZ]は3次元を表し、続く [] は時間を表す。この配列から直接値を読み出すことも
// 可能であるが、リングバッファになっているのでどこを読み出すかは注意が必要。
// ここではこの関数の引数として ft が渡されているのでこの書き方ができる。
// 引数でない場合は gFT[FT5] という書き方になる。
// 計測される値は何も力がかからなければ 0.1 未満、力がかかると2とか3以上になる
// ことが多い。力の有無を判定する閾値を決める参考にするとよい。
///////////////////////////////////////////////////////////////////////////////
	    fprintf(stderr, "%.2f ", ft[FT5]. Ftlastave());
	}
    }
    if (ACT_END == state) {
	act_next = ACT_NECK;
    }
    return CALLMATCH;
}

/////////////////////////////////////////////////////////////////////////////////

FTセンサの値により脚、関節の動きを止める方法

関連するメンバ変数
armclass.h, motorclass.h
int ftstopflag;          // FTセンサによって停止するかの可否 ON or OFF                               
int ft_idx[NUMARMFT];    // この脚または関節の停止に関係するFTセンサの番号配列
float ft_thrd = 2.5;     // FTセンサの閾値、これを越えたら反応ありとみなす

関連するメンバ関数
armclass.h, motorclass.h
setftflag(int f);	// ftstopflag の設定用関数

脚と FTセンサ の組み合わせを指定して、特定の FTセンサ が反応したら
脚の動きを止められる。
記述は armclass.h の 60 行付近、ft_idx[NUMARMFT]　にある。
ftstopflag を ON にすると、この機能が有効になる。
ft_thrd を越えたらFTセンサが反応したとみなして停止する。

関節と FTセンサ の組み合わせを指定して、特定の FTセンサ が反応したら
関節の動きを止められる。
記述は motorclass.cpp の 40~55 行付近、ft_idx[NUMMOTORFT]　にある。
ftstopflag を ON にすると、この機能が有効になる。
ft_thrd を越えたらFTセンサが反応したとみなして停止する。

設定可能なFTセンサの最大個数は NUMARMFT, NUMMOTORFT 個

ftstopflag は任意のタイミングで ON or OFF を切り替えてよい。ON にしたら
すぐに効果が現れる。停止の必要がなくなったらすぐに OFF にしてよい。
書き方の例
// gMotor[A1J1]. setftflag(ON);
// gArm[LF]. setftflag(OFF);

以下は機能のデモの内容、効果を確認後はコメントアウトまたは消してよい。
FTセンサは各脚の接地面に１個だけ FTA1S1, --- FTA6S1 として付けてある。

armclass.h
// 必要な脚のみ停止判定用 FT センサの番号を設定する
        if      (LF == id) {ary[0] = FTA1S1;} // 複数個指定も可 {ary[0] = FTA1S1; ary[1] = FT5;}
        else if (LM == id) {ary[0] = FTA2S1;}
        else if (LB == id) {ary[0] = FTA3S1;}
        else if (RF == id) {ary[0] = FTA4S1;}
        else if (RM == id) {ary[0] = FTA5S1;}
        else if (RB == id) {ary[0] = FTA6S1;}
// 例として、左中脚だけFTセンサによる停止を有効にしてみる
	if (LM == id) ftstopflag = ON;

motorclass.cpp
// 例として、背中に荷重があれば右後脚の３関節の動きを止めてみる
    else if (A6J1 == id) {ary[0] = FT5;}
    else if (A6J2 == id) {ary[0] = FT5;}
    else if (A6J3 == id) {ary[0] = FT5;}

#########################################################################
#########################################################################
#########################################################################
#########################################################################

以下は READMEchangelog.txt 関係部分のコピーをもとにして
大幅に加筆したものである。

FTセンサが反応したら脚や関節を止められる仕組みを作った。
脚または関節にチェックすべきFTセンサを登録できるようにした。
脚を1本まとめて管理できるように armclass.h を作成した。

脚と関節に共通して
関連する変数は ftstopflag, ft_idx[], ft_thrd の3個である。
ftstopflag はセンサが反応したら止めるかどうかの設定、
ft_idx[] は脚や関節に関連付けるFTセンサの番号の配列、
ft_thrd は反応したかどうかを判定する閾値　を表す。

脚に関して動きを止める関数は void ftstop() である。
関節に関して動きを止める関数は void C1::Stop() であり、
C1::timerCallback() から呼び出される。

上記変数と関数は、
脚に関しては左中
脚を止める設定の例は armclass.h の 53--57 行付近、// FTセンサによる停止の可否の設定
脚とFTセンサの対応は 59--69 行付近、
に定義されている。
関節に関しては右後脚の
関節を止める設定の例は motorclass.cpp の 50--54 行付近、// 関節３個止める設定
関節とFTセンサの対応は 40--49 行付近、
に定義されている。
脚を止めるかどうかの処理は prg4.cpp の 330 行付近 void stoparmft() で行う。
Gazebo から届くFTセンサの情報と prg4.cpp で制御する脚との関係は prg4.cpp の
120 行付近にある。

FTセンサの反応によって脚や関節の動きを止める処理のデモは
motorclass.cpp の 51 行付近、
armclass.h の 57 行付近、
のコメントを外すと実際に動きが止まるのを見ることができる。
このデモ用に各脚の外側に脚が接地したかを判定するためのFTセンサを
付けることができる。付ける場合は urdf/buildurdfFTDEMO.sh を実行する。
通常は付ける必要はないので、urdf/buildurdf.sh を実行する。
デモ用の FT センサの urdf ファイルは urdf/fta1s1.urdf ~ fta6s1.urdf
である。urdf/buildurdfFTDEMO.sh を実行するとこれらのデータが
ロボット本体の urdf ファイルに追記される。

脚や関節を止める処理は、FTセンサの反応を検出した時点で
関係する関節の目標角度をその時点の角度で書き換えることでそれ以上
動かなくなる。これをFTセンサが反応する度に実行する。
新しい目標角度を与えてその時FTセンサが反応していなければ再度関節は
動く。
