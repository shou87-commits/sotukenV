ロボットの動きを制御するプログラムの処理の詳細

※ この文書の内容は古くなったので読む必要はなくなった。
　最新の内容は READMEprogramming.txt を確認する。
   
プログラム中でロボットの動きを決めている部分は次の
一連の関数になる。

main(); -> intelligence(); -> UserMotion(); -> SuzukiMotion();

intelligence() はタイマーによる設定で、1秒間に100回呼び出される。
すなわちロボットの動きは 1/100 秒周期で指定、変更できる。
intelligence(), UserMotion() は次の関数を呼び出しているだけで、
この構成で実質的に動きを決めているのは SuzukiMotion() である。
SuzukiMotion()は簡単に入れ替え可能で、ここを自分で作った関数に
変更することで独自の動きを実現できる。

複雑な動きを実現するには、次の (1), (2) を作成する必要がある。

------------------------------------------------------------------

(1) 様々な動きを組み合わせて複雑な動きを実現する

suzuki.cpp, SuzukiMotion() はプログラムの書き方を示すために作成した
もので、通常使用するのは user.cpp の MyMotion() である。ただし実現
できることに関して両者で違いはないので、自分で作った ????Motion()
があればMyMotion() を使う必要はない。

以下に SuzukiMotion() の処理の概要を示す。処理の流れがわかるように
細かい設定は省略してある点に注意する。
suzuki.cpp 内の SuzukiMotion() は必要な設定が含まれているので実際に
動かして確認することができる。

関数内の処理は大きく前半と後半に分けることができる。
前半は様々な動きの間のつながりを記述する。
後半は個々の動きを対応する関数を呼び出すことで実行する。

個々の動きを実現する関数は Act????() という名前になっている。
歩行や旋回、立ち上がりなどの基本的な動きは action.cpp, action2.cpp
に記述されている。実行時にどのようなパラメータが必要かなど、詳細は
ソースファイルを参照するとよい。

void SuzukiMotion(C1 m[])
{
//////////////////////////////////////////////////////////
// 
// 前半　各種の行動を実行する順番や繋がりを定義する
// フローチャートのように記述できる
// 
//////////////////////////////////////////////////////////

// ACT_START == state はプログラム起動直後の動きを指定する
// 最初は初期姿勢、立ち上がる

    if (ACT_START == state) {
	act_num = ACT_INITPOSE;
    }

// ACT_MOVING == state は動作が継続中
// この時は特別な理由がなければ指示を変更しない

    if (ACT_MOVING == state) ; 

// ACT_END == state はある動作が終了した状態を表すので、次の動作を 
// act_num に指定する。
// 例として、今までの動きが歩行なら次はターンするという指定は次のようになる
// if (ACT_WALK == act_last) act_num = ACT_TURN;

    if (ACT_END == state) {                    // 直前の行動が終了したら
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を覚えておく、後で必要になるかも

	if (ACT_INITPOSE == act_last) act_num = ACT_SUZUKI;
	if (ACT_SUZUKI == act_last) act_num = ACT_SUZUKI;
	if (ACT_WALK == act_last) act_num = ACT_TURN;
	if (ACT_TURN == act_last) act_num = ACT_WALK;
	if (ACT_ERROR == act_last) act_num = ACT_INITPOSE; // エラー時の次の行動を決める
    }
    if (ACT_ERR == state) act_num = ACT_ERROR; // 状態がエラーの時はエラー処理へ

//////////////////////////////////////////////////////////
// 
// 後半　個々の行動の具体的な内容を定義する
// 必要なパラメータをセットして動きを実現する関数を呼び出す
// 
//////////////////////////////////////////////////////////
    
// ここから先は個々の行動に必要なパラメータをセットして関数を呼び出す
// act_num に次に実行する動作の番号が入っているので、switch 文で対応する関数を
// 呼び出す。例として、act_num が ACT_INITPOSE なら関数 ActInitialPose()
// を呼び出す。

    switch (act_num) {
        case ACT_INITPOSE : // 初期姿勢、立ち上がる
	    state = ActInitialPose(m, act_param);
	    break;
	case ACT_SUZUKI : // 動きの例、前の腕を交互に動かす
	    state = ActSuzuki(m, act_param);
	    break;
        case ACT_1 :
	    state = Act1(m, act_param);
	    break;
	case ACT_2 : // 動きの例、実用的なものではない
	    state = Act2(m, act_param);
	    break;
        case ACT_STAY : // 現状維持
	    state = ActStay(m, act_param);
	    break;
        case ACT_ERROR : // エラーが発生したとき
        default : // どの case にもマッチしないとき、本来ここにはマッチしないはず
	    fprintf(stderr, "Error: Bad act_num specified, or other error occured.\n");
	    state = ACT_END;
            break;
    }
}

------------------------------------------------------------------

(2) 個々の動きを実現する関数の書き方

以下では関節や脚を指定してなんらかの動きを実現する関数の書き方を示す。
例として示す関数 ActEX1(),  Act1(), ActSuzuki() の中に、
// 動作の指示はここから ////////////////////////////////

// ここまでの間に書く //////////////////////////////////
というコメントがあるが、具体的な動きの指示はこの間に記述する。
コメントより前の部分は変数の定義や初期設定、
コメントより前の部分は呼び出し回数をカウントして、指定された時間で
実行するための処理が記述されている。どちらもほぼ一定の処理になるので
多くの場合は他の関数からコピーすればよい。

(2-1) 関節に直接指示を送る方法

最初の例 ActEX1() は、関節を直接指定して指定した時間で指定した角度回転
させる、最もローレベルの指示の仕方である。その内容は次の３行になる。
制御のモードについては、README1st.txt「モータの制御の仕方」に説明がある。
m[A7J1].ctrlmode = MODENONE; // 制御のモード、単純なPID制御
m[A7J1].duration = duration; // 動作時間、秒で指定
m[A7J1].targetangle.data = 1.0; // 目標角度、ラディアンで指定

ある動きを実現するのに必要なすべての関節に対して、適切なタイミングで、
この形式で指示を送ればあらゆる複雑な動きを実現できる。しかし、計算が
非常に複雑になるので、多くの場合は次に示す逆運動学を利用して動きを生成
する。ここで示した個別の関節に対するローレベルの指示は、逆運動学の
計算の対象にならない関節を動かすときに利用する。

配列 m の添字 A7J1 は関節の場所を示している。A7 は首を表し、J1 は本体から
見て１番目の関節を表す。const.h に定義されている。
p[AP_TIMETOTAL], p[AP_STATE] はこの関数と呼び出し元の関数間の値（パラメータ）
のやり取りに利用される。関数によってどのようなパラメータが必要になるかは
異なるので、詳細はソースファイルと const.h を確認する。

int ActEX1(C1 m[], float p[])
{
    static int timer = 0; // 呼び出し回数のカウンタ
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100); // この関数が呼び出される回数

// 動作の指示はここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回だけ
	m[A7J1].ctrlmode = MODENONE;
        m[A7J1].duration = duration;
	m[A7J1].targetangle.data = 1.0; // radian
/*	m[A7J2].ctrlmode = MODENONE; // 必要なら 2 個以上に同時に指示することも可能
        m[A7J2].duration = duration;
	m[A7J2].targetangle.data = 1.0; 
*/
        timer = 0;
    }    
// ここまでの間に書く //////////////////////////////////
    timer++;
    if (timer <= cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
	return ACT_END;
    }
}

(2-2) 逆運動学により動きを生成する方法

2番目の例 Act1() は、逆運動学により動きを生成して、手首または指先
を指定した座標に動かす指示ができる。逆運動学の対象は各脚の肩から
手首までの3関節である。
逆運動学は関数 IKMotions() で計算を行い、その結果に沿って脚を動かす
ことができる。引数は例えば IKMotions(m, LF, target1, ikparam) となるが、
・ m はモータドライバの配列、
・ LF は対象とする脚の番号、LFはLeft Front で左前脚を表す、
・ target1 はローカル座標系で表現する目標座標、
・ ikparam は逆運動学を計算する時に必要なパラメータ
を表す。
ikparam[] は次の値を指定する。詳細は const.h に記述されている。
ikparam[0] は実行時間、秒
ikparam[1] は手首、指先のどちらを目標座標に動かすか
ikparam[2] は肘を上にするか下にするか
ikparam[3] は制御のモード
を指定する。

2022年6月に新しい逆運動学の関数 IKIncrementMotions() が追加された。
IKMotions() が目標座標を直接指定するのに対して、IKIncrementMotions()
は対象とする脚に対して最後に指定した目標座標からの相対座標(差分の座標)
を指定してそこに脚を動かすことができる。この関数を活用する動作の指示
として、ActBodyShift() がある。現在の脚先の位置を変えずに、本体だけを
x, y, z ３方向に指定した距離だけ動かすことができる。

逆運動学を計算する関数 IKMotions() IKIncrementMotions() IKCalc() は
当初は返り値がなかったが、2022年6月以降計算が正常に終わったか、問題が
あったかを示す返り値 NORMAL, ERROR を返すようになった。
返り値を返す代わりに、これまでエラーメッセージを表示していたが、それは
止めた。必要であれば返り値を参照することで、逆運動学の計算で問題の有無を
調べることができる。


int Act1(C1 m[], float p[])
{
    static int timer = 0;
    static float ikparam[IKNUMPARAM];
    float duration = p[AP_TIMETOTAL]; 
    int cycletotal = (int)(duration*100);

// 動作の指示はここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	ikparam[0] = duration;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODENONE;
        timer = 0;
    }
    if (0 == (timer % cycletotal)) {
	float target1[XYZ] = { 0.65, 0.65, 0.0};
	float target2[XYZ] = { 0.65,-0.65, 0.3};
	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ここまでの間に書く //////////////////////////////////
    timer++;
    if (timer <= cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
	return ACT_END;
    }
}

(2-3) 複数の動きを組み合わせて順に実行する

3番目の例 ActSuzuki() は、複数の動きを組み合わせて順に実行する
方法の例である。(2-1), (2-2) ではどちらも1個の動きを実行するが、
この例では2個以上の動きの組み合わせを一つの関数で実現できる。
ActSuzuki() では3個の動きを組み合わせている。

これまでよりもパラメータが増えて、以下のように全体の動作時間と、
個々の動きの時間を指定する必要がある。
duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
time1 = p[AP_TIME1]; // 動き１の時間、秒
time2 = p[AP_TIME2]; // 動き２の時間、秒
time3 = p[AP_TIME3]; // 動き３の時間、秒

また動作の切り替えを行うためにタイマのカウントの仕方と判定が
これまでよりも複雑になる。

なおこの書き方は必須ではなくて、1個の関数に3個の動きを詰め込まなくても、
SuzukiMotion() 内で、1個の動きを実現する関数を順に3回呼び出すようにしても
まったく同じことが実現できる。(2-3) の書き方は1回の関数呼び出しにいくつかの
動きを含めることができるので、必ずセットで順に行う動きをまとめて記述する
ときなどに使うとよい。

int ActSuzuki(C1 m[], float p[]) // 3種類の異なる動きを組み合わせる例
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

// 動作の指示はここから ////////////////////////////////

// 複数の動きを順番に行わせるときの書き方
// 初期設定は最初に１回
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { 
	ikparam[0] = time1;
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
    }
// １番目の動き
    if (0 == (timer % cycletotal)) {
	float target1[XYZ] = { 0.65, 0.65, 0.2};
	float target2[XYZ] = { 0.65,-0.65, 0.4};

	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ２番目の動き
    if (cycle1 == (timer % cycletotal)) { // cycle1 回呼び出されたら次の動きに移る
//	float ikparam[IKNUMPARAM]; // 設定値が前と同じなら省略可
	ikparam[0] = time2; // 動きによって時間が異なる場合
//	ikparam[1] = IKHAND;
//	ikparam[2] = IKELBOWUP;
//	ikparam[3] = (float)MODEPROFILE;
	float target1[XYZ] = { 0.65, 0.65, 0.5};
	float target2[XYZ] = { 0.65,-0.65, 0.5};

	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
// ３番目の動き
    if ((cycle1 + cycle2) == (timer % cycletotal)) { // 次の動きに移る
	ikparam[0] = time3; // 動きによって時間が異なる場合
	float target1[XYZ] = { 0.65, 0.65, 0.4};
	float target2[XYZ] = { 0.65,-0.65, 0.2};

	IKMotions(m, LF, target1, ikparam);
	IKMotions(m, RF, target2, ikparam);
    }
/*  if ((cycle1 + cycle2 +cycle3) == (timer % cycletotal)) {
// この処理は次でされるのでここでは不要
    }
*/
// ここまでの間に書く //////////////////////////////////

    timer++;
    if (timer <= cycletotal) { // 継続中
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // タイマをクリアして終了
	timer = 0;
	p[AP_STATE] = ACT_END;
	return ACT_END;
    }
}


------------------------------------------------------------------

自分が作ったファイルや関数を配布されたプロジェクトに統合する手順

src/suzuki.cpp を説明用のサンプルプログラムとして使用する。
これは個別ユーザ(仮想鈴木君)が独自の動きを記述した場合の例で、
このファイルを自分用にコピーして内容を書き換えればそのまま使える。
user.cpp 、その他のファイルを書き換える手間を非常に少なくできる

手順は以下になる。

(1)
自分で作成したファイル src/suzuki.cpp の名前を prj4/CMakeLists.txt
の最後の方にある次の行に「1行で」以下のように追加する。
add_executable(prg4 src/prg4.cpp src/motion.cpp src/motorclass.cpp
 src/ikmotion.cpp src/user.cpp src/action.cpp src/action2.cpp src/suzuki.cpp)

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
