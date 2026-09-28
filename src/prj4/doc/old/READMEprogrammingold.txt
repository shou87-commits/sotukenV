ロボットの動きを制御するプログラムの処理の詳細

プログラム中でロボットの動きを決めている部分は次の
一連の関数になる。

main(); -> intelligence(); -> UserMotion(); -> LessonMotion();

intelligence() はタイマーによる設定で、1秒間に100回呼び出される。
すなわちロボットの動きは 1/100 秒周期で指定、変更できる。
intelligence(), UserMotion() は次の関数を呼び出しているだけで、
この構成で実質的に動きを決めているのは lessonneo.cpp にある
LessonMotion() である。
LessonMotion() は同様の機能を持った別の関数に簡単に入れ替え可能で、
ここを自分で作った関数に変更することで独自の動きを実現できる。

複雑な動きを実現するには、次の (1), (2) を作成する必要がある。

------------------------------------------------------------------

(1) 様々な動きを組み合わせて複雑な動きを実現する

lessonneo.cpp, LessonMotion() はプログラムの書き方を示すために作成した
サンプルプログラムである。
※
似た名前で lesson.cpp というファイルがあるが、こちらは制御の仕組みが古い
ので使用しない。またこのプログラムの開発初期に作成した user.cpp の
MyMotion() もあるが、こちらも記述の仕方が古くて現在では実用的でないので
使用しない。実現できることに関して両者で違いはない。

自分のプログラムを作成する時に、
LessonMotion() を直接書き換えると、後日新しいパッケージが配布された時に
入れ替える作業が大変になるので、別名でコピーした ????Motion()
(例 佐藤君なら sato.cpp を用意して、SatoMotion()とするなど)を用意して、
そちらを編集するのがよい。
自分で作った ????Motion()があれば LessonMotion() を使う必要はない。

以下に LessonMotion() の処理の概要を示す。処理の流れがわかるように
細かい設定は省略してある点に注意する。
lessonneo.cpp 内の LessonMotion() は一通りのデモの動きを実行するのに
必要な設定が含まれているので実際に動かして確認することができる。

一まとまりの動作に対して、
Call????() という関数を作成して、そこから具体的な動きを実行する関数
 Act????() を呼び出すことで一連の動きを実現する。
LessonMotion() は数多くある Call????() を管理して呼び出す機能を実現する。

個々の動きを実現する関数は Act????() という名前になっている。
歩行や旋回、立ち上がりなどの基本的な動きを実現する関数は
action.cpp, action2.cpp に記述されている。
実行時にどのようなパラメータが必要かなど、詳細は
ソースファイルを参照するとよい。

LessonMotion() と Call????() 関数の記述の仕方は、別のファイル
READMEcall.txt に詳しく解説されているのでそちらを参照する。

------------------------------------------------------------------

(2) 個々の動きを実現する関数の書き方

様々な動作は、Call????() と Act????() という2種類の関数を組み合わせて
管理、実行される。一般的に Act????() は Call????()から呼び出される。
Call????()は動作を開始するかどうかの判断、開始時の初期設定や次の動作
との連携を管理する。
Act????() は時間を管理して、実際の動きを実行する役目がある。
以下では関節や脚を指定してなんらかの動きを実現する Act????() 関数の
具体的な書き方を示す。
例として action2.cpp にある ActEX1(), Act1() や、lessonneo.cpp にある
ActLesson3()を見てみる。これらの関数の中に、

// 動作の指示はここから ////////////////////////////////
...
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

3番目の例 ActLesson3() は、複数の動きを組み合わせて順に実行する
方法の例である。(2-1), (2-2) ではどちらも1個の動きを実行するが、
この例では2個以上の動きの組み合わせを一つの関数で実現できる。
ActLesson3() では 4 個の動きを組み合わせている。

これまでよりもパラメータが増えて、以下のように全体の動作時間と、
個々の動きの時間を指定する必要がある。
duration = p[AP_TIMETOTAL]; // 合計動作時間、秒
time1 = p[AP_TIME1]; // 動き１の時間、秒
time2 = p[AP_TIME2]; // 動き２の時間、秒
time3 = p[AP_TIME3]; // 動き３の時間、秒
time4 = ...

また動作の切り替えを行うためにタイマのカウントの仕方と判定が
これまでよりも複雑になる。

なおこの書き方は必須ではなくて、1個の関数に 4 個の動きを詰め込まなくても、
LessonMotion() 内で、1個の動きを実現する関数を順に 4 回呼び出すようにしても
まったく同じことが実現できる。(2-3) の書き方は1回の関数呼び出しにいくつかの
動きを含めることができるので、必ずセットで順に行う動きをまとめて記述する
ときなどに使うとよい。

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


------------------------------------------------------------------

自分が作ったファイルや関数を配布されたプロジェクトに統合する手順

src/sato.cpp を説明用の仮のサンプルプログラムとして使用する。
これは個別ユーザ(仮想佐藤君)が独自の動きを記述した場合の例で、
lessonneo.cpp を自分用に sato.cpp としてコピーして内容を
書き換えればそのまま使える。
user.cpp 、その他のファイルを書き換える手間を非常に少なくできる

手順は以下になる。

(1)
自分で作成したファイル src/sato.cpp の名前を prj4/CMakeLists.txt
の最後の方にある次の行に「1行で」以下のように追加する。
add_executable(prg4 src/prg4.cpp src/motion.cpp src/motorclass.cpp
 src/ikmotion.cpp src/user.cpp src/action.cpp src/action2.cpp src/sato.cpp)

(2)
(2-1), (2-2) のどちらかを実行する。(2-1) の方が楽なのでおすすめ、(2-2)
は参考程度の情報として残してある。

(2-1)
自分で作成した関数 SatoMotion() を呼び出すように user.cpp を作成して、
もともとある user.cpp と置き換える。SatoMotion() のプロトタイプ宣言は
自分で作成した user.cpp に記述すれば user.h は変更不要。

(2-2)
別の方法として、一連の動きを定義する関数 SatoMotion() の
プロトタイプ宣言は user.cpp から見えるように user.h に記述する。
また SatoMotion() を UserMotion() から呼び出すように user.cpp 
を書き換える。
当初はこちらの方法を用意したが、後に (2-1) の方法でできるようにした
結果、今では (2-1) の方が簡単だろう。

(3)
自分で作成した独自の動きを実現する関数 ActSato() があるとする。
この関数をこのファイル内でだけ利用するなら、そのプロトタイプ宣言は
このファイル sato.cpp 内に記述すればよい。
int ActSato(C1 m[], float p[]);
一方、他のファイルからも呼び出せるようにするなら、
ActSato() のプロトタイプ宣言は user.h に記述する。

(4)
自分が作った動作を表す任意の数値を以下のように定義する。
const int ACT_SATO = 501;
const.h 内の他の数値 ACT_**** と被らないか事前に確認しておく。
(3) と同じように他のファイルからも利用できるようにする場合は
const.h または user.h に追記する。
