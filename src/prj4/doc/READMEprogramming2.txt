Act????(); ロボットの動きを実現するプログラムの処理の詳細

プログラム中でロボットの動きを決めている部分は次の
一連の関数になる。

main(); -> intelligence(); -> UserMotion(); -> LessonMotion();
-> Call????(); -> Act????(); -> 実際の動き

独自の動きを新たに作る場合は LessonMotion(); 以降の変更が必要になる。
main(); から UserMotion();　までは通常変更する必要はない。
モータの追加やセンサを追加するときは main(); の変更が必要になる。
動きを追加する時は
LessonMotion(); から独自の動きが選択されるように switch 文を変更して、
独自の動きを実現する Call????(); , Act????(); を作成して追加する。

intelligence() はタイマーによる設定で、1秒間に100回呼び出される。
すなわちロボットの動きは 1/100 秒周期で指定、変更できる。
intelligence(), UserMotion() は次の関数を呼び出しているだけで、
この構成で実質的に動きを決めているのは lessonneo.cpp にある
LessonMotion() である。
LessonMotion() は複数の動きの繋がりや順番（例 歩いて、
モノを拾い上げて、持ち帰るなど）を決めている。
LessonMotion() 以降は個別の動きを実現する Call????(); と Act????();
（例 歩行を行う CallWalk();  ActWalk(); など）が呼び出される。
Call????(); は READMEprogramming1.txt で説明されたとおり、
動きに関するパラメータの初期設定を行う。以前は別の処理が含まれていた今はない。
Act????(); が実際の動きを実現する。
LessonMotion() で決める動きの繋がりと必要であれば新しい動きを実現する
Call????(); と Act????();
を自分で作って追加することで独自の動きを実現できる。

複雑な動きを実現するには、次の (1), (2) を作成する必要がある。

------------------------------------------------------------------

(1) 既存の様々な動きを組み合わせて複雑な動きを実現する

lessonneo.cpp, LessonMotion() は当初はプログラムの書き方を示すために作成した
サンプルプログラムであったが、その後ロボットの動きはすべてここで決められている。
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
READMEprogramming1.txt に詳しく解説されているのでそちらを参照する。

------------------------------------------------------------------

(2) Act????() 個々の動きを実現する関数の書き方

様々な動作は、Call????() と Act????() という2種類の関数を組み合わせて
管理、実行される。通常 Act????() は Call????()から呼び出される。
Call????()は動作開始時の初期設定や次の動作との連携を管理する。
Act????() は時間を管理して、実際の動きを実行する役目がある。
・関節を動かす Act????() は物理的な動きを伴うので数秒間（数百回の呼出）
　実行される。
・センサ情報を処理する Act????() は瞬時に（1回だけ呼出）実行される。

以下では関節や脚を指定してなんらかの動きを実現する Act????() 関数の
具体的な書き方を示す。
例として lessonneo.cpp にある ActLesson1(), ActLesson2(), ActLesson3()
を見てみる。これらの関数の中に、

// 動作の指示はここから ////////////////////////////////
...
// ここまでの間に書く //////////////////////////////////

というコメントがあるが、具体的な動きの指示はこの間に記述する。
コメントより前の部分は変数の定義や初期設定、
コメントより前の部分は呼び出し回数をカウントして、指定された時間で
実行するための処理が記述されている。どちらもほぼ一定の処理になるので
多くの場合は他の関数からコピーすればよい。

(2-1) 関節に直接指示を送る方法

最初の例 ActLesson1() は、関節を直接指定して指定した時間で指定した角度回転
させる、最もローレベルの指示の仕方である。その指示する内容は次の３行になる。
制御のモードについては、README1st.txt「モータの制御の仕方」に説明がある。
m[A7J1].ctrlmode = MODENONE; // 制御のモード、単純なPID制御
m[A7J1].duration = duration; // 動作時間、秒で指定
m[A7J1].targetangle.data = 1.0; // 目標角度、ラディアンで指定
上記m[] に指示する値は CallLesson1() 内で、またはそれ以前に別の場所で指定しておく。
m[] の値はモータドライバに対して目標角度や時間を指定する。
モータドライバは main() とは別に独立して機能するので、新しい目標角度を
指定すると直ちに回転が始まる。よってモータを回転させたい時は目標角度を
指定するだけでよい。

ある動きを実現するのに必要なすべての関節に対して、適切なタイミングで、
この形式で指示を送ればあらゆる複雑な動きを実現できる。しかし、計算が
非常に複雑になるので、多くの場合は次に示す逆運動学を利用して動きを生成
する。
ここで示した個別の関節に対するローレベルの指示は、逆運動学の
計算の対象にならない関節を動かすときに利用する。

配列 m の添字 A7J1 は関節の場所を示している。A7 は首を表し、J1 は本体から
見て１番目の関節を表す。const.h に定義されている。
p[] はCallLesson1() から受け取るパラメータの配列で、動きに固有の指定が
必要な値が渡される。
p[AP_TIMETOTAL], p[AP_STATE] はこの関数と呼び出し元の関数間の値（パラメータ）
のやり取りに利用される。関数によってどのようなパラメータが必要になるかは
異なるので、詳細はソースファイルと const.h を確認する。

この例では
CallLesson1()からパラメータを受け渡しする引数として p[] が渡され、
動作時間3秒が指定される。ActLesson1()ではそれを受け取って時間を指定する。
目標角度は -1.3 radian でActLesson1()内で固定されている。
また制御の種類も MODENONE でシンプルなPID制御がActLesson1()内で固定されている。
パラメータの設定は動きの実行前であれば LessonMotion(), Call????(), Act????() のどこ
で設定しても結果は同じになる。

CallLesson1(), ActLesson1() は単純な動きの例なので
CallLesson1(), ActLesson1() のそれぞれでパラメータの値が記述されているが、
一般的に様々な動きを実現するには Call????() でパラメータを決めて, Act????()
では動的にパラメータを変更するなどの操作はしないほうがよいと思われる。
考え方として
・Call????() では環境の変化によるパラメータの動的な設定
・Act????() では渡されたパラメータに沿って動きを実現する。
　実行内容が動的に変化する要素を持たせない。
というように機能を分担するのがよいと思われる。

int ActLesson1(C1 m[], float p[])
{
    static int timer = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

// 動作の指示はここから ////////////////////////////////
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	// A1J1 は1番の arm の1番の joint 、左前脚の肩
	m[A1J1].ctrlmode = MODENONE; // モータ1個にパラメータ3個、これは制御モード PID
        m[A1J1].duration = duration; // 回転時間
	m[A1J1].targetangle.data = -1.3; // 目標角度 radian
	
	m[A1J2].ctrlmode = MODENONE; // ここから2個めのモータのパラメータ
        m[A1J2].duration = duration;
	m[A1J2].targetangle.data = 1.3; // radian
        timer = 0; // この行はなくてもよさそう
    }    
// ここまでの間に書く //////////////////////////////////
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


(2-2) 逆運動学により動きを生成する方法

2番目の例 ActLesson2() は、逆運動学により動きを生成して、手首または指先
を指定した座標に動かす指示ができる。逆運動学の対象は各脚の肩から
手首までの3関節であり、目標座標を1点指定すれば、脚途中の3関節の目標角度を
計算して自動で動かしてくれる。
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

逆運動学を計算する関数 IKMotions(), IKIncrementMotions(), IKCalc() は
当初は返り値がなかったが、2022年6月以降計算が正常に終わったか、問題が
あったかを示す返り値 NORMAL, ERROR を返すようになった。
返り値を返す代わりに、これまでエラーメッセージを表示していたが、それは
止めた。必要であれば返り値を参照することで、逆運動学の計算で問題の有無を
調べることができる。

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
    if (0 == (timer % cycletotal)) { // 以下の処理は上の if 文に移してもよさそう？
	float target1[XYZ] = { 0.65, 0.15, 0.1}; // 原点（本体重心）からの目標座標
	float target2[XYZ] = { 0.65,-0.15, 0.5};
	IKMotions(m, LF, target1, ikparam); // 左前脚を target1 に動かす
	IKMotions(m, RF, target2, ikparam); // 右
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
自分で作成した関数 SatoMotion() を呼び出すように user.cpp, UserMotion();
を作成して、もともとある user.cpp と置き換える。SatoMotion() の
プロトタイプ宣言は自分で作成した user.cpp に記述すれば user.h は変更不要。

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

------------------------------------------------------------------

練習問題
１．辺の長さを適当に決めて、四角形の経路を歩かせる。
２．URDFの記述の仕方を理解する、青色の階段と緑色の表彰台を作って置く。
３．階段を登らせる。
４．画像処理の方法を理解する、表彰台を見つけて中央に載る。
