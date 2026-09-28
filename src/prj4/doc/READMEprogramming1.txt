// サンプルプログラム LessonMotion() と様々な動きを呼び出す Call????() の書き方

ここでは lessonneo.cpp, LessonMotion() による動きの選択と起動の仕組みと、
Call????() による個々の動きの起動の仕方、を示す。

Call????() 関数による記述では以下のようになる。
・一つの動きに関する記述は一つの Call????() 関数にまとめることができて管理が容易に
　なった。
・個々の動きに関する記述はすべて Call????() 関数内にあるので、LessonMotion() の行数
　が短くなり可読性が改善され、メンテナンスが容易になった。
・次の動きを指定する機能として、新たに配列 next_a[] を導入したことで、ある動きの終了
　時に次の動きを明示的に指定できるようになった。前の動きの結果によって次の動きを切り
　替える処理がわかりやすく書けるようになった。
　next_a[] は前の動きが終わって次の動きが始まるまでに変更すれば反映される。
　変更する処理はプログラムのどこで実行してもよい。
　センサで調べた結果によって次の動きが変わるケースが典型的な活用例となる。
・起動する動きは個々の動きに割り当てられた定数 ACT_???? で選択され、現在の動きを記憶
　する変数 act_num の値を調べて switch 文の分岐で実行される。
　以下はswitch 文の冒頭部分である。
  switch (act_num) {
    case ACT_INITPOSE : CallInitialPose(m, 0); break; 
  ...
  }
  act_num に ACT_INITPOSE が設定されていたら最初の case 文が実行され、
  CallInitialPose(m, 0) が実行される。
　次の動きの指定の例として、CallInitialPose(m, 0) の次の動きは next_a[ACT_INITPOSE]
  に記録されている。

次に LessonMotion(), Call????() を使用する書き方の例を示す。実際のプログラムの
LessonMotion() の冒頭部分である。

void LessonMotion(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[])
// m[] はモータコントローラのインスタンスへのポインタの配列、モータを直接動かすのに必要
// *c はRGBカメラのインスタンスへのポインタ、画像処理、画像を扱うために必要
// *d は深度(depth)カメラのインスタンスへのポインタ、深度画像を扱うために必要
// ft[] は力覚・トルクセンサのインスタンスへのポインタの配列
{
// 各種の行動を実行する順番や繋がりを ACT_????, Call????(), next_a[ACT_????] の組合わせで定義する
    
//  act_param[AP_STATE] = ACT_START;           // ここで実行すると毎回初期化になってまずいので不可

// 以下の4個の if 文で行う処理は、 Call????() を導入後変更されたので注意する。
// act_num の機能の変更に伴うものである

// state の取り得る値は以下の if 文の4種類。
// ACT_START == state は prg4 の起動直後のみで、その後この状態は現れない。
//     代わりに act_param[AP_STATE] == ACT_START によって動作開始を表す。
// ・state = ACT_START はプログラムの起動を表す。
// ・act_param[AP_STATE] = ACT_START はある一つの動きの起動を表す。
// ACT_MOVING == state は一つの動きが動作中なので何もすることはない。
// ACT_END == stateは一つの動作終了で次の動作に移る。
// ACT_ERR == state はエラーが起きたときで、通常は関係ない。
    
    if (ACT_START == state) {                  // プログラム起動直後、最初「だけ」ここから
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_num = ACT_INITPOSE;                // 最初の動作は ACT_INITPOSE
	act_last = ACT_NONE;                   // 直前の動作はまだない
//	act_next = ACT_INITPOSE;               // 次の動作を指定、Call????() 用、今は不使用
	SetNextMotion();                       // 各動作の次の動作の初期設定をする
    }
//  if (ACT_MOVING == state) NOTHING;          // 動作継続中の時は理由がなければ指示を変更しない
    if (ACT_END == state) {                    // 直前の行動が終了したら次を決める
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_num = next_a[act_num];
	act_last = act_num;                    // 直前の行動を記録、後で必要になるかもしれないので
//      act_next = ???;                        // act_next は直前の Call??() 末尾で指定、ここでは不要
    }
    if (ACT_ERR == state) {                    // 状態がエラーの時の処理
	act_last = act_num;                    // 直前の動作を記録
	act_param[AP_STATE] = ACT_START;       // 今までの動きを終了
	act_param[AP_TIMETOTAL] = 60.0;
	act_num = ACT_STAY;                    // 60 秒間動きを停止
//	act_next = ACT_STAY;                   // 60 秒間動きを停止、202603以後ここでは指定不要
	fprintf(stderr, "ACT_ERR was detected. 60 sec stopped.\n");
    }

// ロボットの位置や向きをログファイルに記録する
    Log();
    
// センサの状態の確認や、その結果により現在実行中の動作や
// 次の動作を変更できる関数 Think() を新たに用意した。
    Think(m, c, d, ft);
    
    float xyzrpy[6] = {0, 0, 0.3, 0, 0, 0}; // ロボットの現在位置の読出し、設定用の配列
    
// 以下の case 文のどれかが条件にマッチして対応する動作 Call????() が実行される

    switch (act_num) {
    case ACT_INITPOSE : CallInitialPose(m, 0); break; // 第二引数は使わなくなったので 0 でも可
    case ACT_STAY : CallStay(m, next_a[ACT_STAY]); break; // 動きを止める
    case ACT_RELAX : CallRelax(m, next_a[ACT_RELAX]); break; // 脱力、全関節を0度に
    ...    
//  中略
    ...
    case ACT_SHELL : CallShell(m, next_a[ACT_SHELL]); break; // シェル、コマンド入力で動かせる
    case ACT_SHELLSLEEP : CallShellSleep(m, next_a[ACT_SHELLSLEEP]); break; // シェル、一時停止
    default : 
// どの ACT_????, Call????() にもマッチしないときの処理、本来ここには達しないはず
    fprintf(stderr, "Error: Bad next_a[] %d specified, or other error occured.\n", 0);
    state = ACT_ERR; break;
    }
    return;
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

Call????() の書き方の例として CallLesson1(), CallLesson2() を次に示す。
以前の Call????() は複雑な書き方であったが、202603 の改良で簡潔に
書けるようになった。現在はメッセージの表示と最初の呼出で初期設定をして、
その後 Act????()を呼び出すだけになった。
以前は Call????() の返り値は利用されていたが一旦利用しないことになった。
next_a[] による次の動きを変更したい時は Call????() の中でも可。


int CallLesson1(C1 m[], int next) // 動きの例、関節を指定して動かす
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLesson1(), 関節を指定して個別に回転させる\n");
#endif
	act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
    }
    state = ActLesson1(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

int CallLesson2(C1 m[], int next) // 動きの例、逆運動学で腕を動かす
{
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのは起動直後だけなのでこうは書けない
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLesson2(), 逆運動学で脚を動かす\n");
#endif
	act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
    }
    state = ActLesson2(m, act_param);
    return CALLMATCH; // 起動したことを返り値で戻す
}

上記二つの例では動作時間のみを指定しているが、回転角度や目標座標をあわせて
指定することも可能である。
