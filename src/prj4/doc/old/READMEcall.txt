// サンプルプログラム LessonMotion() と様々な動きを呼び出す Call????() の書き方

ここでは (1) 関数前半の if 文と後半の switch 文の組み合わせで一連の動きを実現して
いた、従来の動きの選択と起動の仕組みと、(2) 新しく導入された Call????() による動き
の選択と起動の仕組みを対比させて、両者の仕組みやプログラムの書き方の違いを示す。
両者を比較することで、従来の書き方に慣れたユーザが新しい書き方を短期間で利用できる
ようになることを目指す。

従来の関数前半の if 文と後半の switch 文の組み合わせによる記述を取りやめて、新たに
Call????() 関数による記述を採用したのは以下のような理由による。
・ある動きを追加する時には、前半の if 文と後半の switch 文の2箇所に記述する必要が
　あり、編集作業が煩雑である。
・動きを指定する初期設定やパラメータの設定が、if 文と switch 文のどちらでもできて
　しまうので、確認やメンテナンスの妨げとなっていた。
・if 文と switch 文ともに、動きの種類が多くなると行数が多くなるため、ソースコードの
　可読性が低下してメンテナンスが難しくなる。従来の記述法の LessonMotion() は既に
　数百行になっており改善が必要とされていた。
・動きを選択する if 文では、直前の動きが○○なら次は□□という書き方しかできず、複数の
　処理を場合によって切り替えるのが複雑な記述になっていた。

新しい Call????() 関数による記述では以下のように改善された。
・一つの動きに関する記述は一つの Call????() 関数にまとめることができて管理が容易に
　なった。
・個々の動きに関する記述はすべて Call????() 関数内にあるので、LessonMotion() の行数
　が短くなり可読性が改善され、メンテナンスが容易になった。
・次の動きを指定する機能として、新たに act_next 変数を導入したことで、ある動きの終了
　時に次の動きを明示的に指定できるようになった。前の動きの結果によって次の動きを切り
　替える処理がわかりやすく書けるようになった。

注意する点として、新旧の記述で似ている部分があるが、細かい点で変更されている点がある
ため、新旧の記述を混ぜてプログラムを作成したり、新旧のプログラムから一部ずつ抜き出して
組み合わせるなどすると、誤動作の原因となるのでしないようにする。

以下に示す新旧の記述法によるソースコードは同じ機能を実現する。
動作を行う関数は仮想の関数として用意した ActLesson0(m, act_param) の1個だけとして、
処理の骨格だけを短く簡潔に示す例とした。

最初に従来の書き方を示す。

////////////////////////////////////////////////////////////////////////////////

// 今までの書き方

void LessonMotion(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[])
// m[] はモータコントローラのインスタンスへのポインタの配列、モータを直接動かすのに必要
// *c はRGBカメラのインスタンスへのポインタ、画像処理、画像を扱うために必要
// *d は深度(depth)カメラのインスタンスへのポインタ、深度画像を扱うために必要
// ft[] は力覚・トルクセンサのインスタンスへのポインタの配列
{
//  act_param[AP_STATE] = ACT_START;           // ここで実行すると毎回初期化になってまずいので不可

// act_num の次の値は(必要なら動作結果を act_param[] で受け取って)、次の if 文内で決める
    if (ACT_START == state) {                  // プログラム起動直後、最初はここから
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を覚えておく、後で必要になるかも
	act_num = ACT_INITPOSE;                // 最初は初期姿勢、立ち上がる
    }
//  if (ACT_MOVING == state) NOTHING;          // 動作継続中の時は理由がなければ指示を変更しない
    if (ACT_END == state) {                    // 直前の行動が終了したら
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を覚えておく、後で必要になるかも

	if (ACT_INITPOSE == act_last) {
            act_param[AP_NECK_HA] =  0.0; // 水平回転、0.0rad, 正面
            act_param[AP_NECK_VA] = -0.8; // 下向 0.8rad, 45.8366度
            act_num = ACT_LESSON0;
        }
	else if (ACT_ERR == act_last) act_num = ACT_RELAX; // エラー時の次の行動を決める
    }
    if (ACT_ERR == state) act_num = ACT_ERR; // 状態がエラーの時はエラー処理へ

//////////////////////////////////////////////////////////
// 
// 個々の行動の具体的な内容を定義する
// 必要なパラメータをセットして動きを実現する関数を呼び出す
// 
//////////////////////////////////////////////////////////
    
    switch (act_num) {
	case ACT_LESSON0 : // 動きの例、関節を指定して動かす
	    if (ACT_START == act_param[AP_STATE])
		fprintf(stderr, "ActLesson1(), 関節を指定して個別に回転させる\n");
	    act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
	    state = ActLesson0(m, act_param);
	    break;
//      case ACT_ERR : // ACT_ERROR -> ACT_ERR の変更に伴ってこの行は不要になった
        default : // どの case にもマッチしないとき、本来ここにはマッチしないはず
	    fprintf(stderr, "Error: Bad act_num %d specified, or other error occured.\n", act_num);
	    state = ACT_END;
            break;
    }
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

次に Call????() を使用する新しい書き方の例を示す。

// 新しい書き方

void LessonMotion(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[])
// m[] はモータコントローラのインスタンスへのポインタの配列、モータを直接動かすのに必要
// *c はRGBカメラのインスタンスへのポインタ、画像処理、画像を扱うために必要
// *d は深度(depth)カメラのインスタンスへのポインタ、深度画像を扱うために必要
// ft[] は力覚・トルクセンサのインスタンスへのポインタの配列
{
// 各種の行動を実行する順番や繋がりを Call????() で定義する
    
//  act_param[AP_STATE] = ACT_START;           // ここで実行すると毎回初期化になってまずいので不可

// 以下の4個の if 文で行う処理は、 Call????() を導入後変更されたので注意する。
// act_num, act_next の機能の変更に伴うものである

// state の取り得る値は以下の if 文の4種類。
// ACT_START == state は起動直後のみで、その後はこの状態にはならない。
//     代わりに act_param[AP_STATE] == ACT_START によって動作開始を表す。
// ACT_MOVING == state は動作中なので何もすることはない。
// ACT_ERR == stateはエラーが起きたときで、通常は関係ない。
// ACT_END == stateは動作終了で次の動作に移る。
    
    if (ACT_START == state) {                  // プログラム起動直後、最初「だけ」ここから
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_num = ACT_NONE;                    // 最初は指定していない
	act_last = ACT_NONE;                   // 直前の動作はまだない
	act_next = ACT_INITPOSE;               // 次の動作を指定、Call????() 用
    }
//  if (ACT_MOVING == state) NOTHING;          // 動作継続中の時は理由がなければ指示を変更しない
    if (ACT_ERR == state) {                    // 状態がエラーの時の処理
	act_last = act_num;                    // 直前の動作を記録
	act_param[AP_STATE] = ACT_START;       // 今までの動きを終了
	act_param[AP_TIMETOTAL] = 600.0;
	act_next = ACT_STAY;                   // 600秒間動きを停止、短かくしても可
	fprintf(stderr, "ACT_ERR was detected. 600 sec stopped.\n");
    }
    if (ACT_END == state) {                    // 直前の行動が終了したら次を決める
	act_param[AP_STATE] = ACT_START;       // 次の行動を最初から始めるための指示、必須
	act_last = act_num;                    // 直前の行動を記録、後で必要になるかもしれないので
    }

// 通常は以下の Call????() のどれかが条件にマッチして対応する動作が実行される
    
    if (CALLMATCH == CallLesson0(m)) return;
    if (CALLMATCH == Call????(m)) return;
    if (CALLMATCH == Call????(m)) return;

// どの Call????() にもマッチしないときの処理、本来ここには達しないはず
    fprintf(stderr, "Error: Bad act_next %d specified, or other error occured.\n", act_next);
    state = ACT_ERR;
    return;
}

// 以下はコメントを少なくして処理の内容を見やすくした CallLesson0() である。
// 初期設定の処理やパラメータが少なければこのくらいの行数になる。

int CallLesson0(C1 m[]) // Call????() の書き方の例
{
    int goflag = NO;
    if (ACT_LESSON0 == act_next) goflag = YES;
//  if (ACT_???? == act_last) goflag = YES;
    if (NO == goflag) return CALLTHRU;
    
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これは間違い、書かない
#ifdef CALLMSG
	fprintf(stderr, "CallLesson0(), Call????() の書き方の例\n");
#endif
	act_param[AP_TIMETOTAL] = 3.0;
	act_num = ACT_LESSON0;
    }

    state = ActLesson0(m, act_param);

    if (ACT_END == state) act_next = ACT_LESSON1;
    return CALLMATCH;
}

// 以下は同じ関数で詳しいコメントを付加して、処理の内容をわかりやすく示したものである。
/*
int CallLesson0(C1 m[]) // Call????() の書き方の例、コメント付
{
// 最初は起動するかどうかの判断
    int goflag = NO; // Act????() を起動するかどうかのフラグ
    if (ACT_LESSON0 == act_next) goflag = YES; // act_next が指定されていたら起動
//  if (ACT_???? == act_last) goflag = YES; // 必要なら直前の処理からも確認できる
    if (NO == goflag) return CALLTHRU; // 起動不要ならここで終わり、そのまま戻る
// 起動の判断ここまで、Act????() を実行する場合は以下に進む
    
// ここから初期設定、最初に1回だけ実行される    
    if (ACT_START == (int)act_param[AP_STATE]) { // 次行ではなくこちらを書く
//  if (ACT_START == state) { // これが成立するのはプログラム起動直後だけなので間違い
#ifdef CALLMSG // 次行の表示が不要なら const.h で CALLMSG をコメントアウト
	fprintf(stderr, "CallLesson0(), Call????() の書き方の例\n");
#endif
	act_param[AP_TIMETOTAL] = 3.0; // 動作時間 (sec)
//	act_param[AP_????1] = 1.0; // Act????() の実行に必要なパラメータを
//	act_param[AP_????2] = 2.0; // 何個でも設定できる 
	act_num = ACT_LESSON0; // 選択した動作の番号を登録
    }
// ここまで初期設定

// ここで Act????() を実行する
    state = ActLesson0(m, act_param);

// 実行終了なら次の動きを指定可能、この機能を使うと動作の繋がりの管理が楽
    if (ACT_END == state) act_next = ACT_LESSON1; // 必要なら次の動きを指定可能

// Act????() を実行したら返り値で呼び出し元に通知する 
    return CALLMATCH; // 起動したことを返り値で戻す
}
*/

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

Act????() 関数の書き方は従来から変更された点はない。よって、今までに開発された
関数はそのまま流用することができる。

int ActLesson0(C1 m[], float p[])
{
    static int timer = 0;
    float duration = p[AP_TIMETOTAL]; // 動いている時間(sec)
    int cycletotal = (int)(duration*100);

    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
	// A1J1 は1番の arm の1番の joint 、左前脚の肩
	m[A1J1].ctrlmode = MODENONE; // モータ1個にパラメータ3個、これは制御モード PID
        m[A1J1].duration = duration; // 回転時間
	m[A1J1].targetangle.data = -1.3; // 目標角度 radian
	
	m[A1J2].ctrlmode = MODENONE; // ここから2個めのモータのパラメータ
        m[A1J2].duration = duration;
	m[A1J2].targetangle.data = 1.3; // radian
        timer = 0;
    }    
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
