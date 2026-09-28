カメラ画像、深度画像の処理と物体認識

・カメラ画像から対象物体の３次元座標が求められるようになったので、
　今後は画像から求めた操作対象の座標を利用してアームや脚を
　動作させることができる。

・画像中のどの位置の点を対象として、脚を動かしたり、掴んだり
　すればよいのか、今後は画像処理結果から注目点を決める作業が
　重要になる。

・輪郭を求めるなどの基本的な画像処理は OpenCV の機能で簡単に
　実行できるが、その結果から注目点を決めるための処理を自分で
　考えて作成する必要がある。処理の内容は対象物（梯子、
　ジャングルジム、山腹、ボルダリング壁など）によって異なる。

・画像処理や物体認識を容易にするために物体の構成要素（梯子の
　横棒など）に区別しやすい色を付けたり、形状を認識しやすく
　変更するのは構わない。高度な画像処理を実現するのが研究の目的
　ではないので、画像処理を容易にして、その先の動作の生成に
　注力できるようにする。

・ロボットが動作中にカメラではどのようなシーンが見えていて、
　対象物体はどのように見えるのか、window に表示されるカメラ
　映像を観察して、必要な処理の内容を考えてプログラムとして
　実現する。

・画像処理をした結果を画面に表示したときの見やすさと、
　その後の処理の難易は比例しないことがあるので、人の目で
　見た時の印象だけで安易に判断しない。

-----

カメラ画像(RGB画像、カラー画像)の取得とその処理に関する機能は
prj4/src/imageclass.h
に記述されている。
class RGBCam
{
    ....
}
に変数や関数が定義されている。

深度画像(Depth画像、距離画像)の取得とその処理に関する機能は
prj4/src/depthclass.h
に記述されている。
class DepthCam
{
    ....
}
に変数や関数が定義されている。

OpenCV を利用するフィルタ、輪郭抽出、色認識、ラベリングによる
物体検出などの機能は
prj4/src/image.h
prj4/src/image.cpp
に記述されている。

----- 2026 -------------------------------------------------------
※ 2026年5月の時点でカメラ画像 
(RGB(カラー)画像、深度(距離)画像) を処理する関数の構成が変わった。
以前は他の関数と同じ記述形式の
int CallVision(C1 m[], RGBCam *c, DepthCam *d, int next);
と
int ActVision(C1 m[], RGBCam *c, DepthCam *d, float p[]);
の組合せで実現されていたが、これら2個の関数を統合して現在は以下の
関数だけで実現されている。Act????() は省略された。
int CallNVisionColor(RGBCam *c, DepthCam *d, int next); // カラー画像
int CallNVisionDepth(RGBCam *c, DepthCam *d, int next); // 深度画像
名前に NV(New Vision) が含まれるのが特徴である。以前に作られた
名前に Vision が含まれる関数が多数存在するが、現在は NV 系の上記2個の関数
だけ理解すればよい。内部で利用する
imFindObjs(), imLaplacian(), imLabel(), imCogCorners(), imFindColorObj(),
imShow(), imHough(), LocalXyz() など、下位レベルの関数群はこの文書の後の
説明と互換性があるので以降の記述内容は参考にできる。
上記 im????() の関数群はパラメータの設定で動作の内容を変更できる。
関数内の記述内容は近年変更されてないものが多いので、上位レベル
int CallNVisionColor(RGBCam *c, DepthCam *d, int next); // カラー画像
int CallNVisionDepth(RGBCam *c, DepthCam *d, int next); // 深度画像
からの利用方法を理解すればそれで使いこなせる可能性が高い。
----- 2026 -------------------------------------------------------

具体的な利用の仕方は lessonneo.cpp の CallVision(), action.cpp の
ActVision() やそこから呼び出される image.cpp の様々な関数を見て
内容を理解するとよい。

-----

画像の取得、処理から3次元座標を求めて、次の動作を行うまでの処理の流れ

★
簡潔にまとめると、ActVision() を実行することで画像処理から物体検出、
3次元座標の推定までできるようになっている。

1. カメラ画像は1秒間に30回更新されるので、最新のデータに対して
　　画像処理を行う。imageclass.h にあるサブスクライバの関数で画像を
    取得して、action.cpp の ActVision() が実行された時に画像処理が
    行われる。

2. ActVision() では最新の画像に対して、imFindObjs() を呼び出す。
　　今は例として深度画像から Laplacian
　　フィルタにより物体の輪郭を検出する関数 imLaplacian() を利用して
    画像中の物体の領域を求める処理になっている。
    フィルタは他に、色検出の imFindColorObj()、imSobel() フィルタがある。
　　フィルタの結果に対してラベリングの処理を施して指定した色や輪郭で
　　囲まれる領域を検出できる。その後は各領域に対して重心と外接長方形の
　　座標を求める。ここまでの標準で用意されている処理以外の独自の処理が
　　必要な場合は自分で独自の関数を作成することになる。
　　ラベリングの処理により対象物体が複数個あってもすべて検出できるように
　　なったため、2個以上の物体が検出された場合に1個を選択する必要があれば
　　自分のプログラムで決める必要がある。ここまでが2次元の画像処理となる。

3. 2次元画像中で求めた物体の座標をix, iy とすると、次に3次元座標を求める
　　関数 d -> LocalXyz() により
　　ローカル座標系の3次元座標を x, y, z として求める。この座標は
　　ローカル座標系の原点(ロボット本体の重心) からの座標で、アームや
　　ハンドを動かすときの目標座標としてそのまま使用できる。

4. 3次元座標 x, y, z を活用して必要な動作を行う。lessonneo.cpp では
　　ActGrabRelease() が相当して、対象物体を掴んで持ち上げて放す一連の
　　動作を実行する。x, y, z は物体の見えている面の画像上の重心なので
　　ロボットからの距離は物体そのものの重心よりも短くなる。そのため
　　掴むなどの動作では x, y, z よりも少し遠目の座標を指定する必要がある。

一連の処理で求めた
結果で重要なのは、2次元画像から検出した領域(物体)の個数、重心と外接長方形の
座標、3次元空間中の物体の座標であるが、以下に示すソースファイルでは
// ★ 次の?行は重要
というコメントで上記の結果を記録している場所を示している。
画像処理の次の処理では、これらの値が必要になるので、どの変数に記録されている
かは★　付きのコメントで確認するとよい。

-----

プログラムにおける画像処理の実際

全体の処理の流れ

モノを見つけて掴んで持ち上げるデモの動きで利用されている画像処理の
関数は次の上から下の順番で呼び出されて実行される。

------------------------------------------------------
lessonneo.cpp
-> LessonMotion() 動作の選択
-> CallVision() 画像処理を選択、起動
->
action.cpp
-> ActVision() 画像処理の実行、以下の関数を呼び出して物体の個数や座標を記録する
->
image.cpp
-> imFindObjs() 画像処理を実行、画像(注1)中の物体を検出して個数や座標を調べる
  -> imLaplacian() Laplacian フィルタをかける (注2)。膨張と収縮により孤立点を統合する
  -> imLabel() ラベリング処理で連続した領域(物体)を分離し、個数を調べる 
  -> imCogCorners() 検出した領域の重心と外接長方形の座標を求める
-> 
depthclass.h
-> LocalXyz() 検出した重心の座標から3次元空間中のローカル座標を求める
->
モノを1個以上検出した場合は以下の処理に続く
lessonneo.cpp
-> LessonMotion() 動作の選択
-> CallGRabRelease() モノを掴んで持ち上げる動作を選択、起動
->
action.cpp
-> ActGrabRelease() 上記の処理の実行、以下略
->

(注1) 画像はカラー画像、深度画像のどちらかを選択する
(注2) 他に色による検出を行う imFindColorObj(), Sobel 変換を行う imSobel()
      も利用できる
------------------------------------------------------


以下では lessonneo.cpp, action.cpp, image.cpp の関係が深い部分を
示してどのようにプログラムを記述すればよいか説明する。
実際のソースファイルでは多くのコメントが
付してあるが、ここでは見やすくするために関係ない細かい処理や
コメントは除いている。代わりに ※  で説明の文章を付加する。

// lessonneo.cpp

// LessonMotion()

※
カメラ画像と深度画像を LessoniMotion() 内で扱えるようにするために以前より引数が増えた。
m[] はモータコントローラのインスタンスへのポインタの配列、モータを直接動かすのに必要
*c はRGBカメラのインスタンスへのポインタ、画像処理、画像を扱うために必要
*d は深度(depth)カメラのインスタンスへのポインタ、深度画像を扱うために必要
ft[] は力覚・トルクセンサのインスタンスへのポインタの配列

void LessonMotion(C1 m[], RGBCam *c, DepthCam *d, FTSENSOR ft[])
{
※
カメラ画像を利用して赤い物体の座標を求め、ハンドで掴んで持ち上げるデモ
初期姿勢 -> カメラを下に向けて -> ★ 画像から物体の位置を求め -> ハンドを動かして
物体を持ち上げて落とす -> 次の動き
関数の冒頭部分は省略してある。
このデモに関係するのは次の 3 個の動き、Call????() の詳細については READMEcall.txt
を参照する

    if (CALLMATCH == CallNeck(m)) return;
    if (CALLMATCH == CallVision(m, c, d)) return;
    if (CALLMATCH == CallGrabRelease(m, d)) return;
}

※
Call????() 関数の詳細は READMEcall.txt を参照する。画像処理に関係するコメントのみ記す

int CallVision(C1 m[], RGBCam *c, DepthCam *d) // カメラ画像から色や輪郭を用いて物体を検出する
{
    int goflag = NO;
    if (ACT_VISION == act_next) goflag = YES;
//  if (ACT_???? == act_last) goflag = YES;
    if (NO == goflag) return CALLTHRU;

    if (ACT_START == (int)act_param[AP_STATE]) {
#ifdef CALLMSG
	fprintf(stderr, "CallVision(), カメラ画像から色や輪郭を用いて物体を検出する\n");
#endif
	act_param[AP_TIMETOTAL] = 0.1;
// 次の行は「深度画像」から「輪郭」により「LAPLACIANフィルタ」で輪郭検出する場合の記述例
	act_param[AP_FILTER] = LAPLACIAN; act_param[AP_IMGORCOL] = DEPTH;
// 次の行は「カラー画像」から「赤色」の物体を「色検出」で調べる場合の記述例	
//	act_param[AP_FILTER] = FOC; act_param[AP_IMGORCOL] = RED;
	act_num = ACT_VISION;
    }
    state = ActVision(m, c, d, act_param);
    if (ACT_END == state) {
	if (0 < act_param[AP_OBJFIND]) { // 1個以上見つけたら
	    act_next = ACT_GRABRELEASE; // それを掴む、持ち上げて、放す
	}
	else { // 見つからなかったら
	    act_next = ACT_LESSON1; // 別の動作
	}
    }
    return CALLMATCH;
}

// action.cpp

// ActVision()

※
ソースファイルではコメントアウトされた内容が多くあるが、ここでは機能の本質が
わかるように画像処理に関係する内容のみ示す。
Act????() の詳細は、READMEprogramming.txt を参照する。

// カラー画像、深度画像を計測できるセンサの画像から色や輪郭で物体を検出する
int ActVision(C1 m[], RGBCam *c, DepthCam *d, float p[])
{
    static int timer = 0;
    static int count = 0;
    static float ikparam[IKNUMPARAM];
    char fname[64];
    float duration = p[AP_TIMETOTAL]; 
    int cycletotal = (int)(duration*100);

// 初期設定
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) {
#ifdef ACTEXEC
	fprintf(stderr, "start ActSuzukiVision()\n");
#endif
        timer = 0;
    }
// ここからが画像処理の本体
    if (0 == (timer % cycletotal)) {
	float x, y, z;            // 計算後のローカル座標を記憶する変数
	float ha = m[A7J1].anglenow.data; // 首の現在の水平回転角度(radian)、座標の補正に必要
	float va = m[A7J2].anglenow.data; // 首の現在の垂直回転角度(radian)、座標の補正に必要
	int filter = (int)p[AP_FILTER]; // LAPLACIAN などフィルタや色検出を指定
	int imgORcol = (int)p[AP_IMGORCOL]; // DEPTH など画像の種類または色を指定
	cv::Mat in;
	if (DEPTH == imgORcol) in = d -> cv_ptr -> image; // DEPTH
	else                   in = c -> cv_ptr -> image; // COLOR(RGB), RED, GREEN, ...
	int objs;                            // 見つけた領域、物体の個数

// 次の処理で画像処理を行い、条件に合う領域を検出する
// ★ 次の行は重要、2次元画像から検出した領域の重心と外接長方形の座標が 
// c -> fareacog, c -> fareacorners に記録される
        objs = imFindObjs(&in, filter, imgORcol, SHOW, "imFindObjs",
			  c -> fareacog, c -> fareacorners);
// ★ 次の2行は重要
        c -> fareanum = objs; // 検出した2次元画像中の領域の個数
	d -> fobjnum  = objs; // 検出した3次元空間中の物体の個数、両者は同じ値になる

// 入力画像とフィルタでの処理は次の組み合わせが可能
/*
// (1) 指定した色の領域(物体)を抽出する、個々の領域に対して重心と外接長方形を求める	
	objs = imFindObjs(&in_c, FOC, RED, SHOW, "ColorObjsResult", cog, corners);
// (2) カラー画像から Sobel フィルタで輪郭を求める、個々の領域に対して重心と外接長方形を求める	
	objs = imFindObjs(&in_c, SOBEL, COLOR, SHOW, "SobelColorResult");
// (3) 距離画像から Sobel フィルタで輪郭を求める、個々の領域に対して重心と外接長方形を求める	
	objs = imFindObjs(&in_d, SOBEL, DEPTH, SHOW, "SobelDepthResult");
// (4) カラー画像から Laplacian フィルタで輪郭を求める、個々の領域に対して重心と外接長方形を求める	
	objs = imFindObjs(&in_c, LAPLACIAN, COLOR, SHOW, "LaplacianColorResult", cog, corners);
// (5) 距離画像から Laplacian フィルタで輪郭を求める、個々の領域に対して重心と外接長方形を求める	
	objs = imFindObjs(&in_d, LAPLACIAN, DEPTH, SHOW, "LaplacianDepthResult", cog, corners);
*/

// 2次元画像の処理はここまで、この後は3次元座標の計算

	if (0 < objs) {        // もし注目領域、物体があれば
	    int a[MAXOBJS];
	    int n = 0;
	    for (int i = 0; i < objs; i++) {
		a[n++] = c -> fareacog[i][0]; // x 座標、画像中に + マークを表示するため
		a[n++] = c -> fareacog[i][1]; // y 座標
// カメラ画像の window で注目する画素の位置（重心）に + マークを描画する、不要なら省略可
		c -> SetMark(objs, a);
// 次の処理で画像中の座標から3次元ローカル座標を求める
		d -> LocalXyz(c -> fareacog[i][0], c -> fareacog[i][1], &x, &y, &z, ha, va);
// ★ 次の4行は重要
// 検出した個々の物体の3次元ローカル座標を配列に記憶する
		d -> fobjxyz[i][0] = x;
		d -> fobjxyz[i][1] = y;
		d -> fobjxyz[i][2] = z;
	    }
//	    c -> ResetMark(); // 登録したマークを削除して描画をやめるとき
	    p[AP_OBJFIND] = objs; // 物体発見のフラグ、見つけた個数
	}
	else {
	    p[AP_OBJFIND] = 0;  // 物体を見つけなかった
	}
    }
// ここまで画像処理の本体

// 以下は終了処理
    timer++;
    if (timer < cycletotal) { // 動作継続中の処理、次行のように書いてはいけない
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // 動作終了、タイマをクリアして ACT_END を返り値に
	timer = 0;
	p[AP_STATE] = ACT_END;
#ifdef ACTEXEC
	fprintf(stderr, "end ActVision()\n");
#endif
	return ACT_END;
    }
}

// image.cpp

// imFindObjs()

※
ソースファイルではコメントアウトされた内容が多くあるが、ここでは機能の本質が
わかるように画像処理に関係する内容のみ示す。

// 入力画像から「色」または「輪郭」により、物体を検出する。複数物体に対応、
// 検出した物体の外接長方形と重心の座標を求める
int imFindObjs(const cv::Mat *input, const int filter, const int imgORcol, const int show,
	       const char winname[], int cog[][XY], int corners[][LURL][XY])
/*
cv::Mat *input, // 入力画像
int filter,     // 使うフィルタの種類, FCO(FindColorObj), SOBEL, LAPLACIAN のどれか
int imgORcol,   // 入力画像または色の指定, filter に FCO を使うときは RED, BLUE など「色」
                // を指定する。filter に SOBEL, LAPLACIAN を使うときは COLOR: カラー画像, 
                // または DEPTH: 距離画像 のどちらか、「画像の種類」を指定する。
int show,       // 結果を表示するか否かのフラグ, SHOW, NOSHOW
char winname[], // 結果を表示するときの window の名前
int cog[][XY],  // 見つけた物体の重心を記録、画像中の座標
int corners[][LURL][XY], // 見つけた物体の左上右下の座標を記録、画像中の座標
*/
{
    int color = imgORcol; // 以下の imFindColorObj() では色と解釈される
                          // sobel, laplacian フィルタでは深度画像と解釈される
    int dummy; // 仮の返り値を受け取る
    cv::Mat output(input -> rows, input -> cols, CV_8UC1);

    switch (filter) { // ここで使用するフィルタを選択する
    case FOC : 
        dummy = imFindColorObj(input, &output, color);    // 指定色の物体を見つける
	break;
    case SOBEL : 
        dummy = imSobel(input, &output, imgORcol);        // Sobel、RGB、膨張と収縮
	break;
    case LAPLACIAN : 
        dummy = imLaplacian(input, &output, imgORcol);    // Laplacian、RGB、膨張と収縮
	break;
    default:
        return 0;
    }
    int label[ROWS][COLS] = {{0}};      // ラベリングの結果を画素毎に記録する配列
    int objs = imLabel(&output, label); // ラベリングにより物体を分離する処理
// ★ 次の行は重要、重心と外接長方形の座標が cog, corners に記録される
    int rcode = imCogCorners(&output, objs, label, cog, corners); // 重心と外接長方形を求める関数
                                                                  // 返り値は領域の個数
    }
    return rcode;
}

// imFindColorObj()

※
ソースファイルではコメントアウトされた内容が多くあるが、ここでは機能の本質が
わかるように画像処理に関係する内容のみ示す。

int imFindColorObj(const cv::Mat *input, cv::Mat *output, int color)
// 指定した色 color の物体(画素の集まり)を見つけて、結果の画像を返す
// 返り値は見つけたか、1以上なら見つかった、0 なら見つからなかった。
{
    cv::Mat out2; // 処理途中の画像を入れる配列
    
    for (int y = 0; y < input -> rows; y++) {
        for (int x = 0; x < input -> cols; x++) {
	    Uchar b = input -> at<cv::Vec3b>(y, x)[0];
	    Uchar g = input -> at<cv::Vec3b>(y, x)[1];
	    Uchar r = input -> at<cv::Vec3b>(y, x)[2];
	    if (YES == TestColor2(r, g, b, color, 4)) { // 色の条件に合ったら白色
		output -> at<Uchar>(y, x) = 255; 
	    }
	    else {
		output -> at<Uchar>(y, x) = 0; // そうでなければ黒色
	    }
	}
    }

    out2 = output -> clone();

//  孤立点、離れ小島を吸収合併する処理、膨張と収縮
    cv::dilate(out2, out2, cv::noArray(), cv::Point(-1, -1), 2); // 膨張 2 回
    cv::erode( out2, out2, cv::noArray(), cv::Point(-1, -1), 2); // 収縮 2 回

    *output = out2.clone();

    return 0; // 必要なら何らかの処理結果の値を返り値とする
}

// imSobel()

※
ソースファイルではコメントアウトされた内容が多くあるが、ここでは機能の本質が
わかるように画像処理に関係する内容のみ示す。

int imSobel(const cv::Mat *input, cv::Mat *output, int type)
// 画像処理の例、Sobel フィルタで輪郭を求める。
// cv::Mat *input,  処理対象の入力画像
// cv::Mat *output, 次の処理のために処理結果の画像が記録される
// int type, 画像の種類、カラーか距離、RGB or DEPTH
{
    cv::Mat gray; // グレイスケールに変換した画像用
    cv::Mat out2; // gray を定数倍した画像、距離画像では200倍するので
    
// Gray 濃淡画像の場合（距離画像）    
    if (DEPTH == type) {
      cv::Sobel(*input, *output, CV_32FC1, 1, 1, 3, 1); // Sobel フィルタ、結果はCV_32FC1
      gray = output -> clone();
// 次は背景の処理
      for (int y = 0; y < gray.rows; y++) { // 背景を値 0.0 にして輪郭の値から離す処理
        for (int x = 0; x < gray.cols; x++) {
	    float f = gray.at<float>(y, x);
// 以下の閾値はいろいろ実験して良さそうな値にしてある。変えると結果が変わる。
//	    if (1.0 == f)                 gray.at<float>(y, x) = 0.0; // カラー画像用
//	    if (0.9999 < f && f < 1.0001) gray.at<float>(y, x) = 0.0; // これはダメ
	    if (0.999  < f && f < 1.001 ) gray.at<float>(y, x) = 0.0; // これが良さそう
//	    if (0.99   < f && f < 1.01  ) gray.at<float>(y, x) = 0.0; // これでも大体良い
	}
      }
      cv::convertScaleAbs(gray, out2, 200.0, 0.0); // 各要素 * 200.0 + 0.0、絶対値、符号無8bitへ
    }
// Gray 濃淡画像ここまで
    
// RGB カラー画像の場合
    else if (RGB == type) {
      cv::Sobel(*input, *output, CV_32F, 1, 1, 3, 1); // Sobel フィルタ、結果はCV_32F
      cv::cvtColor(*output, gray, cv::COLOR_BGR2GRAY); // RGB が不要なら Gray にすると後が楽
// 次は背景の処理
      for (int y = 0; y < gray.rows; y++) { // 背景を値 0.0 にして輪郭の値から離す処理
        for (int x = 0; x < gray.cols; x++) {
	    float f = gray.at<float>(y, x);
// 以下の閾値はいろいろ実験して良さそうな値にしてある。変えると結果が変わる。
	    if (1.0 == f)                 gray.at<float>(y, x) = 0.0; // カラー画像はこれで
//	    if (0.999  < f && f < 1.001 ) gray.at<float>(y, x) = 0.0; // 距離画像用
//	    if (0.99   < f && f < 1.01  ) gray.at<float>(y, x) = 0.0; // 距離画像用
	}
      }
//    out2 = gray.clone(); カラー画像は定数倍しないのでこれでもよい
      cv::convertScaleAbs(gray, out2, 1.0, 0.0); // 各要素 * 1.0 + 0.0、絶対値、符号無8bitへ
    }
// RGB カラー画像ここまで

    cv::threshold(out2, out2, 0, 255, cv::THRESH_BINARY); // 2値化、背景を値０にしたのでこれで

//  離れ小島を吸収合併する処理、膨張と収縮
    cv::dilate(out2, out2, cv::noArray(), cv::Point(-1, -1), 2); // 膨張 2 回
    cv::erode( out2, out2, cv::noArray(), cv::Point(-1, -1), 2); // 収縮 2 回

    *output = out2.clone();

    return 0; // 必要なら何らかの処理結果の値を返り値とする
}

// imLaplacian()

※
ソースファイルではコメントアウトされた内容が多くあるが、ここでは機能の本質が
わかるように画像処理に関係する内容のみ示す。

int imLaplacian(const cv::Mat *input, cv::Mat *output, int type)
// 画像処理の例、Laplacian フィルタで輪郭を求める。
// cv::Mat *input,  処理対象の入力画像
// cv::Mat *output, 次の処理のために処理結果の画像が記録される
// int type, 画像の種類、カラーか距離、RGB or DEPTH 
{
    cv::Mat gray; // グレイスケールに変換した画像用
    cv::Mat out2; // gray を定数倍した画像、距離画像では200倍するので

// Gray 濃淡画像の場合（距離画像）    
    if (DEPTH == type) {
      cv::Laplacian(*input, *output, CV_32FC1, 1, 1, 1); // Laplacian フィルタ、輪郭はGray
      gray = output -> clone();
// 次は背景の処理
      for (int y = 0; y < gray.rows; y++) { // 背景を値 0.0 にして輪郭の値から離す処理
        for (int x = 0; x < gray.cols; x++) {
	    float f = gray.at<float>(y, x);
// 以下の閾値はいろいろ実験して良さそうな値にしてある。変えると結果が変わる。
//	    if (1.0 == f)                 gray.at<float>(y, x) = 0.0; // カラー画像用
//	    if (0.9999 < f && f < 1.0001) gray.at<float>(y, x) = 0.0; // これはダメ
	    if (0.999  < f && f < 1.001 ) gray.at<float>(y, x) = 0.0; // これが良い
//	    if (0.99   < f && f < 1.01  ) gray.at<float>(y, x) = 0.0; // これでも大体良い
	}
      }
      cv::convertScaleAbs(gray, out2, 200.0, 0.0); // 各要素 * 200.0 + 0.0、絶対値、符号無8bitへ
    }
// Gray 濃淡画像ここまで
    
// RGB カラー画像の場合
    else if (RGB == type) {
      cv::Laplacian(*input, *output, CV_32F, 1, 1, 1); // Laplacian フィルタ、輪郭はRGB
      cv::cvtColor(*output, gray, cv::COLOR_BGR2GRAY); // RGB が不要なら Gray にすると後が楽
// 次は背景の処理
      for (int y = 0; y < gray.rows; y++) { // 背景を値 0.0 にして輪郭の値から離す処理
        for (int x = 0; x < gray.cols; x++) {
	    float f = gray.at<float>(y, x);
// 以下の閾値はいろいろ実験して良さそうな値にしてある。変えると結果が変わる。
	    if (1.0 == f)                 gray.at<float>(y, x) = 0.0; // カラー画像はこれで
//	    if (0.999  < f && f < 1.001 ) gray.at<float>(y, x) = 0.0; // 距離画像用
//	    if (0.99   < f && f < 1.01  ) gray.at<float>(y, x) = 0.0; // 距離画像用
	}
      }
//    out2 = gray.clone(); カラー画像は定数倍しないのでこれでもよい
      cv::convertScaleAbs(gray, out2, 1.0, 0.0); // 各要素 * 1.0 + 0.0、絶対値、符号無8bitへ
    }
// RGB カラー画像ここまで

    cv::threshold(out2, out2, 0, 255, cv::THRESH_BINARY); // 2値化、背景を値０にしたのでこれで

//  離れ小島を吸収合併する処理、膨張と収縮
    cv::dilate(out2, out2, cv::noArray(), cv::Point(-1, -1), 3); // 膨張 2 回
    cv::erode( out2, out2, cv::noArray(), cv::Point(-1, -1), 3); // 収縮 2 回
    
    *output = out2.clone();

    return 0; // 必要なら何らかの処理結果の値を返り値とする
}

// imLabel()

※
ソースファイルではコメントアウトされた内容が多くあるが、ここでは機能の本質が
わかるように画像処理に関係する内容のみ示す。

int imLabel(const cv::Mat *input, int label[ROWS][COLS])
// ラベリング、隣接した画素から輪郭を求め物体を分離する、膨張と収縮も
// input は　Uchar x 1ch, 符号無 8bit, 640x480 であること
{
    const int FSW = 1; // 造語 Filter Scan Width, 注目画素の周囲を調べる幅、普通は１
    int k = 1; // 次のラベルの値、１からはじまり最終的に物体の個数＋１になるようだ
    int flag = 0; // 途中でラベル値の変更があったかを表すフラグ

    do {
	flag = 0;
	for (int y = FSW; y < input -> rows - FSW; y++) { // ３ｘ３領域内をスキャンする
	    for (int x = FSW; x < input -> cols - FSW; x++) {
//		Uchar uc = input -> at<Uchar>(y, x);
		if (0 < input -> at<Uchar>(y, x)) { // 0 より大きな値なら輪郭候補で以下の処理
		    int min = k; // ラベルの最小値を仮に次の値ｋにしておく
		    int sum = 0; // ラベルの合計値
		    for(int i = -1*FSW; i <= FSW; i++){
			for(int j = -1*FSW; j <= FSW; j++){
			    sum += label[y+i][x+j]; // ３ｘ３領域内のラベル値の合計
			    if (0 < label[y+i][x+j] && label[y+i][x+j] <= min) {
                                min = label[y+i][x+j]; // ３ｘ３領域内のラベル最小値
			    }
			}
		    }
		    if (sum == 0) { // ３ｘ３領域でラベルが一つも付いていない
                        label[y][x] = k++; // ラベルの値を割当、ｋの更新
                        flag++; // ラベルの変更があった
                    }
		    else { // (sum != 0) ３ｘ３領域で既にラベルが付いていた 
			if (min == label[y][x]) ; // 既にラベルが付いていれば何もしない
			else { // ラベルが付いてなければ
			    label[y][x] = min; // ラベルの最小値を割当
			    flag++; // ラベルの変更があった
			}
		    }
		}
//		if (0 < label[y][x]) fprintf(stderr, "%d", label[y][x]);
	    }
	}
    } while (flag > 0); // ラベルの変更がされなくなるまで繰り返す

    return k-1; //「統合前」の物体の個数、無駄に大きいが配列中に残るラベル値が不連続なので
}

// imCogCorners()

※
ソースファイルではコメントアウトされた内容が多くあるが、ここでは機能の本質が
わかるように画像処理に関係する内容のみ示す。

// 各物体の重心、外接長方形を求める ////////////////////////////////////////////
int imCogCorners(const cv::Mat *input, int noo, int label[ROWS][COLS], int cog[][XY],
		 int corners[][LURL][XY])
/*
cv::Mat *input;        // ラベル付けに使用した画像 Uchar x 1ch, 符号無 8bit, 640x480
int noo;               // 物体の個数 Number of Objects
int label[ROWS][COLS]; // ラベル付けの結果、値が不連続なのに注意
int cog[][XY];          // 結果、Center of Gravity 求めた重心を記録して戻す
int corners[][LURL][XY];   // 結果、求めた外接長方形の左上と右下の座標を記録して戻す
*/
{
    int l; // 探索するラベルの値
    int noo2 = 0; // この関数で求めた物体の個数 Number of Objects
    cv::Mat result;
    cv::cvtColor(*input, result, cv::COLOR_GRAY2RGBA);

    for (l = 1; l <= noo; l++) { // ラベルの値を順番に変えて探索する、背景を除くため1から
	int sumx = 0, sumy= 0; // 重心計算用
	int count = 0; // 見つけた画素の個数
	int minx = 0, miny = 0, maxx = 0, maxy = 0; // 外接長方形のカドを探す
	for (int y = 0; y < input -> rows; y++) {
	    for (int x = 0; x < input -> cols; x++) {
		if (l == label[y][x]) {
		    sumx += x;
		    sumy += y;
		    if (0 == count) {
			minx = x; miny = y;
			maxx = x; maxy = y;
		    }
		    else {
			if (x < minx) minx = x;
			if (y < miny) miny = y;
			if (maxx < x) maxx = x;
			if (maxy < y) maxy = y;
		    }
		    count++;
		}
	    }
	}
	if (0 != count) { // 領域が見つかった
	    noo2++;
	    sumx /= count; // 座標の和を個数で除算して重心を求める
	    sumy /= count;
// 呼び出し元に結果を戻すために記録する
	    cog[noo2-1][0] = sumx;
	    cog[noo2-1][1] = sumy;
	    corners[noo2-1][0][0] = minx;
	    corners[noo2-1][0][1] = miny;
	    corners[noo2-1][1][0] = maxx;
	    corners[noo2-1][1][1] = maxy;
	}
    }

    fprintf(stderr, "NumOfObjs=%d\n", noo2); // 背景は除く
    return noo2; // 領域の個数を返り値とする
}
