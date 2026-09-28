※
この文書の内容は古くなったので読む必要はなくなった。
最新の内容は READMEvision.txt を確認する。

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

-----

カメラ画像(RGB画像、カラー画像)とその処理に関する機能は
src/imageclass.h
に記述されている。
class RGBCam
{
    ....
}
に変数や関数が定義されている。

深度画像(Depth画像、距離画像)とその処理に関する機能は
src/depthclass.h
に記述されている。
class DepthCam
{
    ....
}
に変数や関数が定義されている。

具体的な利用の仕方は suzuki.cpp の ActSuzukiVision() や
SuzukiVSobel() などの関数を読むとよい。

-----

画像の取得、処理から3次元座標を求めて、次の動作を行うまでの処理の流れ

1. カメラ画像は1秒間に30回更新されるので、最新のデータに対して
　　画像処理を行う。suzuki.cpp では ActSuzukiVision() が相当する。
2. ActSuzukiVision() では最新の画像に対して、例として赤色の物体を
　　検出する関数 SuzukiVFindColorObj() を利用して物体の重心座標を
　　ix, iy として求める。
　　SuzukiVFindColorObj() 以外の処理を行う場合は独自の関数を作成する。
　　目標座標 ix, iy を決めるために、事前にどのような処理が必要に
　　なるかは各自考える。
　　SuzukiVFindColorObj() では対象物体を1個と仮定して処理しているため、
　　目標座標が1組となっているが、場合によっては2組以上の目標座標が
　　必要となる(例、左右のアームで異なる作業をする)ことがある。
　　その場合は必要な数の目標座標を求める。
3. 求めたix, iy に対して、3次元座標を求める関数 d -> LocalXyz() により
　　ローカル座標系の3次元座標を x, y, z として求める。この座標は
　　ローカル座標系の原点(ロボット本体の重心) からの座標で、アームや
　　ハンドを動かすときの目標座標としてそのまま使用できる。
　　ここまでが画像処理となる。
4. 3次元座標 x, y, z を活用して必要な動作を行う。suzuki.cpp では
　　ActSuzukiGrab() が相当して、対象物体を掴んで持ち上げて放す一連の
　　動作を実行する。x, y, z は物体の見えている面の画像上の重心なので
　　ロボットからの距離は物体そのものの重心よりも短くなる。そのため
　　掴むなどの動作では x, y, z よりも少し遠目の座標を指定する必要がある。

-----

プログラムにおける画像処理の実際

以下では suzuki.cpp を例としてどのようにプログラムを記述
すればよいか説明する。実際のソースファイルでは多くのコメントが
付してあるが、ここでは見やすくするために関係ない細かい処理や
コメントは除いている。代わりに ※  で説明の文章を付加する。

// suzuki.cpp

※  カメラ画像と深度画像を SuzukiMotion() 内で扱えるようにするために
　　引数が増えた。これまでの記述と互換性がなくなったので注意する。

void SuzukiMotion(C1 m[], RGBCam *c, DepthCam *d)
{

※  カメラ画像を利用して赤い物体の座標を求め、ハンドで掴んで持ち上げるデモ
　初期姿勢 -> カメラを下に向けて -> ★ 画像から物体の位置を求め -> ハンドを動かして
　物体を持ち上げて落とす -> 休憩 -> ★ に戻る

    if (ACT_END == state) {                    // 直前の行動が終了したら
	if (ACT_INITPOSE == act_last) {
	    act_param[AP_NECK_VA] = -0.8; // 首を下に曲げる
	    act_num = ACT_NECK;
	}
	if (ACT_NECK == act_last) act_num = ACT_SUZUKIVISION;

	if (ACT_SUZUKIVISION == act_last) {
	    if (YES == act_param[AP_OBJFIND]) {
		act_num = ACT_SUZUKIGRAB; // 何かものを見つけたら掴む
	    }
	    else {
		act_param[AP_TIMETOTAL] = 1;
		act_num = ACT_STAY;       // 無ければおやすみ
	    }
	}
	if (ACT_SUZUKIGRAB == act_last) {
	    act_param[AP_TIMETOTAL] = 2;
	    act_num = ACT_STAY;
	}
	if (ACT_STAY == act_last) act_num = ACT_SUZUKIVISION;
    }

//////////////////////////////////////////////////////////
// 
// 個々の行動の具体的な内容を定義する
// 必要なパラメータをセットして動きを実現する関数を呼び出す
// 
//////////////////////////////////////////////////////////

※  今回重要なのは ACT_SUZUKIVISION と ACT_SUZUKIGRAB の２個の動き
　ACT_SUZUKIGRABでは５種類の動作を組み合わせて、アームを対象物まで
　動かして掴んで持ち上げて放すまで行っている。

    switch (act_num) {
        case ACT_INITPOSE : // 初期姿勢、立ち上がる
	    state = ActInitialPose(m, act_param);
	    break;
	case ACT_SUZUKIVISION : // 画像処理
	    state = ActSuzukiVision(m, c, d, act_param);
	    break;
	case ACT_SUZUKIGRAB : // ものをつかむ
	    act_param[AP_TIMETOTAL] = 7.0; // 動作時間 (sec)
	    act_param[AP_TIME1] = 1.0; // 動作時間 (sec)
	    act_param[AP_TIME2] = 2.0; // 動作時間 (sec)
	    act_param[AP_TIME3] = 1.0; // 動作時間 (sec)
	    act_param[AP_TIME4] = 2.0; // 動作時間 (sec)
	    act_param[AP_TIME5] = 1.0; // 動作時間 (sec)
	    state = ActSuzukiGrab(m, act_param);
	    break;
        case ACT_NECK : // 首を動かす
	    act_param[AP_TIMETOTAL] = 2.0; // 合計動作時間 (sec)
	    state = ActNeck(m, act_param);
	    break;
        case ACT_STAY : // 現状維持
//	    act_param[AP_TIMETOTAL] = 5.0; // 合計動作時間 (sec)
	    state = ActStay(m, act_param);
	    break;
    }
}

int ActSuzukiVision(C1 m[], RGBCam *c, DepthCam *d, float p[]) 
{
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { // 初期設定は最初に１回
        timer = 0;
    }
    if (0 == (timer % cycletotal)) {
	int ix = 319, iy = 239;   // 画像の注目座標、最初は適当な初期値なので意味なし
	float x, y, z;            // 計算後のローカル座標を記憶する変数
	float ha = p[AP_NECK_HA]; // 首の水平回転角度(radian)、座標の補正に必要
	float va = p[AP_NECK_VA]; // 首の垂直回転角度(radian)、座標の補正に必要

※  カメラで撮影した原画像に対して、必要な画像処理を施し、注目点を決める
　重要なのは注目点を決める処理で、これは自分で考えて作るしかない。
　OpenCV では基本的な操作はできるが、最後はその結果から自分で注目点を決める。

//	if (0 < SuzukiVSobel(c, d, &ix, &iy)) {            // Sobel フィルタ、輪郭を求める
//	if (0 < SuzukiVLaplacian(c, d, &ix, &iy)) {        // Laplacian フィルタ、輪郭を求める
	if (0 < SuzukiVFindColorObj(c, d, &ix, &iy, RED)) { // 指定色の物体を見つける

※  この段階では ix, iy に値が入っている必要がある。この例では ix, iy は１点だけだが
　2点以上を考えても構わない。次に ix, iy から３次元のローカル座標を求める、
　x, y, z はアームを動かす目標座標などで使える。

	    d -> LocalXyz(ix, iy, &x, &y, &z, ha, va);
	    p[AP_LOCALX] = x; // p[] に入れて計算結果を呼び出し元に返す
	    p[AP_LOCALY] = y;
	    p[AP_LOCALZ] = z;
	    p[AP_OBJFIND] = YES; // 物体発見のフラグ、見つけたか否かで次の動作を変えられる
	}
	else {
	    p[AP_OBJFIND] = NO;  // 物体を見つけなかった
	}

    timer++;
    if (timer < cycletotal) { // 動作継続中の処理
	p[AP_STATE] = ACT_MOVING;
	return ACT_MOVING;
    }
    else    {   // 動作終了、タイマをクリアして ACT_END を返り値に
	timer = 0;
	p[AP_STATE] = ACT_END;
	return ACT_END;
    }
}

int ActSuzukiGrab(C1 m[], float p[]) // 物体を掴む、持ち上げる、落とす
{
    float x, y, z; // 掴む対象となる物体の座標
    
    x = p[AP_LOCALX]; // 掴む対象となる物体の座標、p[]経由で受け取る
    y = p[AP_LOCALY];
    z = p[AP_LOCALZ];
    
    if (ACT_START == (int)p[AP_STATE] || 0 == timer) { 
	ikparam[1] = IKHAND;
	ikparam[2] = IKELBOWUP;
	ikparam[3] = (float)MODEPROFILE;
        timer = 0;
    }
// １番目の動き、物体の上にハンドを移動させる
    if (0 == (timer % cycletotal)) {
        float zz = z + 0.25, target1[XYZ] = {x, y, zz}; // 物体の 250mm 上
        ikparam[0] = time1;
// 次の5行はその後のコメントアウトされている処理と同等、三項演算子を使ってまとめた 
        IKMotions(m, 0.0<y?LF:RF, target1, ikparam); // アーム
        Hand(m, 0.0<y?LF:RF, HANDOPEN, 1.57); // ハンドを開く, PI/2 radian
        float targetrest[XYZ] = {0.6, 0.2, 0.3};
        targetrest[1] *= 0.0<y?-1:1;
        IKMotions(m, 0.0<y?RF:LF, targetrest, ikparam); // 使わないアーム、おやすみ    
    }
// ２番目の動き、腕を下ろす
    if (cycle1 == (timer % cycletotal)) {
        ikparam[0] = time2; // 動きによって時間が異なる場合
	float xx = x + 0.05, zz = z - 0.075, target1[XYZ] = {xx, y, zz};
// 物体が左右どちらにあるかで動かす腕を変える
	IKMotions(m, 0.0<y?LF:RF, target1, ikparam); // 物体の 50mm 奥、75mm 下
    }
// ３番目の動き、ハンドを閉じる
   略
// ４番目の動き、持ち上げる
   略
// ５番目の動き、ハンドを開いて落とす
   略
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

int SuzukiVFindColorObj(RGBCam *c, DepthCam *d, int *x, int *y, int color)
// 指定した色 color の物体(画素の集まり)を見つけて、重心座標を *x, *y で返す。
// 返り値は見つけた画素の個数、1以上なら見つかった、0 なら見つからなかった。
// 画像中の物体の個数は１個を前提としている。
// ２個以上の場合は正しく動作しないので、別の機能を実装する必要がある。
{
	cv_bridge::CvImagePtr p = c -> cv_ptr;

※  画像全体で何かを調べるときには、以下のような全画素をスキャンする二重ループ
　が必要になるのでこの書き方を理解する。

	for (int y = 0; y < p -> image.rows; y++) {
	    for (int x = 0; x < p -> image.cols; x++) {
		Uchar b = p -> image.at<cv::Vec3b>(y, x)[0];
		Uchar g = p -> image.at<cv::Vec3b>(y, x)[1];
		Uchar r = p -> image.at<cv::Vec3b>(y, x)[2];
		if (YES == c -> TestColor(r, g, b, color)) { // 色の条件に合ったら
		    sumx += x; sumy += y; sum++;
		    img.at<Uchar>(y, x) = 255;
		}
		else {
		    img.at<Uchar>(y, x) = 0;
		}
	    }
	}

　　　　途中略

	return sum; // 見つかった画素の個数を返り値としている
}

※  OpenCVの機能を利用するときにはSuzukiVSobel() や SuzukiVLaplacian()
　のような書き方になる。自分で作ると難しい処理でも、OpenCVの関数を利用
　すると１，２行で実行できるので楽である。

int SuzukiVSobel(RGBCam *c, DepthCam *d, int *x, int *y)
{
　　略
}

int SuzukiVLaplacian(RGBCam *c, DepthCam *d, int *x, int *y)
// 画像処理の例、Laplacian フィルタで輪郭を求める。
{
    cv_bridge::CvImagePtr p = c -> cv_ptr; // 原画像の構造体へのポインタ

    cv::Mat input = p -> image; // カメラで撮影した原画像
    cv::Mat output, out2; // 配列の大きさは指定しなくてもよい、自動で決まる

    cv::Laplacian(input, output, CV_32F, 1, 1, 3); // Laplacian フィルタ、輪郭

    cv::convertScaleAbs(output, out2, 1, 0);

    cv::imshow("Lapracian", out2); // 処理結果を画像として表示する
    cv::waitKey(5); // 5ms 待つ、消してはいけない

    cv::threshold(out2, out2, 32, 255, cv::THRESH_BINARY);
    cv::imshow("LapracianBin", out2); // 処理結果を画像として表示する
    cv::waitKey(5); // 5ms 待つ、消してはいけない

    *x = 0; // 必要なら何らかの座標を呼び出し元に返す
    *y = 0;

    return 0; // 何らかの処理結果の値を返り値とする
}
