// imageclass.h

#ifndef	HEADER_IMAGECLASS
#define	HEADER_IMAGECLASS

const int COLOR = 0;
const int RGB = COLOR;

class RGBCam
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_; // 画像データのサブスクライバ
    image_transport::Publisher  image_pub_; // 画像データのパブリッシャ
    const char *sname, *pname; // 画像データのサブスクライバ, パブリッシャの名前
    const char *wname        ; // 画像を表示する Window の名前
    int markn = 0;
    int mark[MAXOBJS*2]; // x, y の2次元なので、2倍の要素数を用意する
    int id; // カメラの ID
    Uchar winopen; // imshow() で window を開いたかを示すフラグ
    
public:
    cv_bridge::CvImagePtr cv_ptr;        // 最初に画像データを記録しておく場所
    cv::Mat cvmary[CVMALL];              // 画像処理途中のデータを記録しておく場所
    std::vector<cv::Vec4i> lines;        // Hough 変換で検出した直線の座標を記憶
// 以下の3個は画像処理で検出した領域のデータ
    int	fareanum;                        // 検出領域の個数
    int fareacog[MAXOBJS][XY];           // 検出領域の重心、Center of Gravity
    int fareacorners[MAXOBJS][LURL][XY]; // 検出領域の外接長方形の左上と右下角の座標
    
    
    RGBCam(int camid, const char sn[], const char pn[], const char wn[]) : it_(nh_) // コンストラクタ
    {
        id = camid;
        sname = sn; // 引数による名前の設定
        pname = pn;
	wname = wn;
// サブスクライバ、パブリッシャの用意
// サブスクライバは、カメラから画像のメッセージを受け取る度に imageCallback() で処理する
	image_sub_ = it_.subscribe(sname, 1, &RGBCam::imageCallback, this);
// パブリッシャは ROS で画像を扱えるように変換後の画像をパブリッシュする
	image_pub_ = it_.advertise(pname, 1);
    }

    ~RGBCam() // デストラクタ
    { 
// 表示に使ったWindowを破棄する
        if (YES == winopen) cv::destroyWindow(wname);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
//	printf("c");
        try {
// ROS から OpenCV の形式に toCvCopy() で変換、cv_ptr->image が cv::Mat フォーマット
	    if (sensor_msgs::image_encodings::isColor(msg -> encoding)) {
	        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    }
	}
	catch (cv_bridge::Exception& e) { // もしエラーがあれば
	    ROS_ERROR("cv_bridge(color) exception: %s", e.what());
	    return;
	}

        static int count = 0;

// id を変えると別のカメラの画像を表示できる
	if (0 == count++ % 5) { // 深度カメラが３台になったのでIDチェックは一旦外した
//	if (IDDEPTHCAMCOLOR == id && 0 == count++ % 5) {
// マルチスレッドで深度画像を同時に表示させるとエラーで動かないので注意する。
//	    return; // その場合はここで return すればよい

/*	    for (int y = 50; y < 60; y++) { // データ参照の例、画像に青色の四角形を描く
		for (int x = 50; x < 60; x++) {
		    cv_ptr -> image.at<cv::Vec3b>(y, x)[0] = 255; // Blue
		    cv_ptr -> image.at<cv::Vec3b>(y, x)[1] = 0;   // Green
		    cv_ptr -> image.at<cv::Vec3b>(y, x)[2] = 0;   // Red
		}
	    }
*/
// 画像処理の例として、赤色の領域の重心座標を求める、後に FindObj() を作成した
/*
	    int sumx = 0, sumy = 0, sum  = 0;
	    for (int y = 0; y < cv_ptr -> image.rows; y++) {
		for (int x = 0; x < cv_ptr -> image.cols; x++) {
		    Uchar b = cv_ptr -> image.at<cv::Vec3b>(y, x)[0];
		    Uchar g = cv_ptr -> image.at<cv::Vec3b>(y, x)[1];
		    Uchar r = cv_ptr -> image.at<cv::Vec3b>(y, x)[2];
		    if ((b*4 < r) && (g*4 < r)) {sumx += x; sumy += y; sum++;}
		}
	    }
	    if (0 != sum) printf("(%d,%d)(%d,%d,%d) ", sumx/sum, sumy/sum,
				 cv_ptr -> image.at<cv::Vec3b>(sumy/sum, sumx/sum)[2],
				 cv_ptr -> image.at<cv::Vec3b>(sumy/sum, sumx/sum)[1],
				 cv_ptr -> image.at<cv::Vec3b>(sumy/sum, sumx/sum)[0]);
*/
// kinetic では途中で表示が固まるので、imshow() はコメントアウトするとよい
	    if (0 < markn) {
//		if (MAXOBJS < markn) markn = MAXOBJS; // この対応は事前に済なので不要なはず
		RGBmark(markn, mark);
	    }
	    cv::imshow(wname, cv_ptr -> image); winopen = YES;
	    cv::waitKey(5); // 5ms 待つ
// OpenCV による画像の加工の例、画像の大きさを半分にして表示する
/*          cv::Mat cv_tmpimg; // 変換後の画像データを入れておく仮の場所
	    cv::resize(cv_ptr -> image, cv_tmpimg, cv::Size(), 0.5, 0.5);
	    cv::imshow("Half", cv_tmpimg);
	    cv::waitKey(5); // 5ms 待つ 
*/
	}
// 画像を publish, OpenCVからROS形式にtoImageMsg()で変換すると rviz など ROS のツールで扱える
	image_pub_.publish(cv_ptr -> toImageMsg());

// カメラの動作確認用に途中で 1 回画像をファイルに書き出してみる
/*
	if (IDHEADCAM == id && 100 == count) { // 画像を100枚読み込んだら
	    imwrite("/tmp/cam1.ppm", cv_ptr -> image);
	}
	if (IDDEPTHCAMCOLOR == id && 100 == count) { // 画像を100枚読み込んだら
	    imwrite("/tmp/cam3.ppm", cv_ptr -> image);
	}
*/
    }
/*
// imFindColorObj() が image.cpp にあるので注意 
    int FindColorObj(int *x, int *y, int color)
// 指定した色 color の物体(画素の集まり)を見つけて、重心座標を *x, *y で返す。
// 返り値は見つけた画素の個数、1以上なら見つかった、0 なら見つからなかった。
// 画像中の物体の個数は１個を前提としている。
// ２個以上の場合は正しく動作しないので、別の機能を実装する必要がある。
    {
	int sumx = 0, sumy = 0, sum  = 0; // 見つけた画素の座標と個数
	cv::Mat img(cv_ptr -> image.rows, cv_ptr -> image.cols, CV_8UC1); // 検出結果
	    
	for (int y = 0; y < cv_ptr -> image.rows; y++) {
	    for (int x = 0; x < cv_ptr -> image.cols; x++) {
		Uchar b = cv_ptr -> image.at<cv::Vec3b>(y, x)[0];
		Uchar g = cv_ptr -> image.at<cv::Vec3b>(y, x)[1];
		Uchar r = cv_ptr -> image.at<cv::Vec3b>(y, x)[2];
		if (YES == TestColor(r, g, b, color)) { // 色の条件に合ったら
//		if ((b*4 < r) && (g*4 < r)) { // 以前の処理、赤色か
		    sumx += x; sumy += y; sum++;
		    img.at<Uchar>(y, x) = 255;
		}
		else {
		    img.at<Uchar>(y, x) = 0;
		}
	    }
	}
	if (0 < sum) { // 指定した色の画素が見つかったとき
	    *x = sumx/sum;
	    *y = sumy/sum;
//	    fprintf(stderr, "(%d,%d) ", *x, *y);
//	    imwrite("/tmp/detect.pbm", img); // 検出結果の画像、見る必要なければ実行しない
	}
	else { // 見つからなかったとき
	    *x = 0; 
	    *y = 0;
	}

	cv::imshow("ColorObj", img); // 検出結果を画像として表示してみる
	cv::waitKey(5); // 5ms 待つ、消してはいけない

	return sum; // 見つかった画素の個数を返り値とする
    }
*/	    
// TestColor2() が image.cpp にあるので注意
    int TestColor(Uchar r, Uchar g, Uchar b, int color) // 画素の色を調べる
    {
	float rate = 4.0; // 仮に 4 としてあるが、変更してもよい
	
	if (RED == color) {
	    if ((g*rate < r) && (b*rate < r)) return YES; // 他色より rate 倍以上 r が大なら合格
	    return NO;
	}
	if (GREEN == color) {
	    if ((r*rate < g) && (b*rate < g)) return YES; // 他色より rate 倍以上 g が大なら合格
	    return NO;
	}
	if (BLUE == color) {
	    if ((r*rate < b) && (g*rate < b)) return YES; // 他色より rate 倍以上 b が大なら合格
	    return NO;
	}
/*
	if (YELLOW == color) {
            if ( ... ) return YES; // 独自の色判定を作ればよい
	}
	if (CYAN == color) {
            if ( ... ) return YES; // 独自の色判定を作ればよい
	}
	if (PURPLE == color) {
            if ( ... ) return YES; // 独自の色判定を作ればよい
	}
*/
	return NO;
    }

    void SetMark(int n, int xy[]) // 画像中に描画する + マークの座標の登録
    {
	markn = n;
	for (int i = 0; i < n*2; i++) mark[i] = xy[i];
    }

    void ResetMark(void) // 登録された座標を削除する 
    {
	markn =	0;
    }
    
    void RGBmark(int n, int xy[]) // 画像中に + マークを描画する
    {
	for (int i = 0; i < n*2; i+=2) {
	    reversecolor(xy[i],   xy[i+1]  );
	    reversecolor(xy[i]-1, xy[i+1]  );
	    reversecolor(xy[i],   xy[i+1]-1);
	    reversecolor(xy[i]+1, xy[i+1]  );
	    reversecolor(xy[i],   xy[i+1]+1);
	    reversecolor(xy[i]-2, xy[i+1]  );
	    reversecolor(xy[i],   xy[i+1]-2);
	    reversecolor(xy[i]+2, xy[i+1]  );
	    reversecolor(xy[i],   xy[i+1]+2);
// 以下のコメントを外すとマークがよりはっきり表示される
//	    reversecolor(xy[i]-1, xy[i+1]-1);
//	    reversecolor(xy[i]-1, xy[i+1]+1);
//	    reversecolor(xy[i]+1, xy[i+1]-1);
//	    reversecolor(xy[i]+1, xy[i+1]+1);
	}
    }
    
    void RGBsave(char fname[]) // RGBカメラが取り込んだ画像をファイルに書き出す
    {
	imwrite(fname, cv_ptr -> image);
    }

    void RGBsavemark(char fname[], int n, int xy[]) // マーク付き RGBsave()
    {
	RGBmark(n, xy);
	RGBsave(fname);
    }

    void reversecolor(int x, int y) // 画像中の画素の値を反転させる
    {
	if (x < 0) x = 0;
	if (639 < x) x = 639; // 画像の横方向画素数、カメラの設定を変えたら要確認
	if (y < 0) y = 0;
	if (479 < y) y = 479; // 画像の縦方向画素数

	int tmp;
	tmp = cv_ptr -> image.at<cv::Vec3b>(y, x)[0];
	if (64 <= tmp && tmp < 128) // 反転した色が薄くなる時の例外処理
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[0] = 255;
	else if (128 <= tmp && tmp < 192) // 反転した色が薄くなる時の例外処理
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[0] = 0;
	else // 通常はここ
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[0] = 255 - tmp;
	
	tmp = cv_ptr -> image.at<cv::Vec3b>(y, x)[1];
	if (64 <= tmp && tmp < 128)
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[1] = 255;
	else if (128 <= tmp && tmp < 192)
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[1] = 0;
	else
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[1] = 255 - tmp;
	
	tmp = cv_ptr -> image.at<cv::Vec3b>(y, x)[2];
	if (64 <= tmp && tmp < 128)
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[2] = 255;
	else if (128 <= tmp && tmp < 192)
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[2] = 0;
	else
	    cv_ptr -> image.at<cv::Vec3b>(y, x)[2] = 255 - tmp;
    }
};

#endif // HEADER_IMAGECLASS
