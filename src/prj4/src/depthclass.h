// depthclass.h

#ifndef	HEADER_DEPTHCLASS
#define	HEADER_DEPTHCLASS

const int DEPTH = 1;

class DepthCam
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_; // 画像データのサブスクライバ
    image_transport::Publisher  image_pub_; // 画像データのパブリッシャ
    const char *sname, *pname; // 画像データのサブスクライバ, パブリッシャの名前
    const char *wname;
    Uchar winopen; // imshow() で window を開いたかを示すフラグ
    float necklen; // カメラが付いている首の長さ
    float cgx, cgy, cgz; // 本体重心からのカメラの相対座標
public:
    cv_bridge::CvImagePtr cv_ptr; // 最初に画像データを記録しておく場所
    cv::Mat cvmary[CVMALL];
    std::vector<cv::Vec4i> lines;        // Hough 変換で検出した直線の座標を記憶
    int   id; // カメラの ID
    int   fobjnum;                // 検出物体の個数
    float fobjxyz[MAXOBJS][XYZ];  // 検出物体のローカル3次元座標 XY →  XYZ に修正済
    float ha, va; // カメラの回転角度

    int fareanum;                        // 検出領域の個数 from imageclass.h
    int fareacog[MAXOBJS][XY];           // 検出領域の重心、Center of Gravity
    int fareacorners[MAXOBJS][LURL][XY]; // 検出領域の外接長方形の左上と右下角の座標             
    
    DepthCam(int camid, const char sn[], const char pn[], const char wn[]) : it_(nh_) // コンストラクタ
    {
        id = camid;
        sname = sn;
        pname = pn;
	wname = wn;
	fobjnum = 0;
	fareanum = 0;
	if (IDDEPTHCAMDEPTH == id) { // 本体前面首に付いているカメラの角度や位置
	    ha = 0.0; va = 0.0;
	    necklen = 0.075;
	    cgx = 0.15; cgy = 0.0; cgz = 0.25;
	}
	else if (IDDEPTHCAMDEPTHSL == id) { // 本体左側面に付いているカメラの角度や位置
	    ha = 1.570796; va = -0.5236; // 90度横、下に30度回転、buildurdf.urdf も確認
	    necklen = 0.0;
	    cgx = -0.065; cgy = 0.15; cgz = -0.05;
	}
	else if (IDDEPTHCAMDEPTHSR == id) { // 本体右側面に付いているカメラの角度や位置
	    ha = -1.570796; va = -0.5236; // 映る範囲はurdfで決まる。でもこの値は座標計算に要
	    necklen = 0.0;
	    cgx = -0.065; cgy = -0.15; cgz = -0.05;
	}
// サブスクライバ、パブリッシャの用意
// サブスクライバは、カメラから画像のメッセージを受け取る度に depthimageCallback() で処理する
	image_sub_ = it_.subscribe(sname, 1, &DepthCam::depthimageCallback, this);
// パブリッシャは ROS で画像を扱えるように変換後の画像をパブリッシュする
	image_pub_ = it_.advertise(pname, 1);
    }

    ~DepthCam() // デストラクタ
    { 
// 表示に使ったWindowを破棄する
        if (YES == winopen) cv::destroyWindow(wname);
    }

    void depthimageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
//	printf("d");
        try {
// ROS から OpenCV の形式に toCvCopy() で変換、cv_ptr->image が cv::Mat フォーマット
	    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e) { // もしエラーがあれば
	    ROS_ERROR("cv_bridge(depth) exception: %s", e.what());
	    return;
	}
/*
	for (int y = 50; y < 60; y++) { // データ参照のテスト、画像の一部に四角形を描く
	    for (int x = 50; x < 60; x++) {
		cv_ptr -> image.at<float>(y, x) = 5.0;
	    }
	}
*/
//	printf("%d %d\n", cv_ptr -> image.rows, cv_ptr -> image.cols); // 480 640
//	cv::Mat depth(cv_ptr -> image.rows, cv_ptr -> image.cols, CV_32FC1);
	cv::Mat   img(cv_ptr -> image.rows, cv_ptr -> image.cols, CV_8UC1);

// 画像データの処理は以下の書き方なら正しく動作する	
	for (int y = 0; y < cv_ptr -> image.rows; y++) {
	    for (int x = 0; x < cv_ptr -> image.cols; x++) {
		float tmp;
// isnan() ではなくて std::isnan() とするのは Ubuntu16.04 でコンパイルを通すため
// Ubuntu18.04 以降は isnan() でも通る。同様の記述は140行付近にもある。
		if (0 != std::isnan(cv_ptr -> image.at<float>(y, x))) { // too far
		    tmp = 99;
		}
		else {
		    tmp = cv_ptr -> image.at<float>(y, x);
		}
//		depth.at<float>(y, x) = cv_ptr -> image.at<float>(y, x);
	        tmp *= 25;
		if (tmp < 0) tmp = 0; else if (255 < tmp) tmp = 255;
		img.at<Uchar>(y, x) = tmp;
	    }
	}
// 画像データの処理は以下の書き方では結果が乱れたので使わない
/*
	for (int i = 0; i < cv_ptr -> image.rows; i++) {
	    float *dptr = depth.ptr<float>(i);
	    Uchar *iptr =   img.ptr<Uchar>(i);
	    for (int j = 0; j < cv_ptr -> image.cols; j++) {
	        int tmp = dptr[j] * 25; 
		if (tmp < 0) tmp = 0; else if (255 < tmp) tmp = 255;
	        iptr[j] = tmp;
	    }
	}
*/
        static int count = 0;
	if (0 == count++ % 5) { // 深度カメラが３台になったのでIDチェックは一旦外した
//	if (IDDEPTHCAMDEPTH == id && 0 == count++ % 5) { 
// 以下の３行はどれでも正しく表示できるようになった
	    cv::imshow(wname, img); winopen = YES;
//	    cv::imshow(wname, depth); winopen = YES;
//	    cv::imshow(wname, cv_ptr -> image); winopen = YES;
	    cv::waitKey(5); // 5ms 待つ
// マルチスレッドでカラー画像と同時に表示すると以下のエラーで動かない。
// $ rosrun prj4 prg4 
// start ActInitialPose()
// [xcb] Unknown sequence number while processing reply
// [xcb] Most likely this is a multi-threaded client and XInitThreads has not been called
// [xcb] Aborting, sorry about that.
// prg4: ../../src/xcb_io.c:641: _XReply: アサーション `!xcb_xlib_threads_sequence_lost' に失敗。
// 中止 (コアダンプ)
	}
// 画像を publish, OpenCVからROS形式にtoImageMsg()で変換すると rviz など ROS のツールで扱える
	image_pub_.publish(cv_ptr -> toImageMsg());

// カメラの動作確認用に途中で画像をファイルに書き出してみる
/*	static int f1 = 0;
	if (0 == (count % 100) && f1++ < 2) {
	    char fname[32];
	    sprintf(fname, "/tmp/dcam%03d.pgm", f1);
//          imwrite(fname, cv_ptr -> image);
//	    imwrite(fname, img); // 今は確認不要なので実行しない
        } */
    }

    void LocalXyz(const int ix, const int iy, float *x, float *y, float *z,
		  const float oldha, const float oldva) const
// カメラ画像中の座標(ix, iy)の画素の３次元ローカル座標を求めて x, y, z に代入する。
// ha(horizontal angle), va(vertical angle) は首の水平、垂直回転角度を表す。
// ha, va から首が回転しても正しい座標になるように補正する。
// 変換結果は水平、垂直回転の角度が増すほど誤差が増えるので誤差の許容範囲を考える。
// 水平回転をしていなければ垂直0度から下向き80度くらいは使える
// 垂直回転をしなければ水平回転は問題ない
// 水平、垂直回転を両方使う場合は、水平±40度、垂直0度から下20度くらいまで
    {
// ha, va, necklen, cgx, cgy, cgz はカメラ毎にメンバ変数として指定するように変更した
	xyd2xyz(ix, iy, x, y, z); // カメラのCCD撮像面からの3次元座標を求める
	adjustrotate(ha, va, x, y, z); // カメラの回転を考慮して座標を補正する
	adjustneck(ha, va, necklen, x, y, z); // 長さがある首の回転による座標のズレを補正する
	adjustbodycg(cgx, cgy, cgz, x, y, z); // 本体重心からの座標に変換する
// 赤い箱を使うキャリブレーションで *x が 1.15 ではなく 1.13 になるのは、箱の大きさが
// あるため手前の表面が 10mm 程カメラに近くなるためで間違いではない
	fprintf(stderr, "  LocalXyz() cam:%d ix:%d iy:%d ha:%.2f va:%.2f *x:%.2f *y:%.2f *z:%.2f\n",
		id, ix, iy, ha, va, *x, *y, *z);
    }
    
    void xyd2xyz(const int ix, const int iy, float *x, float *y, float *z) const
// カメラ画像中の座標(ix, iy)の画素の３次元ローカル座標を求めて x, y, z に代入する。
    {
	const float cx = 319.5; // (319+320)/2, horizontal center of CCD, depends on resolution
	const float cy = 239.5; // (239+240)/2, vertical   center of CCD, depends on resolution

	if (0 != std::isnan(cv_ptr -> image.at<float>(iy, ix))) { // 遠すぎて計算できない場合
	    *x = 99; // 一律にこの値にしてあるのでこれ以降計算を続けても無意味
	    *y = 99;
	    *z = 99;
	}
	else {
	    *x = cv_ptr -> image.at<float>(iy, ix);
	    *y = (ix - cx) * *x / -382; // -382 の由来は記録が残っていない
	    *z = (iy - cy) * *x / -382;
	}
    }

    void adjustrotate(const float ha, const float va, float *x, float *y, float *z) const
// ha(horizontal angle), va(vertical angle) は首の水平、垂直回転角度を表す。
// ha が正値だと左回転、負値だと右回転
// va が正値だと下を向きCCDは前に動く、負値だと上を向きCCDは後ろに動く
    {
	float tx, ty, tz; // 計算途中で *x, *y, *z は変更不可なのでこれらが必要
	float mha = ha *  1; // 正負と回転方向の調整、今は必要なし
	float mva = va *  1; // 正負と回転方向の調整、今は必要なし

//	fprintf(stderr, "H%.1f,V%.1f, ", ha, va);
//	fprintf(stderr, " (%.3f %.3f %.3f)", *x, *y, *z);

// z 軸周り水平回転の回転行列
        tx = (cos(mha) * *x) + (-sin(mha) * *y);
        ty = (sin(mha) * *x) + ( cos(mha) * *y);
	tz = 1 * *z;
// y 軸周り垂直回転の回転行列
	*x = ( cos(mva) * tx) + ( sin(mva) * tz);
	*y = 1 * ty;
        *z = (-sin(mva) * tx) + ( cos(mva) * tz);
//	fprintf(stderr, " (%.3f %.3f %.3f)", *x, *y, *z);
    }

    void adjustneck(const float ha, const float va, const float necklen,
		    float *x, float *y, float *z) const
// ha(horizontal angle), va(vertical angle) は首の水平、垂直回転角度を表す。
// ha が正値だと左回転、負値だと右回転
// va が正値だと下を向きCCDは前に動く、負値だと上を向きCCDは後ろに動く
    {
// 以下は首の長さがあることによる撮像面の移動距離の補正
	float ccdmoveH =  sin(va)      * necklen; // 撮像面の前後移動、垂直回転軸からCCDまで75mm
	float ccdmoveV = (cos(va)-1.0) * necklen; // 撮像面の下移動、垂直回転軸からCCDまで75mm
	*x += ccdmoveH * cos(ha);
	*y += ccdmoveH * sin(ha);
	*z += ccdmoveV;
//	fprintf(stderr, " (%.3f %.3f %.3f)", *x, *y, *z);
    }

    void adjustbodycg(const float cgx, const float cgy, const float cgz,
		      float *x, float *y, float *z) const
    {
// 以下は本体重心から撮像面までの距離の補正
	*x += cgx; // CCD撮像面からボディ中心を原点とした座標に変換する
	*y += cgy; // CCD撮像面からボディ中心を原点とした座標に変換する
	*z += cgz; // CCD撮像面からボディ中心を原点とした座標に変換する
//	fprintf(stderr, " (%.3f %.3f %.3f)\n", *x, *y, *z);
    }

    void print3d(void) const
// 画像中の全画素をスキャンして３次元座標が求められる場合は SDF ファイルに書き出す。
    {
	int c = 0;
	float rx, ry, rz;
	FILE *fp;

	if ((FILE *)NULL == (fp = fopen("./src/prj4/sdf/3dprint.sdf", "w"))) {
	    fprintf(stderr, "Error: [%s] can not open\n", "3dprint.sdf");
	    exit(1);
	}
	fprintf(fp, "<?xml version=\"1.0\" ?>\n<sdf version=\"1.5\">\n");
	fprintf(fp, "<model name=\"3dprint\">\n<static>true</static>\n");
	fprintf(fp, "<link name=\"link\">\n\n");
	for (int y = 0; y < cv_ptr -> image.rows; y+=4) {
	    for (int x = 0; x < cv_ptr -> image.cols; x+=4) {
		xyd2xyz(x, y, &rx, &ry, &rz);
		if (10.0 < rx || rz < -0.95) continue;
		fprintf(fp, "<collision name=\"p%dc\">", c);
		fprintf(fp, "<pose>%.3f %.3f %.3f 0 0 0</pose>\n", rx, ry, rz);
		fprintf(fp, "<geometry><box><size>0.01 0.01 0.01</size></box></geometry>\n");
		fprintf(fp, "</collision>\n");
		fprintf(fp, "<visual name=\"p%dv\">", c++);
		fprintf(fp, "<pose>%.3f %.3f %.3f 0 0 0</pose>\n", rx, ry, rz);
		fprintf(fp, "<geometry><box><size>0.01 0.01 0.01</size></box></geometry>\n");
		fprintf(fp, "<material><script><uri>file://media/materials/scripts/gazebo.material</uri>\n");
		fprintf(fp, "<name>Gazebo/%s</name></script></material>\n", colorname(rx));
		fprintf(fp, "</visual>\n\n");
	    }
	}
	fprintf(fp, "</link>\n</model>\n</sdf>\n");
	fclose(fp);
    }

    char* colorname(float x) const
// SDF ファイルを書き出す際に x 座標(奥行き)に応じて色を変化させ、その名前を返す
    {
	static char name[][10] = {"Yellow", "Orange", "Red", "Purple",
				  "Blue", "Indigo", "SkyBlue", "Green",
				  "Grey", "DarkGrey"};

	if      (x < 0.6 ) return(name[0]); // 距離が０．６ｍより近ければ
	else if (x < 0.8 ) return(name[1]);
	else if (x < 1.0 ) return(name[2]);
	else if (x < 1.2 ) return(name[3]);
	else if (x < 1.4 ) return(name[4]);
	else if (x < 1.6 ) return(name[5]);
	else if (x < 1.8 ) return(name[6]);
	else if (x < 2.0 ) return(name[7]);
	else if (x < 3.0 ) return(name[8]);
	else               return(name[9]);
    }

    void Dsave(char fname[])
// 画像を指定したファイル名で記録する
    {
	imwrite(fname, cv_ptr -> image);
    }
};

#endif // HEADER_DEPTHCLASS



// URDF create
// 画像中の全画素で３次元座標が求められる場合は URDF ファイルに書き出す。
/*
	fprintf(fp, "<?xml version=\"1.0\" ?>\n");
	fprintf(fp, "<robot name=\"3dprint\">\n");
	for (int y = 0; y < cv_ptr -> image.rows; y++) {
	    for (int x = 0; x < cv_ptr -> image.cols; x++) {
		xyd2xyz(x, y, &rx, &ry, &rz);
		if (10.0 < rx || rz < 0.3) continue;
		fprintf(fp, "<link name=\"p%d\">\n<visual>\n", c);
		fprintf(fp, "<origin xyz=\"%.2f %.2f %.2f\" rpy=\"0 0 0\" />\n", rx, ry, rz);
		fprintf(fp, "<geometry> <box size=\"0.001 0.001 0.001\"/> </geometry>\n");
		fprintf(fp, "</visual></link>\n");
		fprintf(fp, "<gazebo reference=\"p%d\"> <material>Gazebo/Red</material> </gazebo>\n\n", c++);
	    }
	}
	fprintf(fp, "</robot>\n");
*/

/*
    void adjustrotateOld(const float ha, const float va, float *x, float *y, float *z) const
// ha(horizontal angle), va(vertical angle) は首の水平、垂直回転角度を表す。
// ha, va から首が回転しても正しい座標になるように補正する。
// ha が正値だと左回転、負値だと右回転
// va が正値だと下を向きCCDは前に動く、負値だと上を向きCCDは後ろに動く
    {
	float tx, ty, tz; // 計算途中で *x, *y, *z は変更不可なのでこれらが必要
	float ox = *x, oy = *y, oz = *z; // 補正前の値
	float mha = ha *  1; // 正負と回転方向の調整
	float mva = va * -1; // 正負と回転方向の調整
// 3次元座標の結果がおかしい場合は、ha * 1,-1,  va * 1,-1 の結果を比べてみる。

// 以下は z 軸周り水平回転の補正
        *x = (cos(mha) * ox) + (-sin(mha) * oy);
        *y = (sin(mha) * ox) + ( cos(mha) * oy);
	tx = *x;
	ty = *y;
// 以下は y 軸周り垂直回転の補正
//      *z = (sin(mva) * ox) + ( cos(mva) * oz); // 座標の誤差が大きいので使わない
        *z = (sin(mva) * tx) + ( cos(mva) * oz); // こちらは誤差が小さい
	tz = *z;
	*x = (cos(mva) * tx) + (-sin(mva) * tz);
	
	fprintf(stderr, "H%.1f,V%.1f, ", ha, va);
	fprintf(stderr, " (%.3f %.3f %.3f) ", *x, *y, *z);

// 20230817 までの処理、ここから
// 以下は水平回転の補正
	tx = (cos(mha) * ox) + (-sin(mha) * oy); *x = tx;
	ty = (sin(mha) * ox) + ( cos(mha) * oy); *y = ty;
// 以下は垂直回転の補正
	tz = (sin(mva) * ox) + ( cos(mva) * oz); // これは当時は良かった、tx は更に良
	*z = tz;
//	tx = (cos(mva) * *x) + (-sin(mva) * oz); // *x の更新、次行とどちらがよいか
	tx = (cos(mva) * *x) + (-sin(mva) * *z); *x = tx; // この2度目の更新は効果あり
// 20230817 までの処理、ここまで

	float ccdmoveH =  sin(va)      * 0.075; // 撮像面の前後移動、垂直回転軸からCCDまで75mm
	float ccdmoveV = (cos(va)-1.0) * 0.075; // 撮像面の下移動、垂直回転軸からCCDまで75mm
	*x += ccdmoveH * cos(ha);
	*y += ccdmoveH * sin(ha);
	*z += ccdmoveV;

//	*x += 0.15; // CCD撮像面からボディ中心を原点とした座標に変換する
//	*z += 0.25; // CCD撮像面からボディ中心を原点とした座標に変換する
	
// 以下はヒューリスティックス、加算と乗算で可能な範囲で補正する
	fprintf(stderr, "\n");
	for (float p = 0.0; p < 0.1; p += 0.01) { // 10
	    for (float m = 1.0; m < 1.25; m += 0.05) { // 5
		tx = *x + p;
		tx *= m;
		fprintf(stderr, "p%.2f/m%.2f/x%.2f,", p, m, tx+0.15);
	    }
	    fprintf(stderr, "\n");
	}

// 0080 p 0.14 m 1.7
// 0060 p 0.07 m 1.4
// 0040 p 0.00 m 1.2
// 0020 p 0.00 m 1.05
// 0000 p 0.00 m 1.0

// 2080 p 0.15 m 1.5
// 2060 p 0.05 m 1.4
// 2040 p 0.00 m 1.2
// 2020 p 0.00 m 1.05
// 2000 p 0.00 m 1.0

// 4080 p 0.15 m 1.5
// 4060 p 0.05 m 1.4
// 4040 p 0.00 m 1.2
// 4020 p 0.00 m 1.05
// 4000 p 0.00 m 1.00

// 6080 p 0.15 m 1.5
// 6060 p 0.05 m 1.4
// 6040 p 0.00 m 1.2
// 6020 p 0.00 m 1.05
// 6000 p 0.00 m 1.00
	
//	fprintf(stderr, " HV(%.1f,%.1f) Before(%.3f %.3f %.3f) ", ix-cx, iy-cy, *x, *y, *z);
//	fprintf(stderr, "After(%.3f %.3f %.3f)\n", *x, *y, *z);
    }
*/
