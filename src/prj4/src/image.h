// image.h  image.cpp 用の定数やプロトタイプ宣言

#ifndef HEADER_IMAGE
#define HEADER_IMAGE

#include "imageclass.h"
#include "depthclass.h"

const int ROWS = 480; // カメラ画像の縦の画素数、cv::Mat の値が使えないとき用
const int COLS = 640; // カメラ画像の横の画素数、cv::Mat の値が使えないとき用

// 深度画像から輪郭抽出時に白い帯が現れる現象の対処
// 次の2行はセットで調整する、適切な組み合わせは自分で探してもよい
// 多少ゴミが残っても確実に輪郭を得るか、多少輪郭が消えても確実にゴミを消すかの選択
// 環境が変わるとすぐ影響を受けるので、あまり神経質に調整する必要はない
// 遠くが見えていて白い帯が出るときは 2.0~3.0 および 0.01~0.001 程度の組み合わせ、
// 白い帯が出ないときは 10.0 以上など大きな値と 0.001 でよい
// カメラが斜め下を見ると白い帯が出るかもしれないが、FARDIST = 1.5 で消えた
const float FARDIST = 2.5; // この距離より遠くは見ない imSobel(), imLaplacian()
const float EDGETH = 0.001; // 輪郭を残すかどうかの閾値

int imFindObjs(const cv::Mat *input, const int filter, const int imagetype, const int show,
	       const char winname[], int cog[][XY], int corners[][LURL][XY]);
int imFindColorObj(const cv::Mat *input, cv::Mat *output, int color);
int imSobel(const cv::Mat *input, cv::Mat *output, int type);
int imLaplacian(const cv::Mat *input, cv::Mat *output, int type);
int imHough(const cv::Mat *input, cv::Mat *optr, std::vector<cv::Vec4i> *lines, const int p[]);
int imLabel(const cv::Mat *input, int label[ROWS][COLS]);
int imCogCorners(const cv::Mat *input, int noo, int label[ROWS][COLS],
		 int cog[][XY], int corners[][LURL][XY]);
void DrawTestImage(cv::Mat *img);
int TestColor2(Uchar r, Uchar g, Uchar b, int color, float rate);

void imShowCogCorners(const cv::Mat input, cv::Mat *optr, const char winname[],
		      int cog[][XY], int corners[][LURL][XY], int rcode);
void imShowCog(const cv::Mat input, cv::Mat *optr, const char winname[],
	       int cog[][XY], int rcode);
void imShowCorners(const cv::Mat input, cv::Mat *optr, const char winname[],
		   int corners[][LURL][XY], int rcode);

// 以下は古い関数
int imFindColorObjOld(RGBCam *c, DepthCam *d, int *x, int *y, int color);
int imSobelOld(RGBCam *c, DepthCam *d, int *x, int *y);

#endif // HEADER_IMAGE
