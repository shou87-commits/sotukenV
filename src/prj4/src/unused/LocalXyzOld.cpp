// LocalXyz() 旧版

void LocalXyz(const int ix, const int iy, float *x, float *y, float *z,
		  const float ha, const float va) const
// カメラ画像中の座標(ix, iy)の画素の３次元ローカル座標を求めて x, y, z に代入する。
// ha(horizontal angle), va(vertical angle) は首の水平、垂直回転角度を表す。
// ha, va から首が回転しても正しい座標になるように補正する。
{
	xyd2xyz(ix, iy, x, y, z); // カメラのCCD撮像面からの座標を求める
	adjustrotate(ha, va, x, y, z); // 首の回転による座標のズレを補正する
	*x += 0.15; // ボディ中心からの座標に変換する
	*z += 0.25; // ここまでで完全なローカル座標を求められた
//	fprintf(stderr, "(%.3f %.3f %.3f) ", *x, *y, *z);
// 赤い箱を使うキャリブレーションで *x が 1.15 ではなく 1.13 になるのは、箱の大きさが
// あるため手前の表面が 10mm 程カメラに近くなるためで間違いではない
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
	    *y = (ix - cx) * *x / -382;
	    *z = (iy - cy) * *x / -382;
	}
}

void adjustrotate(const float ha, const float va, float *x, float *y, float *z) const
// ha(horizontal angle), va(vertical angle) は首の水平、垂直回転角度を表す。
// ha, va から首が回転しても正しい座標になるように補正する。
{
	float tx, ty, tz; // 計算途中で *x, *y, *z は変更不可なのでこれらが必要

//	printf("ha:%.2f va:%.2f x:%.3f y:%.3f z:%.3f -> ", ha, va, *x, *y, *z);

	if (0.01 < fabs(ha)) { // 水平回転あり (1deg == 0.0174442)
	    float mha = ha * 1;
// 3次元座標の結果がおかしい場合は、ha * 1, ha * -1 の結果を比べてみる。
// ha をどこから取得するかによって符号が変化する。
	    tx = (cos(mha) * *x) + (-sin(mha) * *y);
	    ty = (sin(mha) * *x) + ( cos(mha) * *y);
	    *x = tx;
	    *y = ty;
	}
	if (0.01 < fabs(va)) { // 垂直回転あり
	    float mva = va * -1;
// 3次元座標の結果がおかしい場合は、va * 1, va * -1 の結果を比べてみる。
// va をどこから取得するかによって符号が変化する。
	    tx = (cos(mva) * *x) + (-sin(mva) * *z);
	    tz = (sin(mva) * *x) + ( cos(mva) * *z);
	    *x = tx;
	    *z = tz;
// 垂直回転は回転軸とCCD撮像面が離れているため、回転するとCCDが前後に移動する。
// 以下はその補正だが、簡易的な処理なので精度が必要なときは真面目に計算すべし。
// 係数の符号は、ha, va と同様、正負の結果を比較する。

	    *x += (va / 3.1416 * 180 * 2 / 1000);
	    *z += (fabs(va) / 3.1416 * 180 * -6 / 10000);

//	    if (0.0 < va) *x += (va / 3.1416 * 180 * 2 / 1000); 
//	    if (va < 0.0) *x += (va / 3.1416 * 180 * 2 / 1000); 

//	    if (0.0 < va) *z += (va / 3.1416 * 180 * -6 / 10000);
//	    if (va < 0.0) *z += (va / 3.1416 * 180 * 5 / 10000);

/* va の変化と x 座標、 z 座標の関係
va(deg)  x (real 1.0)  z (real 0.0)
--------------------------------------
-30        1.050       0.011
-20        1.029       0.004
-10        1.008       0.0
0          0.999       0.0
10         0.964       0.005
20         0.943       0.014
30         0.924       0.017
*/
	}
//	printf("x:%.3f y:%.3f z:%.3f\n", *x, *y, *z);
}
