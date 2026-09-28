// ftclass.h FTSENSOR 用の設定

#ifndef HEADER_FTCLASS
#define HEADER_FTCLASS

#include <geometry_msgs/WrenchStamped.h>

#include "const.h"

#define FTBYOU 2 // 計測時間 sec (これによって配列の長さも変わる)

class FTSENSOR {
private:
    ros::Subscriber force_sub;
    ros::NodeHandle nh;
    const char *sname; //サブスクライバ
public:
    int id; // リンクのid
    int tail;
    float f[XYZ][FTBYOU*100]={{0.123}}; // センサ1個につき XYZ 3軸の力が記録される
//  float dammy[10000]; // デバッグ用、今は不要

public:
    FTSENSOR(int handid, const char sn[])
    {
    	id = handid; // lf1=A8J1, lf2=A8J2, rf1=A8J3, rf2=A8J4 詳しくはconst.hを参照
	             // 上記、正しくは FTA1J1 -- FTA4J2
    	sname = sn;  // サブスクライバ
    	force_sub = nh.subscribe<geometry_msgs::WrenchStamped>(sname, 1, &FTSENSOR::FtCallback, this);
    }

    ~FTSENSOR(){}
	
    void FtCallback(const geometry_msgs::WrenchStampedConstPtr& raw)
    {
	int sec = raw->header.seq % (FTBYOU*100); // ftsensorのもともとあるseqを利用する
	tail = sec;
	int head = tail + 1;
	if (head == (FTBYOU*100)) head = 0; // リングバッファのため

//      ROS_INFO("head=%d tail=%d",head,tail);

/*      メンバ変数 id によってどこのセンサの情報かわかる
        センサのインスタンスそれぞれの配列に格納される
        格納の仕方はリングバッファ(先入れ先出し)、データ読出時は注意 
*/
	f[0][tail] = raw->wrench.force.x;
	f[1][tail] = raw->wrench.force.y;
	f[2][tail] = raw->wrench.force.z;

//	ROS_INFO("%.2f", fabs(f[0][tail]) + fabs(f[1][tail]) + fabs(f[2][tail]));
/*	static int t = 0;
	if (0 == (t++ % 50)) { // どのセンサが反応しているか 50 回に1回表示する
	    float sum = Ftlastave(); // 直近の平均を求める
	    if (2 < sum) { // 表示の可否の閾値、使用者が自由に決めてよい
		char c;
		if      (id == FTA1J1 || id == FTA1J2) c = 'L';
		else if (id == FTA4J1 || id == FTA4J2) c = 'R';
		else if (id == FT5)                    c = '5';
		else                                   c = '?';
//		printf("ft%c:%.1f ", c, sum); // 一旦表示を止めている
	    }
	}
*/
	// どんな情報を読み取っているか確認できる
	if(raw->header.seq%100 == 1){	//１秒ごとに表示
	    //ROS_INFO("f x:%.3f y:%.3f z:%.3f", raw->wrench.force.x, raw->wrench.force.y,
	    //                                   raw->wrench.force.z);
	    //ROS_INFO("f x:%.3f y:%.3f z:%.3f", f[0][tail], f[1][tail], f[2][tail]); //入ってる値確認
	    //ROS_INFO("head=%d tail=%d", head, tail); //headとtailの配列の場所確認
	    //ROS_INFO("id=%p tail=%p f=%p", &id, &tail, f); //idとtailとfのアドレスを確認
    	}
    }

    void Ftyomu(float box[XYZ][FTBYOU*100]){ // 配列をそのまま渡す関数
	for(int i=0; i<FTBYOU*100; i++){
	    box[0][i] = f[0][i];
	    box[1][i] = f[1][i];
	    box[2][i] = f[2][i];
	    //ROS_INFO("force  x:%.3f y:%.3f z:%.3f", f[0][i], f[1][i], f[2][i]);
	}
    }

    void Ftkoredake(int i){ //処理がおかしい時の表示用 prg4.cppの中で使った、今は使ってない
	ROS_INFO("kore id=%p tail=%p f=%p",&id,&tail,f);
    }

    void Ftmax(float maxx[], float maxy[], float maxz[]){ 	//最大値(絶対値)を入れる関数
	maxx[0]=0; maxy[0]=0; maxz[0]=0; //初期化
	float x=0;
	float y=0;
	float z=0;
	float x2,y2,z2; //2乗用

	for(int i=0; i<FTBYOU*100; i++){
	    x2 = f[0][i] * f[0][i];	//-と+で出るので絶対値で取っている　問題があれば変える
	    y2 = f[1][i] * f[1][i];
	    z2 = f[2][i] * f[2][i];

	    if(x*x < x2) x = f[0][i];
	    if(y*y < y2) y = f[1][i];
	    if(z*z < x2) z = f[2][i];
	}
	maxx[0] = x;
	maxy[0] = y;
	maxz[0] = z;
	ROS_INFO("max x:%.3f y:%.3f z:%.3f", x, y, z);
    }

    void Ftave(float avex[], float avey[], float avez[])	//平均を求める関数
    {
	float sumx=0;
        float sumy=0;
        float sumz=0;

        //ROS_INFO("deci id=%p tail=%p f=%p",&id,&tail,f); //idとtailとfのアドレスを確認
        //ROS_INFO("decision %d ", tail);
        for(int i=0; i<FTBYOU*100; i++){
            //ROS_INFO("force  x:%.3f y:%.3f z:%.3f", f[0][i], f[1][i], f[2][i]);
            //ROS_INFO("force  sumx:%.3f sumy:%.3f sumz:%.3f", sumx, sumy, sumz);
            sumx = sumx + f[0][i];
            sumy = sumy + f[1][i];
            sumz = sumz + f[2][i];
        }
        /* FTBYOU 間の平均値を求める */
        avex[0] = sumx/(FTBYOU*100);
        avey[0] = sumy/(FTBYOU*100);
        avez[0] = sumz/(FTBYOU*100);

        //ROS_INFO("force  sumx:%.3f sumy:%.3f sumz:%.3f \n", sumx, sumy, sumz);
        ROS_INFO("ave x:%.3f y:%.3f z:%.3f", avex[0],avey[0],avez[0]);
    }

    int Ftdecision(float decix[], float deciy[], float deciz[])     //当たっているか確認する関数
    {
	float maxx[1],maxy[1],maxz[1];
	float avex[1],avey[1],avez[1];

	Ftmax(maxx, maxy, maxz);
	Ftave(avex, avey, avez);

	// 最大値を材料にして判断する場合
        decix[0] = maxx[0];
        deciy[0] = maxy[0];
        deciz[0] = maxz[0];

	// 平均を材料にして判断する場合
	/*
	decix[0] = avex[0];
	deciy[0] = avey[0];
	deciz[0] = avez[0];
	*/

	// 念の為に初期化
	decix[1] = 3;
        deciy[1] = 3;
        deciz[1] = 3;

	/* 以下 ON か OFF かの判定 配列の1番目にONかOFFか入れる
	   xy軸は押されると-になるので注意、z軸は押されると+になる */
	if(decix[0] >= 3){
	    decix[1] = FTON;
	    //return ON;
	} else decix[1] = FTOFF;

	if(deciy[0] >= 3 || deciy[0] <= -3){ //現在はy軸の結果をreturnで返している
	    deciy[1] = FTON;
	    return ON;
	} else deciy[1] = FTOFF;

	if(deciz[0] >= 3 || deciz[0] <= -3){
	    deciz[1] = FTON;
	    //return ON;
	} else deciz[1] = FTOFF;

	return OFF;
    }

    float Ftlastave(void)	// 直近 dnum 個の平均を求める関数
    {
	float sumx = 0.0, sumy = 0.0, sumz = 0.0;

	int dnum = 10; // データの個数、50だと0.5秒
	int start = tail - (dnum - 1); // リングバッファからデータを取り出す先頭
	int end = tail;                // リングバッファからデータを取り出す最後

	if ((dnum - 1) <= end) { // 配列中の連続した範囲から取り出せる場合
	    for(int i = start; i <= end; i++) {
//		fprintf(stderr, "%d ", i); // 範囲を正しく設定できているか確認
		sumx = sumx + f[0][i];
		sumy = sumy + f[1][i];
		sumz = sumz + f[2][i];
	    }
	}
	else { // 配列中の不連続の範囲（先頭と最後の2箇所に分かれる）の場合
	    for(int i = 0; i <= end; i++) { // 配列の先頭部分
//		fprintf(stderr, "%d ", i);
		sumx = sumx + f[0][i];
		sumy = sumy + f[1][i];
		sumz = sumz + f[2][i];
	    }
	    start += (FTBYOU*100);
	    end = FTBYOU*100;
	    for(int i = start; i < end; i++) { // 配列の末尾部分
//		fprintf(stderr, "%d ", i);
		sumx = sumx + f[0][i];
		sumy = sumy + f[1][i];
		sumz = sumz + f[2][i];
	    }
	}
//	fprintf(stderr, "\n");

        sumx = sumx / dnum; // XYZ各要素の平均
        sumy = sumy / dnum;
        sumz = sumz / dnum;
	return sqrt(sumx*sumx + sumy*sumy + sumz*sumz); // ベクトルの長さに相当
    }

};

#endif // HEADER_FTCLASS
