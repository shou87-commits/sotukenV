// ftclass.h FTSENSOR用の設定

#ifndef HEADER_FTCLASS
#define HEADER_FTCLASS

#define BYOU 2 //計測時間s(これによって配列の長さも変わる)

class FTSENSOR{
private:
    ros::Subscriber force_sub;
    ros::NodeHandle nh;
    const char *sname; //サブスクライバ
public:
    int id; //リンクのid
	int tail;
	float f[3][BYOU*100]={{0.123}}; //lfの２本指
	//float dami[10000];

public:
    FTSENSOR(int handid, const char sn[])
    {
    	id = handid; //lf1=A8J1, lf2=A8J2, rf1=A8J3, rf2=A8J4 詳しくはconst.hを参照
    	sname = sn; //サブスクライバ
    	force_sub = nh.subscribe<geometry_msgs::WrenchStamped>(sname, 1, &FTSENSOR::FtCallback, this);
    }

    ~FTSENSOR(){}
	
    void FtCallback(const geometry_msgs::WrenchStampedConstPtr& raw)
    {
	int sec = raw->header.seq%(BYOU*100); //ftsensorのもともとあるseqを利用する
	//int tail = sec;
	tail = sec;
	int head = tail+1;
	if(head == (BYOU*100)) head = 0; //リングバッファのため

	//ROS_INFO("head=%d tail=%d",head,tail);

	/* 受け取ったidによってハンドのどこの情報かわかる
	   それぞれの配列に格納する
	   格納の仕方はリングバッファ(先入れ先出し)
	   データを読みたいときは注意 */
	f[0][tail] = raw->wrench.force.x;
	f[1][tail] = raw->wrench.force.y;
	f[2][tail] = raw->wrench.force.z;

	//ROS_INFO("force  x:%.3f y:%.3f z:%.3f", f[0][tail], f[1][tail], f[2][tail]);
	//どんな情報を読み取っているか確認できる
	if(raw->header.seq%100 == 1){	//１秒ごとに表示
	    //ROS_INFO("force  x:%.3f y:%.3f z:%.3f \n", raw->wrench.force.x,
	    //         raw->wrench.force.y, raw->wrench.force.z);
	    //ROS_INFO("force  x:%.3f y:%.3f z:%.3f", f[0][tail], f[1][tail], f[2][tail]); //値確認
	    //ROS_INFO("head=%d tail=%d",head,tail); // headとtailの配列の場所確認
	    //ROS_INFO("id=%p tail=%p f=%p",&id,&tail,f); // idとtailとfのアドレスを確認
    	}
    }

    void Ftyomu(float box[3][BYOU*100]){ //配列そのまま渡す関数
	for(int i=0; i<BYOU*100; i++){
	    box[0][i] = f[0][i];
	    box[1][i] = f[1][i];
	    box[2][i] = f[2][i];
	    //ROS_INFO("force  x:%.3f y:%.3f z:%.3f", f[0][i], f[1][i], f[2][i]);
	}
    }

    void Ftkoredake(int i){ // 問題時の表示用 prg4.cppの中に入れて使った、今は使ってない
	ROS_INFO("kore id=%p tail=%p f=%p",&id,&tail,f);
    }

    void Ftmax(float maxx[], float maxy[], float maxz[]) //最大値(絶対値)を入れる関数
    {
	maxx[0]=0; maxy[0]=0; maxz[0]=0; //初期化
	float x=0;
	float y=0;
	float z=0;
	float x2,y2,z2; //2乗用

	for(int i=0; i<BYOU*100; i++){
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
        for(int i=0; i<BYOU*100; i++){
            //ROS_INFO("force  x:%.3f y:%.3f z:%.3f", f[0][i], f[1][i], f[2][i]);
            //ROS_INFO("force  sumx:%.3f sumy:%.3f sumz:%.3f", sumx, sumy, sumz);
            sumx = sumx + f[0][i];
            sumy = sumy + f[1][i];
            sumz = sumz + f[2][i];
        }
        /* BYOU間の平均値を求める */
        avex[0] = sumx/(BYOU*100);
        avey[0] = sumy/(BYOU*100);
        avez[0] = sumz/(BYOU*100);

        //ROS_INFO("force  sumx:%.3f sumy:%.3f sumz:%.3f \n", sumx, sumy, sumz);
        ROS_INFO("ave x:%.3f y:%.3f z:%.3f", avex[0],avey[0],avez[0]);
    }

    int Ftdecision(float decix[], float deciy[], float deciz[])     //当たっているか確認する関数
    {
	float maxx[1],maxy[1],maxz[1];
	float avex[1],avey[1],avez[1];

	Ftmax(maxx, maxy, maxz);
	Ftave(avex, avey, avez);

	/*最大値を材料にして判断する場合*/
        decix[0] = maxx[0];
        deciy[0] = maxy[0];
        deciz[0] = maxz[0];

	/*平均を材料にして判断する場合*/
	/*
	decix[0] = avex[0];
	deciy[0] = avey[0];
	deciz[0] = avez[0];
	*/

	/*念の為に初期化*/
	decix[1] = 3;
        deciy[1] = 3;
        deciz[1] = 3;

	/* 以下ONかOFFかの判定 配列の1番目にONかOFFか入れる
	   xy軸は押されると-になるので注意、z軸は押されると+になる */
	if(decix[0] >= 3){
	    decix[1] = FTON;
	    //return ON;
	} else decix[1] = FTOFF;

	if(deciy[0] >= 3 || deciy[0] <= -3){ //現在はy軸の結果をreturnで返している 必要であれば変える
	    deciy[1] = FTON;
	    return ON;
	} else deciy[1] = FTOFF;

	if(deciz[0] >= 3 || deciz[0] <=-3 ){
	    deciz[1] = FTON;
	    //return ON;
	} else deciz[1] = FTOFF;

	return OFF;
    }
    
};

#endif // HEADER_FTCLASS
