// motorclass.h

#ifndef	HEADER_MOTORCLASS
#define	HEADER_MOTORCLASS

class C1
{
public:
    C1(char *name, int id, float param[NUMPARAM]);
    void timerCallback(const ros::TimerEvent&);
    void CheckCollision(int id);
    int  CheckCollisionCW(int from, int to); // OK or NO
    int  CheckCollisionCCW(int from, int to);
    void Stop();
    void setftflag(int f) { ftstopflag = f; }
    std_msgs::Float64 anglenow;    // モータの現在角度
    std_msgs::Float64 targetangle; // モータの目標角度
    float duration;         // この時間で回転する、単位は秒
    int ctrlmode;           // 制御の種類 MODENONE, MODEPROFILE, MODEPROFILE2
    int state;              // モータの状態、MOVING, DONE 
private:
    int id;
    float lasttarget;       // 以前の目標角度、初期値は意味のない仮の値
    float width;            // 回転させたい角度の幅
    int goal;               // 制御の指示を繰り返す合計回数
    int count;              // プロファイルをどこまで実行したかカウント、記憶する
    float profile[PROFSIZE]; // プロファイル、目標角度の時系列、MotorCtrlITVL==0.01なら10.24秒

    int ftstopflag;         // FTセンサによって停止するかの可否 ON or OFF
    int ft_idx[NUMMOTORFT];   // このモータの停止に関係するFTセンサの番号
    float ft_thrd = 2.5;     // FTセンサの閾値、これを越えたら反応ありとする
    
    const float AttackTime = 0.25; // プロファイル中の立ち上がり時間の割合
    const float ReleaseTime = 0.25;// プロファイル中の立ち下がり時間の割合
    const float SustainTime = 1.0 - AttackTime - ReleaseTime; // プロファイル中の持続時間の割合
    const float Attack  = 0.1;     // 立ち上がり区間で回転させる角度の全角度に占める割合
    const float Release = 0.1;     // 立ち下がり区間で回転させる角度の全角度に占める割合
    const float Sustain = 1.0 - Attack - Release; // 持続区間の回転角度の割合

    ros::Publisher c_pub;
    ros::Timer timer;
    ros::NodeHandle nh;
};

class C2
{
public:
    C2(char *name, int id, float param[NUMPARAM]);
    void timerCallback(const ros::TimerEvent&);
private:
    int id;
    ros::Publisher c_pub;
    ros::Timer timer;
    ros::NodeHandle nh;
};

#endif // HEADER_MOTORCLASS
