// armclass.cpp

#include "const.h"
//#include "user.h"
#include "motorclass.h"
#include "ftclass.h"

extern C1 *gMotor;
extern FTSENSOR *gFT;

template <typename TYPE>
void setparam(TYPE dst[], TYPE src[], int n);

class ARM
{
public:
    ARM(int armid)
    {
	id = armid;
	lastxyz[0] = lastxyz[1] = lastxyz[2] = 0.0; // 逆運動学目標座標初期値

// この脚が持つリンクの長さの設定、現在はすべての脚が同じ構造を想定している
	float l[NUMLINKS] = {0.05, 0.3, 0.3, 0.05, 0.05, 0.05}; // 脚の各リンク長
	setparam(linklen, l, NUMLINKS); // リンクの長さ設定

// この脚が持つ関節の番号の設定、現在はすべての脚が同じ個数の関節を持つ想定
	if (LF == id) {
	    int j[] = {A1J1, A1J2, A1J3, A1J4, A1J5, A1J6};
	    setparam(j_idx, j, NUMJOINTS); j_idx[NUMJOINTS] = EOLV; // ジョイントの番号設定
	}
	else if (LM == id) {
	    int j[] = {A2J1, A2J2, A2J3, A2J4, A2J5, A2J6};
	    setparam(j_idx, j, NUMJOINTS); j_idx[NUMJOINTS] = EOLV; // ジョイントの番号設定
	}
	else if (LB == id) {
	    int j[] = {A3J1, A3J2, A3J3, A3J4, A3J5, A3J6};
	    setparam(j_idx, j, NUMJOINTS); j_idx[NUMJOINTS] = EOLV; // ジョイントの番号設定
	}
	else if (RF == id) {
	    int j[] = {A4J1, A4J2, A4J3, A4J4, A4J5, A4J6};
	    setparam(j_idx, j, NUMJOINTS); j_idx[NUMJOINTS] = EOLV; // ジョイントの番号設定
	}
	else if (RM == id) {
	    int j[] = {A5J1, A5J2, A5J3, A5J4, A5J5, A5J6};
	    setparam(j_idx, j, NUMJOINTS); j_idx[NUMJOINTS] = EOLV; // ジョイントの番号設定
	}
	else if (RB == id) {
	    int j[] = {A6J1, A6J2, A6J3, A6J4, A6J5, A6J6};
	    setparam(j_idx, j, NUMJOINTS); j_idx[NUMJOINTS] = EOLV; // ジョイントの番号設定
	}
	j_idx[NUMJOINTS] = EOLV; // 配列の終端を表す、必須ではないが念の為
	
// FTセンサによる停止の可否の設定
	ftstopflag = OFF; // FTセンサにより停止させない場合、または次行
//	ftstopflag = ON;  // FTセンサにより停止させる場合
// 例として、左中脚だけFTセンサによる停止を有効にするデモ
//	if (LM == id) ftstopflag = ON;

	int ary[NUMARMFT]; // 設定準備用の配列
	for (int i = 0; i < NUMARMFT; i++) ary[i] = EOLV; // 最初にEndOfListVal で初期化
// 必要な脚のみ停止判定用 FT センサの番号を設定する
	if      (LF == id) {ary[0] = FTA1S1;} // 複数個指定も可 {ary[0] = FTA1S1; ary[1] = FT5;}
	else if (LM == id) {ary[0] = FTA2S1;}
	else if (LB == id) {ary[0] = FTA3S1;}
	else if (RF == id) {ary[0] = FTA4S1;}
	else if (RM == id) {ary[0] = FTA5S1;}
	else if (RB == id) {ary[0] = FTA6S1;}
// 最後にセンサ番号を登録する
	setparam(ft_idx, ary, NUMARMFT);
    }
    void set_xyz(float a[XYZ])
    {
	lastxyz[0] = a[0];
	lastxyz[1] = a[1];
	lastxyz[2] = a[2];
    }
    void get_xyz(float a[XYZ])
    {
	a[0] = lastxyz[0];
	a[1] = lastxyz[1];
	a[2] = lastxyz[2];
    }
    void ftstop()
    {
	int i = 0;
	if (OFF == ftstopflag) return;
	while (EOLV != ft_idx[i]) {
	    if (ft_thrd < gFT[ft_idx[i]]. Ftlastave()) { // ftセンサが一個でも反応したら
		for (int j = 0; j < NUMJOINTS; j++) { // アームの全関節を停止
		    gMotor[j_idx[j]]. Stop();
		}
	    }
	    i++;
	}
    }
    void setftflag(int f)
    {
	ftstopflag = f;
    }
private:
    int id;
    float linklen[NUMLINKS]; // ハンドは含まない
    int j_idx[NUMJOINTS];    // ハンドは含まない、このアームに含まれるジョイントの番号
    float lastxyz[XYZ];      // 逆運動学、直前の目標座標

    int ftstopflag;          // FTセンサによって停止するかの可否 ON or OFF
    int ft_idx[NUMARMFT];    // このアームの停止に関係するFTセンサの番号
    float ft_thrd = 2.5;     // FTセンサの閾値、これを越えたら反応ありとする
};
