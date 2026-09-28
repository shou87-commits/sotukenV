// dancemaker.cpp

// 逆運動学を利用して６本の脚を連動させて動かすときの、各脚先の座標を計算する。
// 出力された結果 dance.txt は user.cpp に取り込んで使うことができる。

// $ g++ -o dancemaker dancemaker.cpp
// Usage: ./dancemaker > dance.txt

#include <iostream>
#include <cmath>

#include "const.h"

const int NUMSTATES = 36;
const float RADIUS = 0.3;

float dsin(float d);
float dcos(float d);

int main(void)
{
    float origin[NUMARMS][XYZ] = {{ 0.4, 0.6,-0.3}, { 0.0, 0.6,-0.3}, {-0.4, 0.6,-0.3},
				  { 0.4,-0.6,-0.3}, { 0.0,-0.6,-0.3}, {-0.4,-0.6,-0.3}};
    float devx, devy, devz;
    
    printf("const int NUMSTATES = %d;\n", NUMSTATES);
    printf("float target4[NUMARMS][%d][XYZ] = {\n", NUMSTATES);
    for (int i = 0; i < NUMARMS; i++) {
	putc('{', stdout);
	for (int j = 0; j < NUMSTATES; j++) {
	    devx = RADIUS * dsin(j*10);
	    devy = RADIUS * dcos(j*10);
	    devz = 0.25 * (dsin((NUMSTATES-j)*10)*origin[i][0] + dcos((NUMSTATES-j)*10)*origin[i][1]);
	    putc('{', stdout);
	    printf("%.4f, ", origin[i][0] + devx);
	    printf("%.4f, ", origin[i][1] + devy);
	    printf("%.4f}, ",origin[i][2] + devz);
	}
	puts("},");
    }
    puts("};");
    
    return 0;
}

float dsin(float d)
{
    return(sin(d / 180.0 * M_PI));
}

float dcos(float d)
{
    return(cos(d / 180.0 * M_PI));
}

/*
    float dev[NUMSTATES][XYZ] = {
				 {RADIUS * dsin( 0), RADIUS * dcos( 0), 0.0},
				 {RADIUS * dsin(10), RADIUS * dcos(10), 0.0},
				 {RADIUS * dsin(20), RADIUS * dcos(20), 0.0},
				 {RADIUS * dsin(30), RADIUS * dcos(30), 0.0},
				 {RADIUS * dsin(40), RADIUS * dcos(40), 0.0},
				 {RADIUS * dsin(50), RADIUS * dcos(50), 0.0},
				 {RADIUS * dsin(60), RADIUS * dcos(60), 0.0},
				 {RADIUS * dsin(70), RADIUS * dcos(70), 0.0},
				 {RADIUS * dsin(80), RADIUS * dcos(80), 0.0},
				 {RADIUS * dsin(90), RADIUS * dcos(90), 0.0},
				 {RADIUS * dsin(100), RADIUS * dcos(100), 0.0},
				 {RADIUS * dsin(110), RADIUS * dcos(110), 0.0},
				 {RADIUS * dsin(120), RADIUS * dcos(120), 0.0},
				 {RADIUS * dsin(130), RADIUS * dcos(130), 0.0},
				 {RADIUS * dsin(140), RADIUS * dcos(140), 0.0},
				 {RADIUS * dsin(150), RADIUS * dcos(150), 0.0},
				 {RADIUS * dsin(160), RADIUS * dcos(160), 0.0},
				 {RADIUS * dsin(170), RADIUS * dcos(170), 0.0},
				 {RADIUS * dsin(180), RADIUS * dcos(180), 0.0},
				 {RADIUS * dsin(180+10), RADIUS * dcos(180+10), 0.0},
				 {RADIUS * dsin(180+20), RADIUS * dcos(180+20), 0.0},
				 {RADIUS * dsin(180+30), RADIUS * dcos(180+30), 0.0},
				 {RADIUS * dsin(180+40), RADIUS * dcos(180+40), 0.0},
				 {RADIUS * dsin(180+50), RADIUS * dcos(180+50), 0.0},
				 {RADIUS * dsin(180+60), RADIUS * dcos(180+60), 0.0},
				 {RADIUS * dsin(180+70), RADIUS * dcos(180+70), 0.0},
				 {RADIUS * dsin(180+80), RADIUS * dcos(180+80), 0.0},
				 {RADIUS * dsin(180+90), RADIUS * dcos(180+90), 0.0},
				 {RADIUS * dsin(180+100), RADIUS * dcos(180+100), 0.0},
				 {RADIUS * dsin(180+110), RADIUS * dcos(180+110), 0.0},
				 {RADIUS * dsin(180+120), RADIUS * dcos(180+120), 0.0},
				 {RADIUS * dsin(180+130), RADIUS * dcos(180+130), 0.0},
				 {RADIUS * dsin(180+140), RADIUS * dcos(180+140), 0.0},
				 {RADIUS * dsin(180+150), RADIUS * dcos(180+150), 0.0},
				 {RADIUS * dsin(180+160), RADIUS * dcos(180+160), 0.0},
				 {RADIUS * dsin(180+170), RADIUS * dcos(180+170), 0.0},
    };
*/

/*
    float target[NUMARMS][4][XYZ] = { // 直線歩行パターンの例
       {{ 0.7, 0.4,-0.18}, { 0.7, 0.4,-0.38}, { 0.3, 0.4,-0.38}, { 0.3, 0.4,-0.18}},
       {{-0.2, 0.5,-0.38}, {-0.2, 0.5,-0.18}, { 0.2, 0.5,-0.18}, { 0.2, 0.5,-0.38}},
       {{-0.3, 0.4,-0.18}, {-0.3, 0.4,-0.38}, {-0.7, 0.4,-0.38}, {-0.7, 0.4,-0.18}},
       {{ 0.3,-0.4,-0.38}, { 0.3,-0.4,-0.18}, { 0.7,-0.4,-0.18}, { 0.7,-0.4,-0.38}},
       {{ 0.2,-0.5,-0.18}, { 0.2,-0.5,-0.38}, {-0.2,-0.5,-0.38}, {-0.2,-0.5,-0.18}},
       {{-0.7,-0.4,-0.38}, {-0.7,-0.4,-0.18}, {-0.3,-0.4,-0.18}, {-0.3,-0.4,-0.38}}
    }; 
*/
