// $ gcc -o makestep makestep.c
// $ ./makestep > step.sdf
// $ cat sdfhead.sdf step.sdf sdftail.sdf > model.sdf
// ~/.gazebo/models/slope/ に model.sdf を置く
// abc.world 内から slope を読み込むように登録する
// $ gazebo abc.world

#include <stdio.h>
#include <stdlib.h>

#define WIDTH	10.0	// 斜面の中心の周囲の幅、この距離まで対象とする [m]
#define HEIGHT 	9.0	// 斜面の中心の周囲の奥行き、この距離まで対象とする [m]
#define NUMX 	10	// 上で指定した幅の範囲に何個置くか、個数
#define NUMY 	10	// 上で指定した奥行きの範囲に何個置くか、個数
#define ITVLX 	(WIDTH / NUMX) // 横方向の岩の間隔 [m]
#define ITVLY 	(HEIGHT / NUMY) // 縦方向の岩の間隔 [m]
#define MAXKIND 1	// 段差の見た目の種類
#define RANDRANGE (ITVLX / 2 * 0.7) // 段差を置く位置を乱数で変える範囲 [m]
#define MINRADIUS 0.1	// 段差の長さの最小値 [m]
#define MAXRADIUS 0.3	// 段差の長さの最大値 [m]

int main(void)
{
    int i, j, kind;
    float x, y, rx, ry, rz, r, p, yo;

    for (i = 0; i < NUMY; i++) {
	for (j = 0; j < NUMX; j++) {
	    x = ITVLX * j + ITVLX *  0.5 - (WIDTH /2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    y = ITVLY *	i + ITVLY *  0.5 - (HEIGHT/2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    rx = 0.05;
	    ry = MINRADIUS + (random() % 1000 / 1000.0 * (MAXRADIUS - MINRADIUS));
	    rz = 0.05;
	    r = 0;
	    p = 0;
	    yo= (random() % 1000 - 500) / 500.0 * 3.1416 / 4;
	    kind = random() % MAXKIND + 1;

	    printf("<collision name=\"step%d%dcollision\">\n", i, j);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, 10+rz/2, r, p, yo);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
	    printf("</geometry>\n</collision>\n");

	    printf("<visual name=\"step%d%dvisual\">\n", i, j);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, 10+rz/2, r, p, yo);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
	    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
            printf("<name>Gazebo/Soil1</name> </script> </material>\n</visual>\n\n");
	}
    }
    exit(EXIT_SUCCESS);
}
