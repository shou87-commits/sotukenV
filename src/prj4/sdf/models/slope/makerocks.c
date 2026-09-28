// $ gcc -o makerocks makerocks.c
// $ ./makerocks > rocks.sdf
// $ cat sdfhead.sdf rocks.sdf sdftail.sdf > model.sdf
// ~/.gazebo/models/slope/ に model.sdf を置く
// abc.world 内から slope を読み込むように登録する
// $ gazebo abc.world

#include <stdio.h>
#include <stdlib.h>

#define WIDTH	10.0	// 斜面の中心の周囲の幅、この距離まで対象とする [m]
#define HEIGHT 	9.0	// 斜面の中心の周囲の奥行き、この距離まで対象とする [m]
#define NUMX 	5	// 上で指定した幅の範囲に何個置くか、個数
#define NUMY 	5	// 上で指定した奥行きの範囲に何個置くか、個数
#define ITVLX 	(WIDTH / NUMX) // 横方向の岩の間隔 [m]
#define ITVLY 	(HEIGHT / NUMY) // 縦方向の岩の間隔 [m]
#define MAXKIND 1	// 岩の種類
#define RANDRANGE (ITVLX / 2 * 0.7) // 岩を置く位置を乱数で変える範囲 [m]
#define MINRADIUS 0.1	// 岩の大きさの最小値 [m]
#define MAXRADIUS 0.3	// 岩の大きさの最大値 [m]

int main(void)
{
    int i, j, kind;
    float x, y, rx, ry, rz, r, p, yo;

    for (i = 0; i < NUMY; i++) {
	for (j = 0; j < NUMX; j++) {
	    x = ITVLX * j + ITVLX *  0.5 - (WIDTH /2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    y = ITVLY *	i + ITVLY *  0.5 - (HEIGHT/2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    rx = MINRADIUS + (random() % 1000 / 1000.0 * (MAXRADIUS - MINRADIUS));
	    ry = MINRADIUS + (random() % 1000 / 1000.0 * (MAXRADIUS - MINRADIUS));
	    rz = MINRADIUS + (random() % 1000 / 1000.0 * (MAXRADIUS - MINRADIUS));
	    r = (random() % 1000 - 500) / 500.0 * 3.1416 / 3;
	    p = (random() % 1000 - 500) / 500.0 * 3.1416 / 3;
	    yo= (random() % 1000 - 500) / 500.0 * 3.1416 / 3;
	    kind = random() % MAXKIND + 1;

	    printf("<collision name=\"rock%d%dcollision\">\n", i, j);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, 10+MINRADIUS/2, r, p, yo);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
	    printf("</geometry>\n</collision>\n");

	    printf("<visual name=\"rock%d%dvisual\">\n", i, j);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, 10+MINRADIUS/2, r, p, yo);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
	    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
            printf("<name>Gazebo/Granite</name> </script> </material>\n</visual>\n\n");
	}
    }
    exit(EXIT_SUCCESS);
}
