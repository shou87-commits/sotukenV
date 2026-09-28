// $ gcc -o makeforest makeforest.c
// $ ./makeforest > forest.sdf
// $ cat sdfhead.sdf forest.sdf sdftail.sdf > model.sdf
// ~/.gazebo/models/slope/ に model.sdf を置く
// abc.world 内から slope を読み込むように登録する
// $ gazebo abc.world

#include <stdio.h>
#include <stdlib.h>

#define WIDTH	10.0	// 斜面の中心の周囲の幅、この距離まで対象とする [m]
#define HEIGHT 	9.0	// 斜面の中心の周囲の奥行き、この距離まで対象とする [m]
#define NUMX 	4	// 上で指定した幅の範囲に何本植えるか、本数
#define NUMY 	4	// 上で指定した奥行きの範囲に何本植えるか、本数
#define ITVLX 	(WIDTH / NUMX) // 横方向の木の間隔 [m]
#define ITVLY 	(HEIGHT / NUMY) // 縦方向の木の間隔 [m]
#define MAXKIND 7	// 木の種類
#define RANDRANGE (ITVLX / 2 * 0.7) // 木を植える位置を乱数で変える範囲 [m]
#define MINRADIUS 0.15	// 木の幹の最小半径 [m]
#define MAXRADIUS 0.4	// 木の幹の最大半径 [m]
#define THEIGHT 5.0     // 木の高さ

int main(void)
{
    int i, j, kind;
    float x, y, radius;

    for (i = 0; i < NUMY; i++) {
	for (j = 0; j < NUMX; j++) {
	    x = ITVLX * j + ITVLX *  0.5 - (WIDTH /2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    y = ITVLY *	i + ITVLY *  0.5 - (HEIGHT/2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    kind = random() % MAXKIND + 1;
	    radius = MINRADIUS + (MAXRADIUS - MINRADIUS) * (random() % 100) / 100.0;

	    printf("<collision name=\"wood%d%dcollision\">\n", i, j);
	    printf("<pose>%f %f 11.56 0 -0.7845 0</pose>\n", x-1.96, y);
	    printf("<geometry> <cylinder> <radius>%f</radius> ", radius);
	    printf("<length>%f</length> </cylinder> </geometry>\n</collision>\n", THEIGHT);

	    printf("<visual name=\"wood%d%dvisual\">\n", i, j);
	    printf("<pose>%f %f 11.56 0 -0.7845 0</pose>\n", x-1.96, y);
	    printf("<geometry> <cylinder> <radius>%f</radius> ", radius);
	    printf("<length>%f</length> </cylinder> </geometry>\n", THEIGHT);

	    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
            printf("<name>Gazebo/Tree%d</name> </script> </material>\n</visual>\n\n", kind);
	}
    }
    exit(EXIT_SUCCESS);
}
