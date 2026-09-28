// $ gcc -o makegrass makegrass.c
// $ ./makegrass > grass.sdf
// $ cat sdfhead.sdf grass.sdf sdftail.sdf > model.sdf
// ~/.gazebo/models/slope/ に model.sdf を置く
// abc.world 内から slope を読み込むように登録する
// $ gazebo abc.world

#include <stdio.h>
#include <stdlib.h>

#define WIDTH	10.0	// 斜面の中心の周囲の幅、この距離まで対象とする [m]
#define HEIGHT 	9.0	// 斜面の中心の周囲の奥行き、この距離まで対象とする [m]
#define NUMX 	15	// 上で指定した幅の範囲に何本植えるか、本数
#define NUMY 	15	// 上で指定した奥行きの範囲に何本植えるか、本数
#define ITVLX 	(WIDTH / NUMX) // 横方向の草の間隔 [m]
#define ITVLY 	(HEIGHT / NUMY) // 縦方向の草の間隔 [m]
#define MAXKIND 1	// 草の種類
#define RANDRANGE (ITVLX / 2 * 0.7) // 草を植える位置を乱数で変える範囲 [m]
#define MINRADIUS 0.01	// 草の太さの最小半径 [m]
#define MAXRADIUS 0.04	// 草の太さの最大半径 [m]
#define GHEIGHT	0.15	// 草の高さ

int main(void)
{
    int i, j, kind;
    float x, y, radius, r, p;

    for (i = 0; i < NUMY; i++) {
	for (j = 0; j < NUMX; j++) {
	    x = ITVLX * j + ITVLX *  0.5 - (WIDTH /2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    y = ITVLY *	i + ITVLY *  0.5 - (HEIGHT/2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    r = (random() % 1000 - 500) / 500.0 * 3.1416 / 10;
	    p = (random() % 1000 - 500) / 500.0 * 3.1416 / 10;
	    kind = random() % MAXKIND + 1;
	    radius = MINRADIUS + (MAXRADIUS - MINRADIUS) * (random() % 100) / 100.0;

	    printf("<collision name=\"grass%d%dcollision\">\n", i, j);
	    printf("<pose>%f %f %f %f %f 0</pose>\n", x, y, 10+GHEIGHT/2, r, p);
	    printf("<geometry> <cylinder> <radius>%f</radius> ", radius);
	    printf("<length>%f</length> </cylinder> </geometry>\n</collision>\n", GHEIGHT);

	    printf("<visual name=\"grass%d%dvisual\">\n", i, j);
	    printf("<pose>%f %f %f %f %f 0</pose>\n", x, y, 10+GHEIGHT/2, r, p);
	    printf("<geometry> <cylinder> <radius>%f</radius> ", radius);
	    printf("<length>%f</length> </cylinder> </geometry>\n", GHEIGHT);

	    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
            printf("<name>Gazebo/Grass2</name> </script> </material>\n</visual>\n\n");
	}
    }
    exit(EXIT_SUCCESS);
}
