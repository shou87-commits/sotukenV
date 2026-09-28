// $ gcc -o makebldwall makebldwall.c
// $ ./makebldwall > bldwall.sdf
// $ cat sdfhead.sdf bldwall.sdf sdftail.sdf > model.sdf
// ~/.gazebo/models/bldwall/ に model.sdf を置く、またはリンクを張る
// abc.world 内から bldwall を読み込むように登録する
// $ gazebo abc.world

#include <stdio.h>
#include <stdlib.h>

/* オリジナル
#define WIDTH	8.0	// 壁の幅 [m]
#define HEIGHT 	8.0	// 壁の高さ [m]
#define THICK 	0.5	// 壁の厚さ [m]
#define LENGTH 	0.05	// 根本の石の長さ [m]
#define LENGTH2	0.1	// つかむ石の大きさ [m]
#define NUM 	8	// 上で指定した範囲に縦横何個ずつ置くか、個数
#define ITVLX 	(WIDTH  / NUM) // 石の横の間隔 [m]
#define ITVLY 	(HEIGHT / NUM) // 石の縦の間隔 [m]
#define RANDRANGE (ITVLX / 2 * 0.75) // 石を置く位置を乱数で変える範囲 [m]
*/

#define WIDTH	5.0	// 壁の幅 [m]
#define HEIGHT 	5.0	// 壁の高さ [m]
#define THICK 	0.2	// 壁の厚さ [m]
#define LENGTH 	0.25	// 根本の石の長さ [m]
#define LENGTH2	0.075	// つかむ石の大きさ [m]
#define NUM 	10	// 上で指定した範囲に縦横何個ずつ置くか、個数
#define ITVLX 	(WIDTH  / NUM) // 石の横の間隔 [m]
#define ITVLY 	(HEIGHT / NUM) // 石の縦の間隔 [m]
#define RANDRANGE (ITVLX / 2 * 0.2) // 石を置く位置を乱数で変える範囲 [m]


int main(void)
{
    int i, j;
    float x, y, z, rx, ry, rz;
    char color[][16] = {"Red", "Green", "Blue"};
	
    x = WIDTH / 2.0;
    y = 0;
    z = HEIGHT / 2.0;
    rx = WIDTH;
    ry = THICK;
    rz = HEIGHT;

    printf("<collision name=\"bldwallV1collision\">\n");
    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
    printf("</geometry>\n</collision>\n");

    printf("<visual name=\"bldwallV1visual\">\n");
    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
    printf("<name>Gazebo/Granite</name> </script> </material>\n</visual>\n\n");

    for (i = 0; i < NUM; i++) {
	for (j = 0; j < NUM; j++) {
 	    x = ITVLX * j + ITVLX/ 2 + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    y = THICK / 2 + LENGTH / 2;
	    z = ITVLY *	i + ITVLX/ 2 + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    rx = 0.025;
	    ry = LENGTH;
	    rz = 0.025;

	    printf("<collision name=\"bldwall%d%dcollision\">\n", i, j);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
	    printf("</geometry>\n</collision>\n");

	    printf("<visual name=\"bldwall%d%dvisual\">\n", i, j);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
	    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
            printf("<name>Gazebo/Grey</name> </script> </material>\n</visual>\n\n");

	    y = THICK / 2 + LENGTH + LENGTH2 / 2;
	    rx = LENGTH2;
	    ry = LENGTH2;
	    rz = LENGTH2;

	    printf("<collision name=\"bldwallb%d%dcollision\">\n", i, j);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
	    printf("</geometry>\n</collision>\n");

	    printf("<visual name=\"bldwallb%d%dvisual\">\n", i, j);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
	    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
            printf("<name>Gazebo/%s</name> </script> </material>\n</visual>\n\n", color[random()%3]);
	}
    }
    
    exit(EXIT_SUCCESS);
}
