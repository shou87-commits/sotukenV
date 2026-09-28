// $ gcc -o makeladder makeladder.c
// $ ./makeladder > ladder.sdf
// $ cat sdfhead.sdf ladder.sdf sdftail.sdf > model.sdf
// ~/.gazebo/models/ladder/ に model.sdf を置く、またはリンクを張る
// abc.world 内から ladder を読み込むように登録する
// $ gazebo abc.world

#include <stdio.h>
#include <stdlib.h>

#define WIDTH	1.0	// 梯子の幅 [m]
#define HEIGHT 	8.0	// 梯子の高さ [m]
#define NUM 	15	// 上で指定した高さの範囲に何段置くか、個数
#define ITVLY 	(HEIGHT / NUM) // 縦方向の横棒の間隔 [m]
#define RANDRANGE (ITVLY / 2 * 0.75) // 横棒を置く位置を乱数で変える範囲 [m]

int main(void)
{
    int i;
    float x, y, z, rx, ry, rz;

    x = 0;
    y = 0;
    z = HEIGHT / 2.0;
    rx = 0.05;
    ry = 0.05;
    rz = HEIGHT;

    printf("<collision name=\"ladderV1collision\">\n");
    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
    printf("</geometry>\n</collision>\n");

    printf("<visual name=\"ladderV1visual\">\n");
    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
    printf("<name>Gazebo/Grey</name> </script> </material>\n</visual>\n\n");

    x = WIDTH;

    printf("<collision name=\"ladderV2collision\">\n");
    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
    printf("</geometry>\n</collision>\n");

    printf("<visual name=\"ladderV2visual\">\n");
    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
    printf("<name>Gazebo/Grey</name> </script> </material>\n</visual>\n\n");

    for (i = 1; i < NUM; i++) {
 	    x = WIDTH / 2;
	    y = 0;
	    z = ITVLY *	i + (random() % 1000 - 500) / 500.0 * RANDRANGE;
//	    z = ITVLY *	i + ITVLY * 0.5 - (ITVLY/2) + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    rx = WIDTH;
	    ry = 0.05;
	    rz = 0.05;

	    printf("<collision name=\"ladderH%dcollision\">\n", i);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
	    printf("</geometry>\n</collision>\n");

	    printf("<visual name=\"ladderH%dvisual\">\n", i);
	    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, 0.0, 0.0, 0.0);
	    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
	    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
            printf("<name>Gazebo/Grey</name> </script> </material>\n</visual>\n\n");
    }

    exit(EXIT_SUCCESS);
}
