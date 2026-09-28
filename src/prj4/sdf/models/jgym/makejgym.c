// $ gcc -o makejgym makejgym.c
// $ ./makejgym > jgym.sdf
// $ cat sdfhead.sdf jgym.sdf sdftail.sdf > model.sdf
// ~/.gazebo/models/jgym/ に model.sdf を置く、またはリンクを張る
// abc.world 内から jgym を読み込むように登録する
// $ gazebo abc.world

#include <stdio.h>
#include <stdlib.h>

#define WIDTH	5.0	// ジャングルジムの幅 [m]
#define DEPTH 	4.0	// ジャングルジムの奥行き [m]
#define HEIGHT 	3.0	// ジャングルジムの高さ [m]
#define NUM 	7	// 上で指定した高さの範囲に何段置くか、個数
#define ITVLY 	(HEIGHT / NUM) // 縦方向の横棒の間隔 [m]
#define RANDRANGE (ITVLY / 2 * 0.75) // 横棒を置く位置を乱数で変える範囲 [m]

void printbox(char key[], int i, float x, float y, float z, float r, float p, float yo,
              float rx, float ry, float rz);

int main(void)
{
    float x, y, z, rx, ry, rz;

    z = HEIGHT / 2.0;
    rx = 0.05;
    ry = 0.05;
    rz = HEIGHT;

    for (int i = 0; i <= WIDTH; i++) {
	for (int j = 0; j <= DEPTH; j++) {
	    x = i;
	    y = j;
	    printbox("XY", i*10+j, x, y, z, 0.0, 0.0, 0.0, rx, ry, rz);
	}
    }

    for (int i = 1; i < NUM; i++) {
	    z = ITVLY *	i + (random() % 1000 - 500) / 500.0 * RANDRANGE;
	    rz = 0.05;

 	    x = WIDTH / 2;
	    rx = WIDTH;
	    ry = 0.05;
	    for (int j = 0; j <= DEPTH; j++) {
		y = j;
		printbox("YZ", i*10+j, x, y, z, 0.0, 0.0, 0.0, rx, ry, rz);
	    }

	    y = DEPTH / 2;
	    rx = 0.05;
	    ry = DEPTH;
	    for	(int j = 0; j <= WIDTH; j++) {
                x = j;
                printbox("XZ", i*10+j, x, y, z, 0.0, 0.0, 0.0, rx, ry, rz);
            }
    }
    {
	    int i = 9;
	    z = HEIGHT;
	    rz = 0.05;

 	    x = WIDTH / 2;
	    rx = WIDTH;
	    ry = 0.05;
	    for (int j = 0; j <= DEPTH; j++) {
		y = j;
		printbox("YZ", i*10+j, x, y, z, 0.0, 0.0, 0.0, rx, ry, rz);
	    }

	    y = DEPTH / 2;
	    rx = 0.05;
	    ry = DEPTH;
	    for	(int j = 0; j <= WIDTH; j++) {
                x = j;
                printbox("XZ", i*10+j, x, y, z, 0.0, 0.0, 0.0, rx, ry, rz);
            }
    }
    exit(EXIT_SUCCESS);
}

void printbox(char key[], int i, float x, float y, float z, float r, float p, float yo,
	      float rx, float ry, float rz)
{
    printf("<collision name=\"jgym%s%dcollision\">\n", key, i);
    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, r, p, yo);
    printf("<geometry> <box> <size>%f %f %f</size> </box> ", rx, ry, rz);
    printf("</geometry>\n</collision>\n");

    printf("<visual name=\"jgym%s%dvisual\">\n", key, i);
    printf("<pose>%f %f %f %f %f %f</pose>\n", x, y, z, r, p, yo);
    printf("<geometry> <box> <size>%f %f %f</size> </box> </geometry>\n", rx, ry, rz);
    printf("<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
    printf("<name>Gazebo/Grey</name> </script> </material>\n</visual>\n\n");
}
