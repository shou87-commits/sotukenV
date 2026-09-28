// $ gcc -o maketree maketree.c
// $ ./maketree > tree.launch
// tree.launch を prg4_gz.launch.ena の後半に置く

#include <stdio.h>
#include <stdlib.h>

#define WIDTH	1.5	// 瓦礫の最大幅 [m]
#define HEIGHT 	0.8	// 高さ [m]
#define DEPTH 	0.5	// 厚さ [m]
#define RADIUS 	0.25	// 半径 [m]

int main(int argc, char *argv[])
{
    int i;
    int seed = 1; // randomseed
    int numfiles = 1; // num of debris

    if (1 == argc) fprintf(stderr, "Usage: %s [files randomseed]\n", argv[0]);
    if (2 <= argc) numfiles = atoi(argv[1]);
    if (3 <= argc) seed = atoi(argv[2]);
    
    srandom(seed);

    for (i = 0; i < numfiles; i++) {
        const float SX = 8; // 物体をばらまく原点
	const float SY = -20;
	const float SZ = 20;
        const float FX = 18; // ばらまく範囲
	const float FY =  7;
	const float FZ = 10;
	float px = FX * (random() % 80 + 10) / 100.0 + SX; // 物体の初期位置
	float py = FY * (random() % 80 + 10) / 100.0 + SY; 
	float pz = FZ * (random() % 80 + 10) / 100.0 + SZ;

	float roll = 0, pitch = 1.5708, yaw = 0;
//	float roll  = (random() % 100) / 100.0; // 回転角度 radian
//	float pitch = (random() % 100) / 100.0;
//	float yaw   = (random() % 100) / 100.0;
	
	printf("    <arg name=\"tree%03d\" default=\"$(find prj4)/urdf/tree.urdf\"/>\n", i);
	printf("    <param name=\"tree_desc%03d\" textfile=\"$(arg tree%03d)\"/>\n", i, i);
	printf("    <node name=\"tree_spawner%03d\" pkg=\"gazebo_ros\" type=\"spawn_model\"\n", i);
	printf("          args=\"-param tree_desc%03d -urdf ", i);
	printf("-x %.1f -y %.1f -z %.1f -R %.1f -P %.1f -Y %.1f ", px, py, pz, roll, pitch, yaw);
	printf("-model tree%03d\"/>\n\n", i);
    }
    
    exit(EXIT_SUCCESS);
}
