// $ gcc -o makedebrissdf makedebrissdf.c
// $ ./makedebris 

// ~/.gazebo/models/debris/ に model.sdf を置く、またはリンクを張る
// 例として abc.world 内から debris を読み込むように登録、次行で確認
// $ gazebo abc.world

#include <stdio.h>
#include <stdlib.h>

#define WIDTH	1.5	// 瓦礫の最大幅 [m]
#define HEIGHT 	0.8	// 高さ [m]
#define DEPTH 	0.5	// 厚さ [m]
#define RADIUS 	0.25	// 半径 [m]

int main(int argc, char *argv[])
{
    int i;
    char color[][16] = {"Red", "Green", "Blue", "Indigo", "Yellow"};
    char color2[][16] = {"WoodFloor", "WoodPallet", "Wood", "Tree", "Granite"};
    FILE *fpsdf, *fplaunch;
    int seed = 1; // randomseed
    int numfiles = 1; // num of debris

    if (1 == argc) fprintf(stderr, "Usage: %s [files randomseed]\n", argv[0]);
    if (2 <= argc) numfiles = atoi(argv[1]);
    if (3 <= argc) seed = atoi(argv[2]);
    
    srandom(seed);

    fplaunch = fopen("debris.launch", "w");
    
    for (i = 0; i < numfiles; i++) {
	char fname[32];
	sprintf(fname, "debris%03d.sdf", i);
	fpsdf = fopen(fname, "w");

	fprintf(fpsdf, "<?xml version=\"1.0\" ?>\n<sdf version=\"1.5\">\n");
	fprintf(fpsdf, "<model name=\"debris%03d\">\n", i);
//	fprintf(fpsdf, "<static>false</static>\n\n");

	float x = WIDTH  * ((random() % 70) / 100.0 + 0.3); // 瓦礫の大きさ meter
	float y = HEIGHT * ((random() % 70) / 100.0 + 0.3); 
	float z = DEPTH  * ((random() % 70) / 100.0 + 0.3);
	char name[32]; sprintf(name, "%s", color[random()%5]);
	
	fprintf(fpsdf, "<link name=\"debris%03d\">\n", i);

	fprintf(fpsdf, "<visual name=\"debris%03dvisual\">\n", i);
	fprintf(fpsdf, "<pose>0 0 0 0 0 0</pose>\n");
	fprintf(fpsdf, "<geometry> <box> <size>%.2f %.2f %.2f</size> </box> </geometry>\n", x, y, z);
	fprintf(fpsdf, "<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
	fprintf(fpsdf, "<name>Gazebo/%s</name> </script> </material>\n</visual>\n\n", name);

	fprintf(fpsdf, "<collision name=\"debris%03dcollision\">\n", i);
	fprintf(fpsdf, "<pose>0 0 0 0 0 0</pose>\n");
	fprintf(fpsdf, "<geometry> <box> <size>%.2f %.2f %.2f</size> </box> ", x, y, z);
	fprintf(fpsdf, "</geometry>\n</collision>\n\n");

	float mass = 10.0; // 質量 kg
	// 次行は直方体の慣性モーメントの簡易版計算式、３辺の平均値で計算する	
	float inertia = ((x + y + z) / 3) * ((x + y + z) / 3) * 2 * mass / 12.0;
	
	fprintf(fpsdf, "<inertial> <pose>0 0 0 0 0 0</pose>\n");
	fprintf(fpsdf, "<mass>%.1f</mass>\n<inertia> <ixx>%.4f</ixx> <ixy>0</ixy> ",
	       mass, inertia);
	fprintf(fpsdf, "<ixz>0</ixz>\n<iyy>%.4f</iyy> <iyz>0</iyz> <izz>%.4f</izz> ",
	       inertia, inertia);
	fprintf(fpsdf, "</inertia>\n</inertial>\n");
	fprintf(fpsdf, "</link>\n\n");

	for (int j = 0; j < 3; j++) { // ここは縁の出っ張り、掴みやすくするため
	    float inta = 0.01;
	    fprintf(fpsdf, "<link name=\"debris%03d%d\">\n", i, j);
	    fprintf(fpsdf, "<visual name=\"debris%03d%dvisual\">\n", i, j);
	    fprintf(fpsdf, "<pose>%.2f 0 0 0 0 0</pose>\n", x*(j-1)/2);
	    fprintf(fpsdf, "<geometry> <box> <size>%.2f %.2f %.2f</size> </box> </geometry>\n", 0.05, y+0.1, z+0.1);
	    fprintf(fpsdf, "<material> <script> <uri>file://media/materials/scripts/gazebo.material</uri>\n");
	    fprintf(fpsdf, "<name>Gazebo/%s</name> </script> </material>\n</visual>\n\n", name);
	    fprintf(fpsdf, "<collision name=\"debris%03d%dcollision\">\n", i, j);
	    fprintf(fpsdf, "<pose>%.2f 0 0 0 0 0</pose>\n", x*(j-1)/2);
	    fprintf(fpsdf, "<geometry> <box> <size>%.2f %.2f %.2f</size> </box> ", 0.05, y+0.1, z+0.1);
	    fprintf(fpsdf, "</geometry>\n</collision>\n\n");
	    fprintf(fpsdf, "<inertial> <pose>0 0 0 0 0 0</pose>\n");
	    fprintf(fpsdf, "<mass>0.1</mass>\n<inertia> <ixx>%.3f</ixx> <ixy>0</ixy> ", inta);
	    fprintf(fpsdf, "<ixz>0</ixz>\n<iyy>%.3f</iyy> <iyz>0</iyz> <izz>%.3f</izz> ", inta, inta);
	    fprintf(fpsdf, "</inertia>\n</inertial>\n");
	    fprintf(fpsdf, "</link>\n\n");
	    fprintf(fpsdf, "<joint type=\"fixed\" name=\"debris%03d%djoint\">\n", i, j);
	    fprintf(fpsdf, "<pose>0 0 0 0 0 0</pose>\n");
	    fprintf(fpsdf, "<child>\"debris%03d%d\"</child>\n", i, j);
	    fprintf(fpsdf, "<parent>\"debris%03d\"</parent>\n</joint>\n\n", i);
	}

	fprintf(fpsdf, "</model>\n</sdf>\n");
	fclose(fpsdf);

        const float SX = 2.5; // フェンスの原点からの距離 meter
	const float SY = -1.5;
	const float SZ = 0.0;
        const float FX = 5.0; // フェンスの長さ
	const float FY = 3.0;
	const float FZ = 1.0;
	float px = FX * (random() % 80 + 10) / 100.0 + SX; // 瓦礫の初期位置
	float py = FY * (random() % 80 + 10) / 100.0 + SY; 
	float pz = FZ * (random() % 80 + 10) / 100.0 + SZ + (i * 0.1);

	float roll  = (random() % 100) / 100.0; // 回転角度 radian
	float pitch = (random() % 100) / 100.0;
	float yaw   = (random() % 100) / 100.0;
	
	fprintf(fplaunch, "    <arg name=\"debris%03d\" default=\"$(find prj4)/sdf/models/debris/debris%03d.sdf\"/>\n", i, i);
	fprintf(fplaunch, "    <param name=\"debris_desc%03d\" textfile=\"$(arg debris%03d)\"/>\n", i, i);
	fprintf(fplaunch, "    <node name=\"debris_spawner%03d\" pkg=\"gazebo_ros\" type=\"spawn_model\"\n", i);
	fprintf(fplaunch, "          args=\"-param debris_desc%03d -sdf ", i);
	fprintf(fplaunch, "-x %.1f -y %.1f -z %.1f -R %.1f -P %.1f -Y %.1f ", px, py, pz, roll, pitch, yaw);
	fprintf(fplaunch, "-model debris%03d\"/>\n\n", i);
    }

    fclose(fplaunch);
    
    exit(EXIT_SUCCESS);
}
