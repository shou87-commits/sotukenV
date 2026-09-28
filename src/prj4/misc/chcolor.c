// gcc -ggdb -Wall -O -o chcolor chcolor.c

// ロボットのリンクの色を指定した色、またはランダムに変更する

#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>

#define YES 1
#define NO  0

#define RANDOM 0
#define SPECIFIC 1

#define NUMCOLOR 32 // or 44

typedef unsigned char Uchar;

int IncludeWord(char str[], char word[], int *pos);
int token(char  str[], int num[]);

// 普通の色
/*
char color[44][48] = {
"Gazebo/Grey", 
"Gazebo/Gray", 
"Gazebo/DarkGrey", 
"Gazebo/DarkGray", 
"Gazebo/White", 
"Gazebo/FlatBlack", 
"Gazebo/Black", 
"Gazebo/Red", 
"Gazebo/RedBright", 
"Gazebo/Green", 
"Gazebo/Blue", 
"Gazebo/SkyBlue", 
"Gazebo/Yellow", 
"Gazebo/ZincYellow", 
"Gazebo/DarkYellow", 
"Gazebo/Purple", 
"Gazebo/Turquoise", 
"Gazebo/Orange", 
"Gazebo/Indigo", 
"Gazebo/WhiteGlow", 
"Gazebo/RedGlow", 
"Gazebo/GreenGlow", 
"Gazebo/BlueGlow", 
"Gazebo/YellowGlow", 
"Gazebo/PurpleGlow", 
"Gazebo/TurquoiseGlow", 
"Gazebo/TurquoiseGlowOutline", 
"Gazebo/RedTransparentOverlay", 
"Gazebo/BlueTransparentOverlay", 
"Gazebo/GreenTransparentOverlay", 
"Gazebo/OrangeTransparentOverlay", 
"Gazebo/DarkOrangeTransparentOverlay",
"Gazebo/RedTransparent", 
"Gazebo/GreenTransparent", 
"Gazebo/BlueTransparent", 
"Gazebo/DarkMagentaTransparent", 
"Gazebo/GreyTransparent", 
"Gazebo/BlackTransparent", 
"Gazebo/YellowTransparent", 
"Gazebo/LightOn", 
"Gazebo/LightOff", 
"Gazebo/LightBlueLaser", 
"Gazebo/BlueLaser", 
"Gazebo/OrangeTransparent"
};
*/

// 素材のテクスチャ
char color[32][48] = {
"Gazebo/JointAnchor", 
"Gazebo/CoM", 
"Gazebo/CeilingTiled", 
"Gazebo/PaintedWall", 
"Gazebo/PioneerBody", 
"Gazebo/Pioneer2Body", 
"Gazebo/Gold", 
"Gazebo/GreyGradientSky", 
"Gazebo/CloudySky", 
"Gazebo/Wood", 
"Gazebo/WoodFloor", 
"Gazebo/WoodPallet", 
"Gazebo/Bricks", 
"Gazebo/Road", 
"Gazebo/Residential", 
"Gazebo/Tertiary", 
"Gazebo/Pedestrian", 
"Gazebo/Footway", 
"Gazebo/Motorway", 
"Gazebo/Lanes_6", 
"Gazebo/Trunk", 
"Gazebo/Lanes_4", 
"Gazebo/Primary", 
"Gazebo/Lanes_2", 
"Gazebo/Secondary", 
"Gazebo/Lane_1", 
"Gazebo/Steps", 
"Gazebo/BuildingFrame", 
"Gazebo/Runway", 
"Gazebo/Grass", 
"Gazebo/Editor", 
"Gazebo/EditorPlane"
};

int	main(int argc, char *argv[])
{
    int         pos, mode = RANDOM, randomseed = 0;
    char	str[BUFSIZ];
    FILE	*fp1, *fp2;

    if (argc < 3) {
	fprintf(stderr, "Usage: %s infile outfile [color]\n", argv[0]);
	exit(0);
    }
    if (argc == 3) {mode = RANDOM; srandom(randomseed);}
    if (argc == 4)  mode = SPECIFIC;

    if ((FILE *)NULL == (fp1 = fopen(argv[1], "r"))) {
	fprintf(stderr, "Error: [%s] can not open\n", argv[1]);
	exit(1);
    }
    if ((FILE *)NULL == (fp2 = fopen(argv[2], "w"))) {
	fprintf(stderr, "Error: [%s] can not open\n", argv[2]);
	exit(1);
    }

    while ((char *)NULL != fgets(str, BUFSIZ, fp1)) {
	if (YES == IncludeWord(str, "material>", &pos)) {
	    str[pos] = (char)0;
	    fprintf(fp2, "%s", str);
	    if (RANDOM == mode) {
		fprintf(fp2, "%s", color[random() % NUMCOLOR]);
	    }
	    else if (SPECIFIC == mode) {
		fprintf(fp2, "Gazebo/%s", argv[3]);
	    }
	    fprintf(fp2, "</material> </gazebo>\n");
	}
	else fprintf(fp2, "%s", str);
    }

    fclose(fp1);
    fclose(fp2);

    exit(0);
}

int IncludeWord(char str[], char word[], int *pos)
{
    int i;

    for (i = 0; i < strlen(str); i++) {
	if (0 == strncmp(&str[i], word, strlen(word))) {
	    *pos = i + strlen(word);
	    return(YES);
	}
    }
    return(NO);
}

int token(char	str[], int num[])
{
    char	str2[BUFSIZ], *ptr, *ptr2;
    int		i, j, n = 0;

    ptr = str;

    while (1) {
	while (*ptr == ' ') ptr++;
	ptr2 = ptr;
	j = 0;
	while ((*ptr != ' ') && (*ptr != (char)0)) {
	    ptr++;
	    j++;
	}
	strncpy(str2, ptr2, j);
	str2[j] = (char)0;
	for (i = 0; i < j; i++) {
	    switch (str2[i]) {
  		case '0': case '1':
  		case '2': case '3':
  		case '4': case '5':
  		case '6': case '7':
  		case '8': case '9':
  			break;
  		default :
  			num[n] = atoi(str2);
  			return(n+1);
  			break;
	    }
	}
	num[n] = atoi(str2);
	n++;
    }
}
