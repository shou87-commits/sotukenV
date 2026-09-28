// gcc -ggdb -Wall -O -o ppmmark ppmmark.c

// pinhole カメラの設定でカメラから１ｍ前方で１ｍ横や縦にずれると
// 画像中では384ピクセルずれる

#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>

typedef unsigned char Uchar;

int token(char  str[], int num[]);

int	main(int argc, char *argv[])
{
    int         format = 0, x = 0, y = 0, num[10];
    char	str[BUFSIZ], *p;
    FILE	*fp1, *fp2;

    if (argc < 5) {
	fprintf(stderr, "Usage: %s infile outfile x y [x y]\n", argv[0]);
	exit(0);
    }
    if ((FILE *)NULL == (fp1 = fopen(argv[1], "r"))) {
	fprintf(stderr, "Error: [%s] can not open\n", argv[1]);
	exit(1);
    }

    if ((char *)NULL != fgets(str, BUFSIZ, fp1)) {
	if ('P' == str[0] && '6' == str[1]) {
	    format = 6;
	    printf("P%d ", format);
	}
	else exit(1);
    }
    fgets(str, BUFSIZ, fp1); // comment
    if ((char *)NULL != fgets(str, BUFSIZ, fp1)) {
	if (2 == token(str, num)) {
	    x = num[0];
	    y = num[1];
	    printf("X:%d Y:%d\n", x, y);
	}
	else exit(1);
    }
    fgets(str, BUFSIZ, fp1); // depth
    p = (char*)malloc(sizeof(Uchar)*x*y*3);
    fread(p, sizeof(Uchar), x*y*3, fp1);
    fclose(fp1);

    for (int i = 3; i < argc; i += 2) {
	int tx = atoi(argv[i]);
	int ty = atoi(argv[i+1]);
	p[(ty * x + tx) * 3 + 0] = 255;
	p[(ty * x + tx) * 3 + 1] = 255;
	p[(ty * x + tx) * 3 + 2] = 255;
    }
    
    if ((FILE *)NULL == (fp2 = fopen(argv[2], "w"))) {
	fprintf(stderr, "Error: [%s] can not open\n", argv[2]);
	exit(1);
    }
    fprintf(fp2, "P6\n%d %d\n255\n", x, y);
    fwrite(p, sizeof(Uchar), x*y*3, fp2);
    fclose(fp2);

    exit(0);
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
