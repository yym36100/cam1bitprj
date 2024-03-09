
#include "stdafx.h"
#include <stdio.h>

long getFileSize(FILE *file) {
	long size;
	long currentPosition = ftell(file);
	fseek(file, 0, SEEK_END);
	size = ftell(file);
	fseek(file, currentPosition, SEEK_SET);

	return size;
}

int searchPattern(const unsigned char *array,int asize,int startindex, const char *pattern, int patsize) {
	int arrayLength = asize;
	int patternLength = patsize;

	for (int i = startindex; i <= arrayLength - patternLength; i++) {
		int j;

		// Check if the pattern matches starting at index i
		for (j = 0; j < patternLength; j++) {
			if (array[i + j] != pattern[j])
				break;
		}

		// If pattern matched completely, return the index
		if (j == patternLength)
			return i;
	}

	// Pattern not found
	return -1;
}

void RGB565_to_RGB(unsigned short rgb565, unsigned char *red, unsigned char *green, unsigned char *blue) {
	// Extract red, green, and blue components
	*red = (rgb565 >> 11) & 0x1F;      // 5 bits for red
	*green = (rgb565 >> 5) & 0x3F;     // 6 bits for green
	*blue = rgb565 & 0x1F;             // 5 bits for blue

	*red*=8;
	*green*=4;
	*blue*=8;
}

void save_ppm(int n, unsigned char *p){
	static int once=1;
	if(once){
		FILE *f;
		char name[20];
		sprintf(name,"file_%02d.ppm",n);
		f = fopen(name,"wb");
		if(!f)return;
		fprintf(f,"P6\n");
		fprintf(f,"160 30\n");
		fprintf(f,"255\n");

		unsigned short c;
		unsigned char r,g,b;
		for( int i=0;i<9600;i+=2){
			c = (p[i+0]<<8) + p[i+1];
			RGB565_to_RGB(c,&r,&g,&b);

			if(i==0){r=255;g=0;b=0;}
			if(i==2){g=255;r=0;b=0;}
			if(i==4){b=255;r=0;g=0;}

			fwrite(&r,1,1,f);
			fwrite(&g,1,1,f);
			fwrite(&b,1,1,f);
		}

		fclose(f);
		//once = 0;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	FILE *file;
	long size;

	file = fopen(argv[1], "rb"); // Open in binary mode

	if (file != NULL) {
		size = getFileSize(file);
		printf("File size: %ld bytes\n", size);	
	} else {
		printf("Failed to open file.\n");
		return -1;
	}

	unsigned char *buff = new unsigned char[size];

	fread(buff,1,size,file);
	fclose(file);

	int res = -6;
	int prev = 0;
	int cnt=0;
	while(res!=-1){
		res+=6;
		res = searchPattern(buff,size-9606,res,"Start ",6);
		printf("%d dist=%d\n",res, res-prev);	
		prev=res;

		if(res!=-1) save_ppm(cnt++,buff+res+6);
	}

	delete[] buff;

	return 0;
}

