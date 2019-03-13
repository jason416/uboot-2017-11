#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[])
{
	int memsize;
	char *buf;
	FILE *fp;
	char *filename;
	
	if (argc != 3) {
		printf("usage error...\n");
		return -1;
	}
	
	memsize = atoi(argv[2]);
	
	buf = (char *)malloc(memsize);
	if (buf == NULL) {
		printf("malloc error...\n");
		return -1;
	}
	
	memset(buf, 0, memsize);
	
	filename = argv[1];
	fp = fopen(filename, "wb");
	if (fp == NULL) {
		printf("fopen error...\n");
		free(buf);
		return -1;
	}
	
	if ((fwrite(buf, 1, memsize, fp)) != memsize) {
		printf("fwrite error...\n");
		free(buf);
		fclose(fp);
		return -1;
	}
	free(buf);
	fclose(fp);
	return 0;
}
