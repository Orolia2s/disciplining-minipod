#include <stdlib.h>
#include <string.h>

#include "parameters.h"


int get_disciplining_parameters(struct disciplining_parameters *dsc_params, const char *path)
{
	int ret;
	FILE *fp;

	fp = fopen(path,"rb");
	if (fp == NULL) {
		log_error("Could not open disciplining parameters file %s", path);
		return -1;
	}
	ret = fread(dsc_params, sizeof(struct disciplining_parameters), 1, fp);
	if (ret != 1) {
		log_error("Error reading file %s", path);
		return -1;
	}
	fclose(fp);
	return 0;
}

int update_disciplining_parameters(struct disciplining_parameters *dsc_params, const char *path)
{
	FILE *fp = fopen(path,"wb");
	if (fp == NULL) {
		log_error("Could not open discipling parameters file %s", path);
		return -1;
	}
	fwrite(dsc_params, 1, sizeof(struct disciplining_parameters), fp);
	fclose(fp);
	return 0;
}
