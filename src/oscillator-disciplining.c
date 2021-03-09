#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stddef.h>

#include <oscillator-disciplining/oscillator-disciplining.h>

#include "config.h"
#include "parameters.h"


struct od {
    uint32_t data;
    struct parameters params;
    clockid_t clockid;
};

struct od *od_new(clockid_t clockid)
{
	struct od *od;

	od = calloc(1, sizeof(*od));
	if (od == NULL)
		return NULL;
	od->clockid = clockid;

    printf("Od_new called \n");
	return od;
}

struct od *od_new_from_config(const char *path, char err_msg[OD_ERR_MSG_LEN])
{
	struct od *od;
	int ret;

	if (path == NULL || *path == '\0' || err_msg == NULL) {
		errno = EINVAL;
		return NULL;
	}

	od = calloc(1, sizeof(*od));
	if (od == NULL)
		return NULL;

	ret = fill_parameters(&od->params, path, err_msg);

	print_parameters(&od->params);

	od->clockid = CLOCK_REALTIME;
    printf("ret is %d, Path is %s\n", ret, path);
	printf("Od_new_from_config called \n");

	return od;
}

uint32_t od_get_dac_min(const struct od *od)
{
	if (od == NULL)
		return UINT32_MAX;

    printf("Od_get_dac_min called \n");
    return 0;
}

uint32_t od_get_dac_max(const struct od *od)
{
	if (od == NULL)
		return 0;

    printf("Od_get_dac_max called \n");
	return 0;
}

int od_process(struct od *od, const struct od_input *input,
		struct od_output *output)
{
    printf("Od_process called \n");
	return 0;
}

void od_destroy(struct od **od)
{
	if (od == NULL || *od == NULL)
		return;

	free(*od);
	*od = NULL;
    printf("Od_destroy called \n");
}
