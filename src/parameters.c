#include <errno.h>
#include <math.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include <oscillator-disciplining/oscillator-disciplining.h>

#include "parameters.h"
#include "config.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_A) (sizeof(_A) / sizeof((_A)[0]))
#endif


static bool debug = false;

typedef int (*parser_fn)(const char *value, struct parameters *p,
		const struct config_key *key);

static int double_parser(const char *value, struct parameters *p,
		const struct config_key *key)
{
	double value_double;
	double *location;
	char *endptr;

	errno = 0;
	value_double = strtold(value, &endptr);
	if (value_double == HUGE_VAL ||
			(value_double == 0 && errno == ERANGE))
		return -ERANGE;
	location = (typeof(location))((uintptr_t)p + key->offset);
	*location = value_double;

	if (debug)
		fprintf(stderr, "%s is %ff\n", key->name, value_double);

	return 0;
}

static int bool_parser(const char *value, struct parameters *p,
		const struct config_key *key)
{
	bool value_bool;
	bool *location;

	if (strcmp(value, "true") == 0)
		value_bool = true;
	else if (strcmp(value, "false") == 0)
		value_bool = false;
	else
		return -EINVAL;
	location = (typeof(location))((uintptr_t)p + key->offset);
	*location = value_bool;

	if (debug)
		fprintf(stderr, "%s is %s\n", key->name,
				value_bool ? "true" : "false");

	return 0;
}

static int int_parser(const char *value, struct parameters *p,
		const struct config_key *key)
{
	long value_int;
	int *location;
	char *endptr;

	errno = 0;
	value_int = strtol(value, &endptr, 0);
	if ((value_int == LONG_MIN || value_int == LONG_MAX) &&
			errno == ERANGE)
		return -ERANGE;
	location = (typeof(location))((uintptr_t)p + key->offset);
	*location = value_int;

	if (debug)
		fprintf(stderr, "%s is %ld\n", key->name, value_int);

	return 0;
}

static int char_parser(const char *value, struct parameters *p,
	const struct config_key *key)
{
	char *location;
	char value_char = value[0];
	location = (typeof(location)) ((uintptr_t)p + key->offset);
	*location = value_char;

	if (debug)
		fprintf(stderr, "%s is %c\n", key->name, value_char);

	return 0;
}

static const parser_fn parsers[] = {
		[VALUE_TYPE_DOUBLE] = double_parser,
		[VALUE_TYPE_BOOL] = bool_parser,
		[VALUE_TYPE_INT] = int_parser,
		[VALUE_TYPE_CHAR] = char_parser,
};

/* override the parameters which are specified in the configuration file */
int fill_parameters(struct parameters *p, const char *path,
		char err_msg[OD_ERR_MSG_LEN])
{
	unsigned int i;
	const struct config_key *key;
	const char *value;
	parser_fn parser;
	int ret;
	struct config __attribute__((cleanup(config_cleanup))) config;

	/* must be first */
	ret = config_init(&config, path);
	if (ret < 0) {
		snprintf(err_msg, OD_ERR_MSG_LEN, "config_init failed");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(config_keys); i++) {
		key = config_keys + i;
		parser = parsers[key->type];
		value = config_get(&config, key->name);
		if (value == NULL)
			continue;
		ret = parser(value, p, key);
		if (ret != 0) {
			snprintf(err_msg, OD_ERR_MSG_LEN, "parsing %s failed",
					key->name);
			return ret;
		}
	}

	return 0;
}
