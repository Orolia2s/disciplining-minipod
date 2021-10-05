/*
 * liboscillator-disciplining: Disciplining Algorithm for Orolia's mRO50.
 * Copyright (C) 2021  Spectracom SAS

 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <errno.h>
#include <stdlib.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <argz.h>
#include <envz.h>

#include "config.h"
#include "log.h"
#include "utils.h"


static int read_file(const char *path, char **argz, size_t *argz_len)
{

	char __attribute__((cleanup(string_cleanup)))*string = NULL;
	FILE __attribute__((cleanup(file_cleanup)))*f = NULL;
	long size;
	int ret;
	size_t sret;
	char *entry = NULL;

	f = fopen(path, "rbe");
	if (f == NULL)
		return -errno;

	/* compute the size of the file */
	ret = fseek(f, 0, SEEK_END);
	if (ret == -1)
		return -errno;
	size = ftell(f);
	if (size == -1)
		return -errno;
	ret = fseek(f, 0, SEEK_SET);
	if (ret == -1)
		return -errno;

	/* read all */
	string = calloc(size, 1);
	if (string == NULL)
		return -errno;

	sret = fread(string, 1, size, f);
	if (sret < (size_t)size)
		return feof(f) ? -EIO : ret;

	ret = -argz_create_sep(string, '\n', argz, argz_len);
	if (ret)
		return ret;

	while ((entry = argz_next(*argz, *argz_len, entry))) {
		if (entry[0] == '#') {
			argz_delete(argz, argz_len, entry);
			entry = NULL;
		}
	}

	return 0;
}

int config_init(struct config *config, const char *path)
{
	int ret;
	const char *defconfig_path = NULL;
	const char *defconfig_key = NULL;

	defconfig_key = config->defconfig_key;

	memset(config, 0, sizeof(*config));

	config->path = strdup(path);
	if (config->path == NULL)
		return -errno;

	ret = read_file(path, &config->argz, &config->len);
	if (ret)
		return ret;

	if (defconfig_key != NULL) {
		defconfig_path = config_get(config, defconfig_key);
		config->defconfig_key = defconfig_key;
	}

	if (defconfig_path == NULL)
		return 0;

	ret = read_file(defconfig_path, &config->argz_defconfig, &config->len_defconfig);
	if (ret)
		return ret;

	ret = envz_merge(&config->argz_defconfig, &config->len_defconfig,
			 config->argz, config->len, 1);
	if (ret)
		return ret;

	ret = envz_merge(&config->argz, &config->len,
			config->argz_defconfig, config->len_defconfig, 0);

	// remove any null entry
	envz_strip (&config->argz, &config->len);

	return ret;
}

const char *config_get(const struct config *config, const char *key)
{
	return envz_get(config->argz, config->len, key);
}

const char *config_get_default(const struct config *config, const char *key,
		const char *default_value)
{
	const char *value;

	value = config_get(config, key);

	return value == NULL ? default_value : value;
}

bool config_get_bool_default(const struct config *config, const char *key,
		bool default_value)
{
	const char *value;

	value = config_get(config, key);
	if (value == NULL)
		return default_value;

	if (strcmp(value, "true") == 0)
		return true;
	if (strcmp(value, "false") == 0)
		return false;

	return default_value;
}

int config_set(struct config *config, const char *key, const char *value)
{
	return -envz_add(&config->argz, &config->len, key, value);
}

/* Get a number between 0 and (2**31)-1 */
long config_get_unsigned_number(const struct config *config, const char *key)
{
	const char *str_value;
	char *endptr;
	unsigned long value;

	str_value = config_get(config, key);
	if (str_value == NULL)
		return errno != 0 ? -errno : -ESRCH;

	value = strtoul(str_value, &endptr, 0);
	if (*str_value == '\0' || *endptr != '\0')
		return -EINVAL;

	if (value > LONG_MAX)
		return -ERANGE;

	return value;
}

int config_get_uint8_t(const struct config *config, const char *key)
{

	long value;

	value = config_get_unsigned_number(config, key);
	if (value > UINT8_MAX)
		return -ERANGE;

	return value;
}

void config_dump(const struct config *config, char *buf,
		size_t buf_len)
{
	size_t n;
	size_t slen;
	char *argz = malloc(config->len);

	memcpy(argz, config->argz, config->len);
	argz_stringify(argz, config->len, '\n');
	slen = strlen(argz) + 1;
	n = buf_len < slen ? buf_len : slen;
	memcpy(buf, argz, n);
	free(argz);
}

void config_cleanup(struct config *config)
{
	if (config->argz != NULL)
		free(config->argz);
	if (config->path != NULL)
		free(config->path);
	memset(config, 0, sizeof(*config));
}


