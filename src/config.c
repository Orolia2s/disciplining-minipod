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
#include <stdio.h>
#include <inttypes.h>

#include <argz.h>
#include <envz.h>

#include "config.h"

static void string_cleanup(char **s)
{
	if (s == NULL || *s == NULL)
		return;

	free(*s);
	*s = NULL;
}

static void file_cleanup(FILE **f)
{
	if (f == NULL || *f == NULL)
		return;

	fclose(*f);
	*f = NULL;
}

int config_init(struct config *config, const char *path)
{
	int ret;
	char __attribute__((cleanup(string_cleanup)))*string = NULL;
	FILE __attribute__((cleanup(file_cleanup)))*f = NULL;
	long size;
	size_t sret;

	memset(config, 0, sizeof(*config));

	config->path = strdup(path);
	if (config->path == NULL)
		return -errno;

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

	fclose(f);
	return -argz_create_sep(string, '\n', &config->argz, &config->len);
}

const char *config_get(const struct config *config, const char *key)
{
	return envz_get(config->argz, config->len, key);
}

void config_cleanup(struct config *config)
{
	if (config->argz != NULL)
		free(config->argz);
	if (config->path != NULL)
		free(config->path);
	memset(config, 0, sizeof(*config));
}
