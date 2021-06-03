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

#ifndef CONFIG_H_
#define CONFIG_H_
#include <stddef.h>
#include <stdbool.h>

struct config {
	char *argz;
	size_t len;
	char *argz_defconfig;
	size_t len_defconfig;
	const char *defconfig_key;
	char *path;
};

int config_init(struct config *config, const char *path);
const char *config_get(const struct config *config, const char *key);
const char *config_get_default(const struct config *config, const char *key,
		const char *default_value);
bool config_get_bool_default(const struct config *config, const char *key,
		bool default_value);
int config_set(struct config *config, const char *key, const char *value);

/* returns a value in [0, LONG_MAX] on success, -errno on error */
long config_get_unsigned_number(const struct config *config, const char *key);

/* returns a value in [0, UINT8_MAX] on success, -errno on error */
int config_get_uint8_t(const struct config *config, const char *key);
void config_cleanup(struct config *config);

void config_dump(const struct config *config, char *buf, size_t buf_len);
#endif /* CONFIG_H_ */
