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

/**
 * @file config.h
 * @brief Helper to get config parameters from file.
 *
 * Provides functions to analyze a given file and parse key values.
 * These values are then parsed in parameters.h to get the parameters
 * of the disciplining algorithm.
 *
 */

/**
 * @brief configuration structure used to parse config file
 */
struct config {
	char *argz;
	size_t len;
	char *path;
};

/**
 * @brief Init configuration structure from config file's path
 */
int config_init(struct config *config, const char *path);
/**
 * @brief get config value of a key inside config structure
 */
const char *config_get(const struct config *config, const char *key);
/**
 * @brief clean up configuration
 */
void config_cleanup(struct config *config);

#endif /* CONFIG_H_ */
