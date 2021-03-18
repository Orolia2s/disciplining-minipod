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

struct config {
	char *argz;
	size_t len;
	char *path;
};

int config_init(struct config *config, const char *path);
const char *config_get(const struct config *config, const char *key);
void config_cleanup(struct config *config);

#endif /* CONFIG_H_ */
