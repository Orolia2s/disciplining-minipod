#ifndef LOG_H_
#define LOG_H_
/**
 * @file log.h
 * @brief Provide leveled log implementation.
 *
 * Defines log levels that can be used anywhere on the project
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <errno.h>

/** Error log level prefix */
#define ERR "<3>"
/** Warning log level prefix */
#define WARN "<4>"
/** Info log level prefix */
#define INFO "<6>"
/** Debug log level prefix */
#define DEBUG "<7>"

extern bool log_debug_enabled;

/* all those logging functions preserve the value of errno */
/** Error log function */
#define err(...) log(ERR __VA_ARGS__)
/** Warning log function */
#define warn(...) log(WARN __VA_ARGS__)
/** Info log function */
#define info(...) log(INFO __VA_ARGS__)
/** Debug log function */
#define debug(...) do { \
	if (log_debug_enabled) \
		log(DEBUG __VA_ARGS__); \
} while (0)

#define perr(f, e) err("%s: %s\n", f, strerror(abs(e)))

/** log function */
#define log(...) do { \
	int __old_errno = errno; \
	fprintf(stderr, __VA_ARGS__); \
	errno = __old_errno; \
} while (0)

/** Activate debug log level */
void log_enable_debug(bool enable_debug);

#endif /* LOG_H_ */
