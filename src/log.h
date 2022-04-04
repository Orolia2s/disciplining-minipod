/**
 * Copyright (c) 2020 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See `log.c` for details.
 */

#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <time.h>

#include <oscillator-disciplining/oscillator-disciplining.h>

#define LOG_VERSION "0.1.0"

typedef struct {
  va_list ap;
  const char *fmt;
  const char *file;
  struct tm *time;
  void *udata;
  int line;
  int level;
} log_Event;

typedef void (*log_LogFn)(log_Event *ev);
typedef void (*log_LockFn)(bool lock, void *udata);

enum { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };

#define log_trace(...) log_log(LOG_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#define log_debug(...) log_log(LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define log_info(...)  log_log(LOG_INFO,  __FILE__, __LINE__, __VA_ARGS__)
#define log_warn(...)  log_log(LOG_WARN,  __FILE__, __LINE__, __VA_ARGS__)
#define log_error(...) log_log(LOG_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#define log_fatal(...) log_log(LOG_FATAL, __FILE__, __LINE__, __VA_ARGS__)

const char* log_level_string(int level);
void log_set_lock(log_LockFn fn, void *udata);
void log_set_level(int level);
void log_set_quiet(bool enable);
int log_add_callback(log_LogFn fn, void *udata, int level);
int log_add_fp(FILE *fp, int level);

void log_log(int level, const char *file, int line, const char *fmt, ...);

static inline void print_inputs(struct od_input inputs[7])
{
	log_info(
		"Inputs: [%f, %f, %f, %f, %f, %f, %f]",
		(float) inputs[0].phase_error.tv_nsec + (float) inputs[0].qErr / PS_IN_NS,
		(float) inputs[1].phase_error.tv_nsec + (float) inputs[1].qErr / PS_IN_NS,
		(float) inputs[2].phase_error.tv_nsec + (float) inputs[2].qErr / PS_IN_NS,
		(float) inputs[3].phase_error.tv_nsec + (float) inputs[3].qErr / PS_IN_NS,
		(float) inputs[4].phase_error.tv_nsec + (float) inputs[4].qErr / PS_IN_NS,
		(float) inputs[5].phase_error.tv_nsec + (float) inputs[5].qErr / PS_IN_NS,
		(float) inputs[6].phase_error.tv_nsec + (float) inputs[6].qErr / PS_IN_NS
	);
}


#endif
