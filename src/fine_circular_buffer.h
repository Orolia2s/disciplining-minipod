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

/**
 * @file fine_circular_buffer.h
 * @date 2022-05-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MINIPOD_FINE_CIRCULAR_BUFFER_H
#define MINIPOD_FINE_CIRCULAR_BUFFER_H

#include "stdint.h"

/**
 * @brief Number of temperature steps, starting at 20°C
 * Each step is a 0.25° range
 */
#define TEMPERATURE_STEPS 160
/**
 * @brief Number of steps by degrees (1/ 0.25° range)
 */
#define STEPS_BY_DEGREE 4

#define MIN_TEMPERATURE 20.0
#define MAX_TEMPERATURE 60.0

#define CIRCULAR_BUFFER_SIZE 25

#define MIN_VALUES_FOR_MEAN 10

union fine_value {
    float fine_estimated_equilibrium_ES;
    uint16_t fine_applied;
};

struct fine_circular_buffer {
    union fine_value buffer[CIRCULAR_BUFFER_SIZE];
    char fine_type; // 'A' for fine applied, 'S' for smoothed
    float mean_fine;
    int read_index;
    int write_index;
    int buffer_length;
};

void print_tuples(struct fine_circular_buffer *circular_buffer);
int write_fine(struct fine_circular_buffer *circular_buffer, union fine_value fine);
int add_fine_from_temperature(struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS], union fine_value fine, double temp);
int write_buffers_in_file(struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS], const char* output_file);
int compute_mean_value(struct fine_circular_buffer *fine_buffer);
int get_index_of_temperature(float temperature);
float get_delta_fine_from_temperature_table(
    struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS],
    float input_temperature,
    float holdover_mRO_EP_temperature,
    float estimated_equilibrium_ES
);

#endif /* MINIPOD_FINE_CIRCULAR_BUFFER_H */
