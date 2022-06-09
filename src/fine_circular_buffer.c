#include "fine_circular_buffer.h"
#include "log.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#define TOLERANCE_PRECISION 0.00001
static inline int compare_float(float f1, float f2)
{
    return fabs(f1 -f2) <= TOLERANCE_PRECISION ? 1 : 0;
}

int write_fine(struct fine_circular_buffer *circular_buffer, union fine_value fine)
{
    if (!circular_buffer) {
        log_error("Circular buffer is NULL");
        return - EINVAL;
    }

    if (circular_buffer->fine_type == 'A') {
        /* Write tuple at write index */
        circular_buffer->buffer[circular_buffer->write_index].fine_applied = fine.fine_applied;
    } else if (circular_buffer->fine_type == 'S') {
        circular_buffer->buffer[circular_buffer->write_index].fine_estimated_equilibrium_ES = fine.fine_estimated_equilibrium_ES;
    } else {
        log_error("Buffer is not of type 'A' or 'S'");
        return -EINVAL;
    }

    /* Increment write index */
    circular_buffer->write_index++;
    if (circular_buffer->write_index == CIRCULAR_BUFFER_SIZE)
        circular_buffer->write_index = 0;
    /* Increment buffer length */
    if (circular_buffer->buffer_length < CIRCULAR_BUFFER_SIZE)
        circular_buffer->buffer_length++;

    if (circular_buffer->buffer_length >= MIN_VALUES_FOR_MEAN) {
        compute_mean_value(circular_buffer);
    }

    return 0;
}

static int read_buffer(struct fine_circular_buffer *circular_buffer, union fine_value *returned_value)
{
    if (!circular_buffer) {
        log_error("Circular buffer is NULL");
        return -EINVAL;
    }

    if (circular_buffer->buffer_length == circular_buffer->read_index) {
        log_trace("index %d, all data has been read", circular_buffer->read_index);
        return -1;
    }

    if (circular_buffer->fine_type == 'A') {
        returned_value->fine_applied = circular_buffer->buffer[circular_buffer->read_index].fine_applied;
    } else if (circular_buffer->fine_type == 'S') {
        returned_value->fine_estimated_equilibrium_ES = circular_buffer->buffer[circular_buffer->read_index].fine_estimated_equilibrium_ES;
    } else {
        log_error("Buffer is not of type 'A' or 'S'");
        return -EINVAL;
    }

    /* Increment read index */
    circular_buffer->read_index++;

    return 0;
}

void print_tuples(struct fine_circular_buffer *circular_buffer)
{
    int i;
    int index = 0;

    if (!circular_buffer)
        return;

    if (circular_buffer->fine_type == 'A') {
        for (i = 0; i < circular_buffer->buffer_length; i++) {
            index = circular_buffer->read_index + i;
            if (index >= CIRCULAR_BUFFER_SIZE) {
                index -= CIRCULAR_BUFFER_SIZE;
            }
            log_debug("buffer[%u] = %u",
                i,
                circular_buffer->buffer[index].fine_applied);
        }
    } else if (circular_buffer->fine_type == 'S') {
        for (i = 0; i < circular_buffer->buffer_length; i++) {
            index = circular_buffer->read_index + i;
            if (index >= CIRCULAR_BUFFER_SIZE) {
                index -= CIRCULAR_BUFFER_SIZE;
            }
            log_debug("buffer[%u] = %f",
                i,
                circular_buffer->buffer[index].fine_estimated_equilibrium_ES);
        }
    } else {
        log_error("Buffer is not of type 'A' or 'S'");
        return;
    }
}

int get_index_of_temperature(float temperature) {
    if (temperature < MIN_TEMPERATURE) {
        return -1;
    } else if (temperature >= MAX_TEMPERATURE) {
        return TEMPERATURE_STEPS;
    } else {
        return (int) floor(STEPS_BY_DEGREE * (temperature - MIN_TEMPERATURE));
    }
}

int add_fine_from_temperature(struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS], union fine_value fine, double temp)
{
    int index;
    int ret;

    if (temp < MIN_TEMPERATURE || temp > MAX_TEMPERATURE) {
        log_error("Temperature is out of range:  %f, range is [%f, %f]", temp, MIN_TEMPERATURE, MAX_TEMPERATURE);
        return -1;
    }

    /* Compute index of fine_buffer value should be stored */
    index = get_index_of_temperature(temp);

    log_debug("Adding data at temperature %.2f in index %d", temp, index);

    ret = write_fine(&fine_buffer[index], fine);
    if (ret != 0) {
        log_error("Could not add tuple to buffer !");
    }

    return ret;
}

/**
 * @brief Compute mean value over circular buffer if there are more than MIN_VALUES_FOR_MEAN in buffer
 *
 * @param fine_buffer
 * @return int 0 on success else error
 */
int compute_mean_value(struct fine_circular_buffer *fine_buffer)
{
    if (!fine_buffer) {
        log_error("Fine buffer is NULL");
        return -EINVAL;
    }

    if (fine_buffer->buffer_length == 0) {
        log_trace("empty buffer");
        return -EINVAL;
    }

    fine_buffer->mean_fine = 0.0;

    union fine_value fine;

    if (fine_buffer->fine_type == 'A') {
        while (read_buffer(fine_buffer, &fine) == 0) {
            fine_buffer->mean_fine += fine.fine_applied;
        }

    } else if (fine_buffer->fine_type == 'S') {
        while (read_buffer(fine_buffer, &fine) == 0) {
            fine_buffer->mean_fine += fine.fine_estimated_equilibrium_ES;
        }

    } else {
        log_error("Buffer is not of type 'A' or 'S'");
        return -EINVAL;
    }
    fine_buffer->mean_fine = fine_buffer->mean_fine / fine_buffer->buffer_length;

    fine_buffer->read_index = 0;
    return 0;
}

void update_mean_values(struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS])
{
    int i;
    for (i = 0; i < TEMPERATURE_STEPS; i++) {
        if (fine_buffer[i].buffer_length >= MIN_VALUES_FOR_MEAN) {
            compute_mean_value(&fine_buffer[i]);
        }
    }
}

int find_closest_left_circular_buffer_from_index(struct fine_circular_buffer *fine_buffer, int temperature_index)
{
    int left_operand_index = -1;
    int i;
    /* Find closest left circular buffer with enough value to compute mean fine */
    for(i = (temperature_index == TEMPERATURE_STEPS) ? TEMPERATURE_STEPS - 1:temperature_index - 1; i >= 0; i--) {
        if (fine_buffer[i].buffer_length >= MIN_VALUES_FOR_MEAN) {
            log_debug("temperature range [%.2f, %.2f[ (index %d) can be used as left operand for fine computing",
                    (i + STEPS_BY_DEGREE * MIN_TEMPERATURE) / STEPS_BY_DEGREE,
                    (i + 1 + STEPS_BY_DEGREE * MIN_TEMPERATURE) / STEPS_BY_DEGREE,
                    i
                );
            compute_mean_value(&fine_buffer[i]);
            left_operand_index = i;
            break;
        }
    }
    return left_operand_index;
}

int find_closest_right_circular_buffer_from_index(struct fine_circular_buffer *fine_buffer, int temperature_index)
{
    int right_operand_index = -1;
    int i;
    /* Find closest right circular buffer with enough value to compute mean fine */
    for(i = temperature_index + 1; i < TEMPERATURE_STEPS; i++) {
        if (fine_buffer[i].buffer_length >= MIN_VALUES_FOR_MEAN) {
            log_debug("temperature range [%.2f, %.2f[ (index %d) can be used as right operand for fine computing",
                    (i + STEPS_BY_DEGREE * MIN_TEMPERATURE) / STEPS_BY_DEGREE,
                    (i + 1 + STEPS_BY_DEGREE * MIN_TEMPERATURE) / STEPS_BY_DEGREE,
                    i
                );
            compute_mean_value(&fine_buffer[i]);
            right_operand_index = i;
            break;
        }
    }
    return right_operand_index;
}

static float get_fine_from_default_temperature_compensation(float current_temperature, float reference_temperature, float reference_fine_value) {
    if (compare_float(current_temperature, reference_temperature) == 1) {
            return reference_fine_value;
    } else if (current_temperature < reference_temperature) {
        /* Temperature decreased from holdover entry */
        if (reference_temperature < MIN_TEMPERATURE_DEFAULT) {
            return reference_fine_value;
        } else if (reference_temperature >= MAX_TEMPERATURE_DEFAULT) {
            if (current_temperature < MIN_TEMPERATURE_DEFAULT) {
                return reference_fine_value
                    + 2 * DEFAULT_DELTA_TEMPERATURE_COEFF * (MAX_TEMPERATURE_DEFAULT - reference_temperature)
                    + DEFAULT_DELTA_TEMPERATURE_COEFF * (MIN_TEMPERATURE_DEFAULT - MAX_TEMPERATURE_DEFAULT);
            } else if (current_temperature >= MAX_TEMPERATURE_DEFAULT) {
                return reference_fine_value
                    + 2 * DEFAULT_DELTA_TEMPERATURE_COEFF * (current_temperature - reference_temperature);
            } else {
                return reference_fine_value
                    + 2 * DEFAULT_DELTA_TEMPERATURE_COEFF * (MAX_TEMPERATURE_DEFAULT - reference_temperature)
                    + DEFAULT_DELTA_TEMPERATURE_COEFF * (current_temperature - MAX_TEMPERATURE_DEFAULT);
            }
        } else {
            /* 30 <= reference_temperature < 40 */
            if (current_temperature < MIN_TEMPERATURE_DEFAULT) {
                return reference_fine_value
                    + DEFAULT_DELTA_TEMPERATURE_COEFF * (MIN_TEMPERATURE_DEFAULT - reference_temperature);
            } else {
                return reference_fine_value
                    + DEFAULT_DELTA_TEMPERATURE_COEFF * (current_temperature - reference_temperature);
            }
        }
    } else if(current_temperature > reference_temperature) {
        /* Temperature increased from holdover entry */
        if (reference_temperature < MIN_TEMPERATURE_DEFAULT) {
            if (current_temperature < MIN_TEMPERATURE_DEFAULT) {
                return reference_fine_value;
            } else if (current_temperature >= MAX_TEMPERATURE_DEFAULT) {
                return reference_fine_value
                    + DEFAULT_DELTA_TEMPERATURE_COEFF * (MAX_TEMPERATURE_DEFAULT - MIN_TEMPERATURE_DEFAULT)
                    + 2 * DEFAULT_DELTA_TEMPERATURE_COEFF * (current_temperature - MAX_TEMPERATURE_DEFAULT);
            } else {
                return reference_fine_value
                    + DEFAULT_DELTA_TEMPERATURE_COEFF * (current_temperature - MIN_TEMPERATURE_DEFAULT);
            }
        } else if (reference_temperature >= MAX_TEMPERATURE_DEFAULT) {
            return reference_fine_value
                + 2 * DEFAULT_DELTA_TEMPERATURE_COEFF * (current_temperature - reference_temperature);
        } else {
            /* 30 <= reference_temperature < 40 */
            if (current_temperature >= MAX_TEMPERATURE_DEFAULT) {
                return reference_fine_value
                    + DEFAULT_DELTA_TEMPERATURE_COEFF * (MAX_TEMPERATURE_DEFAULT - reference_temperature)
                    + 2 * DEFAULT_DELTA_TEMPERATURE_COEFF * (current_temperature - MAX_TEMPERATURE_DEFAULT);
            } else {
                return reference_fine_value
                    + DEFAULT_DELTA_TEMPERATURE_COEFF * (current_temperature - reference_temperature);
            }
        }
    }
    return -1.0;
}

#define MID_TEMPERATURE_FROM_INDEX(i) (i + STEPS_BY_DEGREE * (MIN_TEMPERATURE + 0.5)) / STEPS_BY_DEGREE

float get_fine_from_table(struct algorithm_state *state, float input_temperature)
{
    int input_temperature_index;
    int left_operand_index = -1;
    int right_operand_index = -1;
    float left_operand_temperature = -1.0;
    float right_operand_temperature = -1.0;

    struct fine_circular_buffer *fine_buffer = (struct fine_circular_buffer *) &state->fine_estimated_es_buffer;
    input_temperature_index = get_index_of_temperature(input_temperature);

    /* Check if a mean value can be computed on bin of input temperature */
    if (input_temperature >= MIN_TEMPERATURE
        && input_temperature < MAX_TEMPERATURE
        && fine_buffer[input_temperature_index].buffer_length >= MIN_VALUES_FOR_MEAN) {
        return fine_buffer[input_temperature_index].mean_fine;
    }

    right_operand_index = find_closest_right_circular_buffer_from_index(fine_buffer, input_temperature_index);
    left_operand_index = find_closest_left_circular_buffer_from_index(fine_buffer, input_temperature_index);

    if (right_operand_index == -1 && left_operand_index == -1) {
        /* We have no value in table */
        return get_fine_from_default_temperature_compensation(
            input_temperature,
            state->holdover_mRO_EP_temperature,
            state->estimated_equilibrium_ES
        );
    /* From now on we know we have at least one value usable in the table */
    } else if (right_operand_index != -1 && left_operand_index != -1) {
        left_operand_temperature = MID_TEMPERATURE_FROM_INDEX(left_operand_index);
        right_operand_temperature = MID_TEMPERATURE_FROM_INDEX(right_operand_index);

        float coef_a = (fine_buffer[right_operand_index].mean_fine - fine_buffer[left_operand_index].mean_fine)
            / (right_operand_temperature - left_operand_temperature);
        return fine_buffer[left_operand_index].mean_fine + coef_a * (input_temperature - left_operand_temperature);
    } else if (left_operand_index == -1) {
        /* We have a mean value on the right side of the input temperature index */
        right_operand_temperature = MID_TEMPERATURE_FROM_INDEX(right_operand_index);
        return get_fine_from_default_temperature_compensation(
            input_temperature,
            right_operand_temperature,
            fine_buffer[right_operand_index].mean_fine
        );
    } else {
        /* We have a mean value on the left side of the input temperature */
        left_operand_temperature = MID_TEMPERATURE_FROM_INDEX(left_operand_index);
        return get_fine_from_default_temperature_compensation(
            input_temperature,
            left_operand_temperature,
            fine_buffer[left_operand_index].mean_fine
        );
    }
    return -1.0;
}

int write_buffers_in_file(struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS], const char* output_file)
{
    FILE *fp;
    int i;

    fp = fopen(output_file, "w");
    if (!fp) {
        return -EINVAL;
    }

    for (i = 0; i < TEMPERATURE_STEPS; i++) {
        char fine_char[2048] = { '\0' };

        union fine_value fine;

        while(read_buffer(&fine_buffer[i], &fine) == 0) {
            char temp[32] = { 0 };

            if (fine_buffer->fine_type == 'A') {
                sprintf(temp, ",%u", fine.fine_applied);
            } else if (fine_buffer->fine_type == 'S') {
                sprintf(temp, ",%.2f", fine.fine_estimated_equilibrium_ES);

            } else {
                log_error("Buffer is not of type 'A' or 'S'");
                return -EINVAL;
            }
            strcat(fine_char, temp);
        }

        fine_buffer[i].read_index = 0;

        char line[4096] = { '\0' };
        char temp[32] = { '\0' };
        sprintf(temp, "%.2f", MIN_TEMPERATURE + (float) i / STEPS_BY_DEGREE);
        strcat(line, temp);
        strcat(line, fine_char);
        strcat(line, "\n");
        fputs(line, fp);

        if (fine_buffer[i].buffer_length >= MIN_VALUES_FOR_MEAN && compute_mean_value(&fine_buffer[i]) == 0) {
            if (fine_buffer->fine_type == 'A') {
                log_debug("FINE APPLIED: Mean temperature over range [%.2f, %.2f[ is : %.2f",
                    (i + STEPS_BY_DEGREE * MIN_TEMPERATURE) / STEPS_BY_DEGREE,
                    (i + 1 + STEPS_BY_DEGREE * MIN_TEMPERATURE) / STEPS_BY_DEGREE,
                    fine_buffer[i].mean_fine
                );
            } else if (fine_buffer->fine_type == 'S') {
                log_debug("FINE ESTIMATED ES: Mean temperature over range [%.2f, %.2f[ is : %.2f",
                    (i + STEPS_BY_DEGREE * MIN_TEMPERATURE) / STEPS_BY_DEGREE,
                    (i + 1 + STEPS_BY_DEGREE * MIN_TEMPERATURE) / STEPS_BY_DEGREE,
                    fine_buffer[i].mean_fine
                );
            } else {
                log_error("Buffer is not of type 'A' or 'S'");
                return -EINVAL;
            }
        }
    }

    fclose(fp);


    return 0;
}
