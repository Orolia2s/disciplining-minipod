#include "fine_circular_buffer.h"

#include "errno.h"
#include "log.h"
#include "math.h"
#include "string.h"

static int write_fine(struct fine_circular_buffer *circular_buffer, union fine_value fine)
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

int add_fine_from_temperature(struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS], union fine_value fine, double temp)
{
    int index;
    int ret;

    if (temp < MIN_TEMPERATURE || temp > MAX_TEMPERATURE) {
        log_error("Temperature is out of range:  %f, range is [%f, %f]", temp, MIN_TEMPERATURE, MAX_TEMPERATURE);
        return -1;
    }

    /* Compute index of fine_buffer value should be stored */
    index = (int) floor(STEPS_BY_DEGREE * (temp - MIN_TEMPERATURE));

    log_debug("Adding data at temperature %.2f in index %d", temp, index);

    ret = write_fine(&fine_buffer[index], fine);
    if (ret != 0) {
        log_error("Could not add tuple to buffer !");
    }

    return ret;
}

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

        if (fine_buffer[i].buffer_length >= 10 && compute_mean_value(&fine_buffer[i]) == 0) {
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
