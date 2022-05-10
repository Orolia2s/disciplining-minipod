#include "fine_circular_buffer.h"

#include "errno.h"
#include "log.h"
#include "math.h"
#include "string.h"

static int write_tuple(struct fine_circular_buffer *circular_buffer, struct fine_tuple tuple)
{
    if (!circular_buffer) {
        log_error("Circular buffer is NULL");
        return - EINVAL;
    }

    /* Write tuple at write index */
    circular_buffer->buffer[circular_buffer->write_index].fine_applied = tuple.fine_applied;
    circular_buffer->buffer[circular_buffer->write_index].fine_estimated_equilibrium_ES = tuple.fine_estimated_equilibrium_ES;
    /* Increment write index */
    circular_buffer->write_index++;
    if (circular_buffer->write_index == CIRCULAR_BUFFER_SIZE)
        circular_buffer->write_index = 0;
    /* Increment buffer length */
    if (circular_buffer->buffer_length < CIRCULAR_BUFFER_SIZE)
        circular_buffer->buffer_length++;

    return 0;
}

int read_buffer(struct fine_circular_buffer *circular_buffer, struct fine_tuple *returned_value)
{
    if (!circular_buffer) {
        log_error("Circular buffer is NULL");
        return -EINVAL;
    }

    if (circular_buffer->buffer_length == circular_buffer->read_index) {
        log_trace("index %d, all data has been read", circular_buffer->read_index);
        return -1;
    }

    returned_value->fine_applied = circular_buffer->buffer[circular_buffer->read_index].fine_applied;
    returned_value->fine_estimated_equilibrium_ES = circular_buffer->buffer[circular_buffer->read_index].fine_estimated_equilibrium_ES;

    /* Increment read index */
    circular_buffer->read_index++;

    return 0;
}

void print_tuples(struct fine_circular_buffer *circular_buffer)
{
    int i;
    int index = 0;
    for (i = 0; i < circular_buffer->buffer_length; i++) {
        index = circular_buffer->read_index + i;
        if (index >= CIRCULAR_BUFFER_SIZE) {
            index -= CIRCULAR_BUFFER_SIZE;
        }
        log_debug("buffer[%u] = %f, %u",
            i,
            circular_buffer->buffer[index].fine_applied,
            circular_buffer->buffer[index].fine_estimated_equilibrium_ES);
    }
}

int add_fine_from_temperature(struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS], uint16_t fine_applied, float fine_estimated_ES, double temp)
{
    int index;
    int ret;

    if (temp < MIN_TEMPERATURE || temp > MAX_TEMPERATURE) {
        log_error("Temperature is out of range:  %f, range is [%f, %f]", temp, MIN_TEMPERATURE, MAX_TEMPERATURE);
        return -1;
    }

    /* Compute index of fine_buffer value should be stored */
    index = (int) floor(2 * (temp - MIN_TEMPERATURE));

    log_debug("Adding data at temperature %.2f in index %d", temp, index);
    struct fine_tuple tuple = {
        .fine_applied = fine_applied,
        .fine_estimated_equilibrium_ES = fine_estimated_ES
    };

    ret = write_tuple(&fine_buffer[index], tuple);
    if (ret != 0) {
        log_error("Could not add tuple to buffer !");
    }

    return ret;
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
        char fine_applied_char[2048] = { '\0' };
        char fine_estimated_char[2048] = { '\0' };

        struct fine_tuple tuple;

        while(read_buffer(&fine_buffer[i], &tuple) == 0) {
            char temp[32] = { 0 };
            char temp_ES[32] = { 0 };
            sprintf(temp, "%u,", tuple.fine_applied);
            strcat(fine_applied_char, temp);
            sprintf(temp_ES, "%.1f,", tuple.fine_estimated_equilibrium_ES);
            strcat(fine_estimated_char, temp_ES);
        }

        fine_buffer[i].read_index = 0;

        char line[4096] = { '\0' };
        char temp[32] = { '\0' };
        sprintf(temp, "%.1f,", MIN_TEMPERATURE + i * 0.5);
        strcat(line, temp);
        strcat(line, fine_applied_char);
        strcat(line, fine_estimated_char);
        strcat(line, "\n");
        fputs(line, fp);
    }

    fclose(fp);


    return 0;
}
