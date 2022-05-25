#include <assert.h>
#include "stdio.h"

#include "compare_floats.h"
#include "../src/log.h"
#include "../src/fine_circular_buffer.h"

static void test_get_index_of_temperature()
{
    assert(get_index_of_temperature(45.6) == 102);
    assert(get_index_of_temperature(38.32) == 73);
    assert(get_index_of_temperature(55.25) == 141);
    assert(get_index_of_temperature(19.7) == -1);
    assert(get_index_of_temperature(60) == TEMPERATURE_STEPS);
}

static void test_get_delta_fine_from_temperature (struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS])
{
    assert(compare_float(get_delta_fine_from_temperature_table(fine_buffer, 45.6, 45.6, 2400.0), 0.0));
    assert(compare_float(get_delta_fine_from_temperature_table(fine_buffer, 38.32, 45.6, 2400.0), -31.933105));
    assert(compare_float(get_delta_fine_from_temperature_table(fine_buffer, 55.25, 45.6, 2400.0), 42.933350));
    assert(compare_float(get_delta_fine_from_temperature_table(fine_buffer, 19.7, 45.6, 2400.0), 0.0));
    assert(compare_float(get_delta_fine_from_temperature_table(fine_buffer, 60, 45.6, 2400.0), 61.983398));

}

int main(int argc, char **argv)
{
    struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS];
    int i, j, ret;

    log_set_level(LOG_INFO);

    /* Init fine_circular_buffer */
    for (i = 0; i < TEMPERATURE_STEPS; i ++) {
        fine_buffer[i].buffer_length = 0;
        fine_buffer[i].read_index = 0;
        fine_buffer[i].write_index = 0;
        fine_buffer[i].fine_type = 'S';
        int j;
        for (j = 0; j < CIRCULAR_BUFFER_SIZE; j++) {
            fine_buffer[i].buffer[j].fine_estimated_equilibrium_ES = 0.0;
        }
    }

    /* Fill buffer with data */
    for (i = 0; i < TEMPERATURE_STEPS; i++) {
        for (j = 0; j < CIRCULAR_BUFFER_SIZE- 10; j++) {
            double temperature = (float) ((float) i + MIN_TEMPERATURE * STEPS_BY_DEGREE) / STEPS_BY_DEGREE;
            union fine_value fine_estimated = {
                .fine_estimated_equilibrium_ES = 2000.0 + i + j + (i + j)/10
            };
            ret = add_fine_from_temperature(fine_buffer, fine_estimated, temperature);
            if (ret != 0) {
                printf("Could not add data to buffer\n");
            }
        }
    }

    ret = write_buffers_in_file(fine_buffer, "./output_buffer.txt");
    if (ret != 0) {
        printf("Error writing file\n");
    }

    test_get_index_of_temperature();
    test_get_delta_fine_from_temperature(fine_buffer);
    log_info("PASSED");
    return 0;
}
