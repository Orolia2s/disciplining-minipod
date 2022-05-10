#include "stdio.h"

#include "../src/fine_circular_buffer.h"

int main(int argc, char **argv)
{
    struct fine_circular_buffer fine_buffer[TEMPERATURE_STEPS];
    int i, j, ret;

    /* Init fine_circular_buffer */
    for (i = 0; i < TEMPERATURE_STEPS; i ++) {
        fine_buffer[i].buffer_length = 0;
        fine_buffer[i].read_index = 0;
        fine_buffer[i].write_index = 0;
        int j;
        for (j = 0; j < CIRCULAR_BUFFER_SIZE; j++) {
            fine_buffer[i].buffer[j].fine_applied = 0;
            fine_buffer[i].buffer[j].fine_estimated_equilibrium_ES = 0.0;
        }
    }

    /* Fill buffer with data */
    for (i = 0; i < TEMPERATURE_STEPS; i+=2) {
        for (j = 0; j < CIRCULAR_BUFFER_SIZE- 10; j++) {
            double temperature = (i + MIN_TEMPERATURE * 2.0) / 2;
            uint16_t fine_applied = 0;
            float fine_estimated = (float) 0.0;
            ret = add_fine_from_temperature(fine_buffer, fine_applied, fine_estimated, temperature);
            if (ret != 0) {
                printf("Could not add data to buffer\n");
            }
        }
    }

    ret = write_buffers_in_file(fine_buffer, "./output_buffer.txt");
    if (ret != 0) {
        printf("Error writing file\n");
    }
    return 0;
}