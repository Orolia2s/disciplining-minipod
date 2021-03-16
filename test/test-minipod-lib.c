#include <oscillator-disciplining/oscillator-disciplining.h>

#include <assert.h>
#include <stdio.h>

/**
 * TEST MINIPOD LIBRARY
 * Test the interface of the oscillator disciplining library
 */
int main()
{
    char err_msg[OD_ERR_MSG_LEN];
    struct od *od;

    /* TEST od_new_from_config */
    printf("\n*** TEST INCOMPLETE CONFIG ***\n");
    od = od_new_from_config("./test_incomplete.conf", err_msg);
    assert(od == NULL);
    printf("*** TEST PASSED ***\n\n");


    printf("*** TEST VALID CONFIG ***\n");
    od = od_new_from_config("./test_valid.conf", err_msg);
    assert(od != NULL);
    printf("*** TEST PASSED ***\n\n");

    printf("*** TEST GET CALIBRATION PARAMETERS ***\n");
    /* Test od_get_calibration_parameters */
    struct calibration_parameters *calib_params;

    calib_params = od_get_calibration_parameters(od);
    assert(calib_params->length == 3);
    assert(calib_params->nb_calibration == 30);
    assert(calib_params->settling_time == 5);
    assert(calib_params->ctrl_points[0] == 1600);
    assert(calib_params->ctrl_points[1] == 2400);
    assert(calib_params->ctrl_points[2] == 3200);
    printf("*** TEST PASSED ***\n\n");
}
