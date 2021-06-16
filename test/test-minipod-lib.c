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
    assert(calib_params->ctrl_points[0] == 1600);
    assert(calib_params->ctrl_points[1] == 2400);
    assert(calib_params->ctrl_points[2] == 3200);
    printf("*** TEST PASSED ***\n\n");
}
