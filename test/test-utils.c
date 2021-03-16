#include "../src/utils.h"

#include <assert.h>
#include <stdio.h>
#include <stdbool.h>

int main() {
    int ret;

    printf("TEST utils program\n");

    printf("Testing simple linear reg\n");
    /* Test 1 */
    double x1[4] = { 1.0, 2.0, 4.0, 8.0 };
    double y1[4] = { 30.0, 50.0, 90.0, 170.0 };

    struct linear_func_param test_result;

    ret = simple_linear_reg(x1, y1, 4, &test_result);

    if (ret != 0) {
        printf("Error occured \n");
    } else {
        printf("a = %f\n", test_result.a);
        printf("b = %f\n", test_result.b);
        assert(test_result.a == 20.0);
        assert(test_result.b == 10.0);
        printf("Test passed\n");
    }

    printf("Testing lin_interp\n");
    double x2[5] = { 1.0, 2.0, 3.0, 4.0, 5.0 };
    double y2[5] = { 10.0, 8.0, 12.0, 10.0, 9.0};
    double result;

    /* X interpolation */
    ret = lin_interp(x2, y2, 5, true, 1.5, &result);
    assert(ret == 0);
    assert(result == 9.0);

    ret = lin_interp(x2, y2, 5, true, 2.5, &result);
    assert(ret == 0);
    assert(result == 10.0);

    ret = lin_interp(x2, y2, 5, true, 3.5, &result);
    assert(ret == 0);
    assert(result == 11.0);

    ret = lin_interp(x2, y2, 5, true, 4.5, &result);
    assert(ret == 0);
    assert(result == 9.5);

    /* Y interpolation */
    double x3[5] = { 1.0, 2.0, 3.0, 4.0, 5.0 };
    double y3[5] = { 10.0, 15.0, 25.0, 50.0, 100.0};

    ret = lin_interp(x3, y3, 5, false, 12.5, &result);
    assert(ret == 0);
    assert(result == 1.5);

    ret = lin_interp(x3, y3, 5, false, 75.0, &result);
    assert(ret == 0);
    assert(result == 4.5);

    ret = lin_interp(x3, y3, 5, false, 150.0, &result);
    assert(ret == 0);
    assert(result == 6.0);

    ret = lin_interp(x3, y3, 5, false, 7.5, &result);
    assert(ret == 0);
    assert(result == 0.5);
    printf("Test passed\n");
}
