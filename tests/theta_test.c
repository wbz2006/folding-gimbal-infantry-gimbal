#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "user_lib.h"
int main(void)
{
    assert(theta_format(180) == 180 && theta_format(-180) == -180);
    assert(theta_format(540) == 180 && theta_format(-540) == -180);
    assert(theta_format(360) == 0 && theta_format(-360) == 0);
    assert(fabsf(theta_format(359.75f) + 0.25f) < 0.0001f);
    assert(fabsf(theta_format(-359.75f) - 0.25f) < 0.0001f);
    const float large[] = {FLT_MAX, -FLT_MAX, 1e20f, -1e20f};
    for (unsigned i = 0; i < sizeof(large)/sizeof(large[0]); ++i)
    {
        float angle = theta_format(large[i]);
        assert(isfinite(angle) && angle >= -180 && angle <= 180);
    }
    assert(isnan(theta_format(INFINITY)) && isnan(theta_format(NAN)));
    puts("PASS production theta_format: signed endpoints, wraparound, huge finite inputs and nonfinite termination");
}
