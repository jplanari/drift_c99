/*
 * drift_rng.c - Vegeu drift_rng.h. Dependències: només libm (logf, sqrtf, cosf).
 */
#include "drift_rng.h"

#include <math.h>

#define DRIFT_TWO_PI 6.2831853071795864769f

void drift_rng_seed(drift_rng_t *r, uint32_t seed)
{
    r->state = (seed != 0u) ? seed : 0x9E3779B9u;
}

static uint32_t xorshift32(drift_rng_t *r)
{
    uint32_t x = r->state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    r->state = x;
    return x;
}

float drift_rng_uniform(drift_rng_t *r)
{
    /* (k + 0.5) / 2^24 amb k a [0, 2^24): mai 0 ni 1, així logf és finit. */
    uint32_t k = xorshift32(r) >> 8;
    return ((float)k + 0.5f) * (1.0f / 16777216.0f);
}

float drift_rng_randn(drift_rng_t *r, float mean, float stddev)
{
    float u1 = drift_rng_uniform(r);
    float u2 = drift_rng_uniform(r);
    float radius = sqrtf(-2.0f * logf(u1));
    float z = radius * cosf(DRIFT_TWO_PI * u2);
    return mean + stddev * z;
}
