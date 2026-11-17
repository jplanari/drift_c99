/*
 * drift_rng.h - Generador pseudoaleatori i gaussià (substitueix cv::randn).
 *
 * xorshift32 per a l'uniforme (sense libc rand) i Box-Muller amb logf, sqrtf i
 * cosf de libm per al gaussià. Només dependeix de LibmCS.
 */
#ifndef DRIFT_RNG_H
#define DRIFT_RNG_H

#include <stdint.h>

typedef struct {
    uint32_t state;   /* mai zero */
} drift_rng_t;

void drift_rng_seed(drift_rng_t *r, uint32_t seed);

/* Uniforme a (0, 1). */
float drift_rng_uniform(drift_rng_t *r);

/* Gaussià amb mitjana mean i desviació stddev (Box-Muller). */
float drift_rng_randn(drift_rng_t *r, float mean, float stddev);

#endif /* DRIFT_RNG_H */
