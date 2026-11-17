/*
 * hs_image.c - Vegeu hs_image.h.
 */
#include "hs_image.h"

#include <stddef.h>

int hs_normalize_minmax_u8(const float *src, uint8_t *dst, int n)
{
    float vmin, vmax, scale;
    int i;

    if (src == NULL || dst == NULL || n <= 0) {
        return -1;
    }

    vmin = src[0];
    vmax = src[0];
    for (i = 1; i < n; i++) {
        if (src[i] < vmin) vmin = src[i];
        if (src[i] > vmax) vmax = src[i];
    }

    if (!(vmax > vmin)) {
        for (i = 0; i < n; i++) dst[i] = 0u;
        return 0;
    }

    scale = 255.0f / (vmax - vmin);
    for (i = 0; i < n; i++) {
        float v = (src[i] - vmin) * scale + 0.5f;   /* arrodoniment a l'enter més proper */
        if (v < 0.0f) v = 0.0f;
        if (v > 255.0f) v = 255.0f;
        dst[i] = (uint8_t)v;
    }
    return 0;
}
