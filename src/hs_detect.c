/*
 * hs_detect.c - Vegeu hs_detect.h.
 */
#include "hs_detect.h"
#include "drift_config.h"

#include <string.h>

#define N_PIX (DRIFT_IMG_W * DRIFT_IMG_H)

/* Bitmap de píxels visitats (1 bit per píxel). ~38 KB per 640x480, ~98 KB per 1024x768. */
static uint8_t visited[(N_PIX + 7) / 8];

/* Pila de flood-fill. Cada píxel s'empila com a màxim una vegada (marcat en empilar). */
static uint32_t stk[DRIFT_STACK_CAP];

static int is_visited(uint32_t i)
{
    return (visited[i >> 3] >> (i & 7u)) & 1u;
}

static void set_visited(uint32_t i)
{
    visited[i >> 3] |= (uint8_t)(1u << (i & 7u));
}

int hs_detect(const float *img, float thr, uint32_t min_area,
              hs_point *out, int max_out, int *dropped)
{
    int n_out = 0;
    int n_drop = 0;
    uint32_t seed;

    if (img == NULL || out == NULL || max_out < 0) {
        return HS_ERR_ARG;
    }

    memset(visited, 0, sizeof(visited));

    for (seed = 0; seed < (uint32_t)N_PIX; seed++) {
        int sp;
        uint32_t count = 0;
        double sum_x = 0.0;
        double sum_y = 0.0;

        if (!(img[seed] > thr) || is_visited(seed)) {
            continue;
        }

        /* Flood-fill 8-connectat amb pila explícita. */
        sp = 0;
        stk[sp++] = seed;
        set_visited(seed);

        while (sp > 0) {
            uint32_t idx = stk[--sp];
            int x = (int)(idx % DRIFT_IMG_W);
            int y = (int)(idx / DRIFT_IMG_W);
            int dy, dx;

            count++;
            sum_x += (double)x;
            sum_y += (double)y;

            for (dy = -1; dy <= 1; dy++) {
                for (dx = -1; dx <= 1; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    uint32_t nidx;

                    if (dx == 0 && dy == 0) continue;
                    if (nx < 0 || nx >= DRIFT_IMG_W || ny < 0 || ny >= DRIFT_IMG_H) continue;

                    nidx = (uint32_t)ny * DRIFT_IMG_W + (uint32_t)nx;
                    if (!(img[nidx] > thr) || is_visited(nidx)) continue;

                    if (sp >= DRIFT_STACK_CAP) {
                        return HS_ERR_STACK;
                    }
                    set_visited(nidx);
                    stk[sp++] = nidx;
                }
            }
        }

        if (count < min_area) {
            continue;
        }

        if (n_out < max_out) {
            out[n_out].x = (float)(sum_x / (double)count);
            out[n_out].y = (float)(sum_y / (double)count);
            out[n_out].area = count;
            n_out++;
        } else {
            n_drop++;
        }
    }

    if (dropped != NULL) {
        *dropped = n_drop;
    }
    return n_out;
}

float hs_window_max(const float *img, float px, float py, int rad)
{
    /* Mateixes truncacions que maxHotspotTemp: (int) del float, després resta/suma. */
    int x1 = (int)px - rad;
    int y1 = (int)py - rad;
    int x2 = (int)px + rad;
    int y2 = (int)py + rad;
    int y, x;
    float maxv = 0.0f;
    int found = 0;

    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 > DRIFT_IMG_W) x2 = DRIFT_IMG_W;
    if (y2 > DRIFT_IMG_H) y2 = DRIFT_IMG_H;

    if (x2 - x1 <= 0 || y2 - y1 <= 0) {
        return 0.0f;
    }

    for (y = y1; y < y2; y++) {
        for (x = x1; x < x2; x++) {
            float v = img[(uint32_t)y * DRIFT_IMG_W + (uint32_t)x];
            if (!found || v > maxv) {
                maxv = v;
                found = 1;
            }
        }
    }
    return maxv;
}
