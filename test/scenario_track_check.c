/*
 * scenario_track_check.c - Prova d'extrem a extrem sobre els escenaris de referència.
 *
 * Cadena: física (ref_scenario.csv) -> render IR -> hs_detect -> tracker_update,
 * comparada amb la veritat del physics.
 *
 * Render IR: transcripció de DataGenerator::getNextFrame sense soroll
 * (fons 250 K, gaussiana sigma=15 px, truncada a 3 sigma, s'afegeix si > 1 K).
 * El soroll (cv::randn, sigma 2) no es reprodueix; amb 35 K de marge sobre el
 * llindar és prou per al 1035 K de l'objecte 1.
 *
 * Criteris de pas (tots han de complir-se):
 *   - Nombre de deteccions = nombre d'hotspots visibles (T > 1000 K) a cada frame.
 *   - Cap detecció ni track a prop d'un hotspot no visible (p.ex. 285 K).
 *   - Després del warmup, cada hotspot visible té un track a <= 2 px en >= 95 % dels frames.
 *   - Després del warmup, el nombre de tracks = nombre d'hotspots visibles.
 *   - Error de detecció (centroide vs centre real) <= 0.5 px.
 *   - Error de velocitat relativa del track (després de 30 frames) <= 0.05 px/frame de mitjana.
 *
 * Ús:  scenario_track_check <ref_scenario.csv> [warmup=15]
 * Només host. Enllaça amb -lm (expf, roundf), no amb LibmCS.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drift_config.h"
#include "hs_detect.h"
#include "tracker.h"

#define MAX_ROWS      4000
#define MAX_OBJ       16
#define TEMP_THRESH   1000.0f
#define MIN_AREA      10u
#define SIGMA         15.0f
#define MATCH_PX      2.0
#define DET_TOL_PX    0.5
#define VEL_TOL       0.05
#define TRACK_RATE    0.95
#define VEL_WARMUP    30
#define EDGE_PX       45   /* 3 sigma: disc tallat per la vora de la imatge */

typedef struct {
    int frame, obj;
    float x, y, vx_rel, vy_rel, vx_abs, vy_abs, mach, temp;
} truth_t;

static truth_t tr[MAX_ROWS];
static int n_tr = 0;
static int n_obj = 0;
static int n_frames = 0;

static float img[DRIFT_IMG_H * DRIFT_IMG_W];
static hs_point det[DRIFT_MAX_DETECTIONS];

/* Per-object statistics (visible objects only). */
static int vis_frames[MAX_OBJ];
static int tracked_frames[MAX_OBJ];
static int hidden_det_hits = 0;
static int hidden_track_hits = 0;
static int det_count_mismatch = 0;
static int track_count_mismatch = 0;
static double det_err_sum = 0.0, det_err_max = 0.0;
static long det_err_n = 0;
static double edge_err_max = 0.0;
static int edge_frames = 0;
static double vel_err_sum[MAX_OBJ];
static long vel_err_n[MAX_OBJ];

static double wrap_delta(double d, double size)
{
    return d - size * round(d / size);
}

static double wrap_dist(double x1, double y1, double x2, double y2)
{
    double dx = wrap_delta(x1 - x2, DRIFT_IMG_W);
    double dy = wrap_delta(y1 - y2, DRIFT_IMG_H);
    return sqrt(dx * dx + dy * dy);
}

static int read_truth(const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[512];
    int a, b;
    float c[8];

    if (!fp) {
        fprintf(stderr, "No es pot obrir %s\n", path);
        return -1;
    }
    if (!fgets(line, sizeof(line), fp)) { /* capçalera */
        fclose(fp);
        return -1;
    }
    while (n_tr < MAX_ROWS && fgets(line, sizeof(line), fp)) {
        if (sscanf(line, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f", &a, &b,
                   &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7]) != 10) {
            continue;
        }
        tr[n_tr].frame = a;
        tr[n_tr].obj = b;
        tr[n_tr].x = c[0];
        tr[n_tr].y = c[1];
        tr[n_tr].vx_rel = c[2];
        tr[n_tr].vy_rel = c[3];
        tr[n_tr].vx_abs = c[4];
        tr[n_tr].vy_abs = c[5];
        tr[n_tr].mach = c[6];
        tr[n_tr].temp = c[7];
        if (a + 1 > n_frames) n_frames = a + 1;
        if (b + 1 > n_obj) n_obj = b + 1;
        n_tr++;
    }
    fclose(fp);
    if (n_obj > MAX_OBJ) {
        fprintf(stderr, "Massa objectes (%d > %d)\n", n_obj, MAX_OBJ);
        return -1;
    }
    return 0;
}

/* Transcripció de DataGenerator::getNextFrame sense soroll. */
static void render_frame(int frame, const int *rows, int n_rows)
{
    int k, i, j;
    const float twoSigmaSq = 2.0f * SIGMA * SIGMA;
    const int radius = (int)(SIGMA * 3);

    for (i = 0; i < DRIFT_IMG_W * DRIFT_IMG_H; i++) img[i] = 250.0f;

    for (k = 0; k < n_rows; k++) {
        const truth_t *h = &tr[rows[k]];
        float viewX = h->x, viewY = h->y;
        int x_start = (int)viewX - radius;
        int x_end = (int)viewX + radius;
        int y_start = (int)viewY - radius;
        int y_end = (int)viewY + radius;
        if (x_start < 0) x_start = 0;
        if (y_start < 0) y_start = 0;
        if (x_end > DRIFT_IMG_W) x_end = DRIFT_IMG_W;
        if (y_end > DRIFT_IMG_H) y_end = DRIFT_IMG_H;

        for (i = y_start; i < y_end; i++) {
            for (j = x_start; j < x_end; j++) {
                float dx = (float)j - viewX;
                float dy = (float)i - viewY;
                float distSq = dx * dx + dy * dy;
                float intensity = (h->temp - 250.0f) * expf(-distSq / twoSigmaSq);
                if (intensity > 1.0f) img[i * DRIFT_IMG_W + j] += intensity;
            }
        }
    }
    (void)frame;
}

int main(int argc, char **argv)
{
    int warmup = 15;
    tracker_t T;
    int f, o, i, pass = 1;

    if (argc < 2) {
        fprintf(stderr, "Ús: %s <ref_scenario.csv> [warmup=15]\n", argv[0]);
        return 2;
    }
    if (argc > 2) warmup = atoi(argv[2]);
    if (read_truth(argv[1]) != 0) return 2;

    printf("Escenari: %d objectes, %d frames, warmup=%d\n", n_obj, n_frames, warmup);
    tracker_init(&T, DRIFT_IMG_W, DRIFT_IMG_H);

    for (f = 0; f < n_frames; f++) {
        static int rows[MAX_OBJ];
        int n_rows = 0, n_vis = 0, dropped = 0, n_det, k;

        /* Files de veritat d'aquest frame. */
        for (i = 0; i < n_tr; i++) {
            if (tr[i].frame == f && n_rows < MAX_OBJ) {
                rows[n_rows++] = i;
                if (tr[i].temp > TEMP_THRESH) n_vis++;
            }
        }

        render_frame(f, rows, n_rows);
        n_det = hs_detect(img, TEMP_THRESH, MIN_AREA, det, DRIFT_MAX_DETECTIONS, &dropped);
        if (n_det < 0) {
            printf("frame %d: hs_detect error %d\n", f, n_det);
            return 1;
        }

        if (n_det != n_vis) {
            det_count_mismatch++;
            if (det_count_mismatch <= 5)
                printf("frame %d: deteccions=%d visibles=%d\n", f, n_det, n_vis);
        }

        /* Error de detecció per a cada hotspot visible + detecció a prop d'un de no visible. */
        for (k = 0; k < n_rows; k++) {
            const truth_t *h = &tr[rows[k]];
            int j;
            double best = 1e9;
            for (j = 0; j < n_det; j++) {
                double d = wrap_dist(det[j].x, det[j].y, h->x, h->y);
                if (d < best) best = d;
            }
            if (h->temp > TEMP_THRESH) {
                if (n_det > 0) {
                    int near_edge = h->x < EDGE_PX || h->x > DRIFT_IMG_W - EDGE_PX ||
                                    h->y < EDGE_PX || h->y > DRIFT_IMG_H - EDGE_PX;
                    if (near_edge) {
                        edge_frames++;
                        if (best > edge_err_max) edge_err_max = best;
                    } else {
                        det_err_sum += best;
                        if (best > det_err_max) det_err_max = best;
                        det_err_n++;
                    }
                }
            } else if (best <= MATCH_PX) {
                hidden_det_hits++;
                if (hidden_det_hits <= 5)
                    printf("frame %d: detecció a prop de l'objecte no visible %d (%.2f px)\n",
                           f, h->obj, best);
            }
        }

        tracker_update(&T, det, n_det);

        if (f >= warmup) {
            if (T.n != n_vis) {
                track_count_mismatch++;
                if (track_count_mismatch <= 5)
                    printf("frame %d: tracks=%d visibles=%d\n", f, T.n, n_vis);
            }
            /* Track a prop d'un hotspot no visible = fals positiu. */
            for (k = 0; k < n_rows; k++) {
                const truth_t *h = &tr[rows[k]];
                if (h->temp > TEMP_THRESH) continue;
                for (i = 0; i < T.n; i++) {
                    if (wrap_dist(T.tr[i].px, T.tr[i].py, h->x, h->y) <= MATCH_PX) {
                        hidden_track_hits++;
                        break;
                    }
                }
            }
        }

        /* Per a cada hotspot visible: track associat i error de velocitat. */
        for (k = 0; k < n_rows; k++) {
            const truth_t *h = &tr[rows[k]];
            int best_j = -1;
            double best = 1e9;
            if (h->temp <= TEMP_THRESH) continue;
            o = h->obj;
            for (i = 0; i < T.n; i++) {
                double d = wrap_dist(T.tr[i].px, T.tr[i].py, h->x, h->y);
                if (d < best) { best = d; best_j = i; }
            }
            if (f >= warmup) {
                vis_frames[o]++;
                if (best_j >= 0 && best <= MATCH_PX) tracked_frames[o]++;
            }
            if (f >= VEL_WARMUP && best_j >= 0 && best <= MATCH_PX) {
                double dv = fabs(T.tr[best_j].vx - h->vx_rel) + fabs(T.tr[best_j].vy - h->vy_rel);
                vel_err_sum[o] += dv / 2.0;
                vel_err_n[o]++;
            }
        }
    }

    /* Informe i veredicte. */
    printf("\nDeteccions (interior): error mitjà %.3f px, màxim %.3f px\n",
           det_err_n ? det_err_sum / det_err_n : 0.0, det_err_max);
    printf("Deteccions a la vora (< %d px): %d frames, error màxim %.3f px (informatiu)\n",
           EDGE_PX, edge_frames, edge_err_max);
    printf("Frames amb nombre de deteccions incorrecte: %d\n", det_count_mismatch);
    printf("Frames (post-warmup) amb nombre de tracks incorrecte: %d\n", track_count_mismatch);
    printf("Deteccions a prop d'hotspots no visibles: %d\n", hidden_det_hits);
    printf("Tracks a prop d'hotspots no visibles (post-warmup): %d\n", hidden_track_hits);

    for (o = 0; o < n_obj; o++) {
        int visible = 0, k2;
        double tmax = 0.0;
        for (k2 = 0; k2 < n_tr; k2++) {
            if (tr[k2].obj != o) continue;
            if (tr[k2].temp > TEMP_THRESH) visible = 1;
            if (tr[k2].temp > tmax) tmax = tr[k2].temp;
        }
        if (!visible) {
            printf("objecte %d: NO VISIBLE (T màx %.1f K < %.0f K), no ha de ser tractat\n",
                   o, tmax, TEMP_THRESH);
            continue;
        }
        {
            double rate = vis_frames[o] ? (double)tracked_frames[o] / vis_frames[o] : 0.0;
            double verr = vel_err_n[o] ? vel_err_sum[o] / vel_err_n[o] : -1.0;
            printf("objecte %d: tracat %.1f %% dels frames, error vel. mitjà %.4f px/frame\n",
                   o, 100.0 * rate, verr);
            if (rate < TRACK_RATE) pass = 0;
            if (verr < 0.0 || verr > VEL_TOL) pass = 0;
        }
    }

    if (det_count_mismatch || track_count_mismatch || hidden_det_hits || hidden_track_hits)
        pass = 0;
    if (det_err_max > DET_TOL_PX) pass = 0;

    printf("\nSCENARI TRACKING: %s\n", pass ? "CORRECTE" : "FALLA");
    return pass ? 0 : 1;
}
