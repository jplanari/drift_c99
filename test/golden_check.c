/*
 * golden_check.c - Compara el port C99 amb el golden master generat per golden_gen.cpp.
 *
 * Ús:  golden_check <dir_golden> [tol_det_px=0.5] [tol_trk=1e-3]
 *
 * Dues etapes, per separar errors:
 *   1. Detecció: hs_detect() sobre cada frame vs ref_det.csv (ordenat per y,x).
 *   2. Tracker: tracker_update() amb les deteccions de referència (ordre original)
 *      vs ref_trk.csv. Compara id, posició i velocitat de cada track actiu.
 *
 * Només host (usa stdio/stdlib); no es compila per al vol.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drift_config.h"
#include "hs_detect.h"
#include "tracker.h"

#define NF        60
#define MAX_REF   20000
#define MAX_OUT   DRIFT_MAX_DETECTIONS

typedef struct { int f; float x, y; } rdet_t;
typedef struct { int f, id; float x, y, vx, vy; } rtrk_t;

static rdet_t rdet[MAX_REF];
static int n_rdet = 0;
static rtrk_t rtrk[MAX_REF];
static int n_rtrk = 0;
static float img[DRIFT_IMG_H * DRIFT_IMG_W];

static double fabs_d(double v) { return v < 0 ? -v : v; }

static int read_ref(const char *dir)
{
    char path[512];
    FILE *fp;
    int a, b;
    float c, d, e, g;

    snprintf(path, sizeof(path), "%s/ref_det.csv", dir);
    fp = fopen(path, "r");
    if (!fp) { fprintf(stderr, "No es pot obrir %s\n", path); return -1; }
    while (n_rdet < MAX_REF && fscanf(fp, "%d,%f,%f", &a, &c, &d) == 3) {
        rdet[n_rdet].f = a; rdet[n_rdet].x = c; rdet[n_rdet].y = d;
        n_rdet++;
    }
    fclose(fp);

    snprintf(path, sizeof(path), "%s/ref_trk.csv", dir);
    fp = fopen(path, "r");
    if (!fp) { fprintf(stderr, "No es pot obrir %s\n", path); return -1; }
    while (n_rtrk < MAX_REF && fscanf(fp, "%d,%d,%f,%f,%f,%f", &a, &b, &c, &d, &e, &g) == 6) {
        rtrk[n_rtrk].f = a; rtrk[n_rtrk].id = b;
        rtrk[n_rtrk].x = c; rtrk[n_rtrk].y = d;
        rtrk[n_rtrk].vx = e; rtrk[n_rtrk].vy = g;
        n_rtrk++;
    }
    fclose(fp);
    return 0;
}

static int load_frame(const char *dir, int f)
{
    char path[512];
    FILE *fp;
    size_t got;
    snprintf(path, sizeof(path), "%s/frame_%03d.bin", dir, f);
    fp = fopen(path, "rb");
    if (!fp) { fprintf(stderr, "No es pot obrir %s\n", path); return -1; }
    got = fread(img, sizeof(float), (size_t)DRIFT_IMG_W * DRIFT_IMG_H, fp);
    fclose(fp);
    return (got == (size_t)DRIFT_IMG_W * DRIFT_IMG_H) ? 0 : -1;
}

static int cmp_hs(const void *a, const void *b)
{
    const hs_point *p = (const hs_point *)a, *q = (const hs_point *)b;
    if (p->y < q->y) return -1;
    if (p->y > q->y) return 1;
    if (p->x < q->x) return -1;
    if (p->x > q->x) return 1;
    return 0;
}

static int cmp_rdet(const void *a, const void *b)
{
    const rdet_t *p = (const rdet_t *)a, *q = (const rdet_t *)b;
    if (p->y < q->y) return -1;
    if (p->y > q->y) return 1;
    if (p->x < q->x) return -1;
    if (p->x > q->x) return 1;
    return 0;
}

int main(int argc, char **argv)
{
    const char *dir;
    double tol_det = 0.5, tol_trk = 1e-3;
    int f, fails = 0;
    double max_det = 0.0, max_trk_pos = 0.0, max_trk_vel = 0.0;
    int det_count_mismatch = 0;
    int total_ref_tracks = 0, matched_tracks = 0;
    static hs_point out[MAX_OUT];
    tracker_t T;

    if (argc < 2) {
        fprintf(stderr, "Ús: %s <dir_golden> [tol_det_px] [tol_trk]\n", argv[0]);
        return 2;
    }
    dir = argv[1];
    if (argc > 2) tol_det = atof(argv[2]);
    if (argc > 3) tol_trk = atof(argv[3]);

    if (read_ref(dir) != 0) return 2;
    printf("Referència: %d deteccions, %d entrades de track\n", n_rdet, n_rtrk);

    tracker_init(&T, DRIFT_IMG_W, DRIFT_IMG_H);

    for (f = 0; f < NF; f++) {
        int i, n_ref = 0, n_ours, dropped = 0, k;
        static rdet_t frame_ref[MAX_OUT];
        int n_frame_ref = 0;

        /* Deteccions de referència d'aquest frame. */
        for (i = 0; i < n_rdet; i++) {
            if (rdet[i].f == f && n_frame_ref < MAX_OUT) {
                frame_ref[n_frame_ref++] = rdet[i];
            }
        }
        n_ref = n_frame_ref;

        if (load_frame(dir, f) != 0) return 2;

        /* Etapa 1: detecció. */
        n_ours = hs_detect(img, 1000.0f, 10u, out, MAX_OUT, &dropped);
        if (n_ours < 0) {
            printf("frame %d: hs_detect error %d\n", f, n_ours);
            fails++;
            continue;
        }
        if (n_ours != n_ref) {
            printf("frame %d: deteccions port=%d ref=%d\n", f, n_ours, n_ref);
            det_count_mismatch++;
            fails++;
        } else if (n_ref > 0) {
            qsort(out, (size_t)n_ours, sizeof(hs_point), cmp_hs);
            qsort(frame_ref, (size_t)n_ref, sizeof(rdet_t), cmp_rdet);
            for (k = 0; k < n_ref; k++) {
                double dx = fabs_d(out[k].x - frame_ref[k].x);
                double dy = fabs_d(out[k].y - frame_ref[k].y);
                double d = (dx > dy) ? dx : dy;
                if (d > max_det) max_det = d;
                if (d > tol_det) {
                    printf("frame %d det %d: port (%.4f,%.4f) ref (%.4f,%.4f)\n",
                           f, k, out[k].x, out[k].y, frame_ref[k].x, frame_ref[k].y);
                    fails++;
                }
            }
        }

        /* Etapa 2: tracker amb les deteccions de referència, en el seu ordre. */
        {
            static hs_point ref_in[MAX_OUT];
            for (k = 0; k < n_ref; k++) {
                ref_in[k].x = frame_ref[k].x;
                ref_in[k].y = frame_ref[k].y;
                ref_in[k].area = 0u;
            }
            tracker_update(&T, ref_in, n_ref);
        }

        /* Comprova els tracks de referència d'aquest frame. */
        {
            int n_ref_trk = 0;
            for (i = 0; i < n_rtrk; i++) {
                const rtrk_t *r = &rtrk[i];
                int j, found = -1;
                if (r->f != f) continue;
                n_ref_trk++;
                total_ref_tracks++;
                for (j = 0; j < T.n; j++) {
                    if (T.tr[j].id == r->id) { found = j; break; }
                }
                if (found < 0) {
                    printf("frame %d: track id %d absent al port\n", f, r->id);
                    fails++;
                    continue;
                }
                matched_tracks++;
                {
                    const track_t *t = &T.tr[found];
                    double dp = fabs_d(t->px - r->x), dp2 = fabs_d(t->py - r->y);
                    double dv = fabs_d(t->vx - r->vx), dv2 = fabs_d(t->vy - r->vy);
                    if (dp2 > dp) dp = dp2;
                    if (dv2 > dv) dv = dv2;
                    if (dp > max_trk_pos) max_trk_pos = dp;
                    if (dv > max_trk_vel) max_trk_vel = dv;
                    if (dp > tol_trk || dv > tol_trk) {
                        printf("frame %d id %d: pos port (%.6f,%.6f) ref (%.6f,%.6f) "
                               "vel port (%.6f,%.6f) ref (%.6f,%.6f)\n",
                               f, r->id, t->px, t->py, r->x, r->y,
                               t->vx, t->vy, r->vx, r->vy);
                        fails++;
                    }
                }
            }
            if (n_ref_trk != T.n) {
                printf("frame %d: nombre de tracks port=%d ref=%d\n", f, T.n, n_ref_trk);
                fails++;
            }
        }
    }

    printf("\nDeteccions: màxima desviació %.6f px (tol %.3f), frames amb recompte diferent: %d\n",
           max_det, tol_det, det_count_mismatch);
    printf("Tracker:    tracks de referència %d, emparellats %d, "
           "màx desv. posició %.3e, màx desv. velocitat %.3e (tol %.1e)\n",
           total_ref_tracks, matched_tracks, max_trk_pos, max_trk_vel, tol_trk);

    if (fails == 0) {
        printf("GOLDEN: CORRECTE\n");
        return 0;
    }
    printf("GOLDEN: %d DISCREPÀNCIES\n", fails);
    return 1;
}
