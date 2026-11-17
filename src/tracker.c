/*
 * tracker.c - Vegeu tracker.h. Ordre de passos idèntic a KalmanTracker::update.
 */
#include "tracker.h"
#include "drift_util.h"

void tracker_init(tracker_t *T, int w, int h)
{
    drift_zero(T, sizeof(*T));
    T->w = w;
    T->h = h;
}

static void register_track(tracker_t *T, float x, float y)
{
    track_t *t;

    if (T->n >= DRIFT_MAX_TRACKS) {
        T->dropped_new++;
        return;
    }
    t = &T->tr[T->n++];
    t->id = T->next_id++;
    t->px = x;
    t->py = y;
    t->vx = 0.0f;
    t->vy = 0.0f;
    t->disappeared = 0;
    t->hist_len = 0;
    kf4_init(&t->kf, x, y);
}

int tracker_update(tracker_t *T, const hs_point *meas, int n_meas)
{
    int i, m;
    int n_before;
    float pred[DRIFT_MAX_TRACKS][2];
    uint8_t used_t[DRIFT_MAX_TRACKS];
    uint8_t used_m[DRIFT_MAX_DETECTIONS];
    double max_dist;
    double max_d2;
    int half_w;   /* divisió entera, com a width/2 original */
    int half_h;

    if (T == NULL || (n_meas > 0 && meas == NULL) || n_meas < 0 || n_meas > DRIFT_MAX_DETECTIONS) {
        return -1;
    }
    half_w = T->w / 2;
    half_h = T->h / 2;

    /* 1. PREDICT */
    n_before = T->n;
    for (i = 0; i < n_before; i++) {
        kf4_predict(&T->tr[i].kf, pred[i]);
        used_t[i] = 0;
    }
    for (m = 0; m < n_meas; m++) {
        used_m[m] = 0;
    }

    /* 2. ASSOCIACIÓ greedy. Distància calculada amb coordenades truncades a int,
     *    com periodicDist(cv::Point, cv::Point) original. */
    max_dist = (double)(T->w > T->h ? T->w : T->h) * DRIFT_DIST_RATIO;
    max_d2 = max_dist * max_dist;

    for (i = 0; i < n_before; i++) {
        int best = -1;
        double min_d2 = 1e18;
        int px = (int)pred[i][0];
        int py = (int)pred[i][1];

        for (m = 0; m < n_meas; m++) {
            double dx, dy, d2;
            if (used_m[m]) continue;
            dx = (double)(px - (int)meas[m].x);
            dy = (double)(py - (int)meas[m].y);
            d2 = dx * dx + dy * dy;
            if (d2 < min_d2 && d2 < max_d2) {
                min_d2 = d2;
                best = m;
            }
        }

        if (best != -1) {
            track_t *t = &T->tr[i];
            float mx = meas[best].x;
            float my = meas[best].y;
            float dx = mx - pred[i][0];
            float dy = my - pred[i][1];

            /* Desplaçament per wrap-around, només per a la correcció del filtre. */
            if (dx > (float)half_w)       mx -= (float)T->w;
            else if (dx < -(float)half_w) mx += (float)T->w;
            if (dy > (float)half_h)       my -= (float)T->h;
            else if (dy < -(float)half_h) my += (float)T->h;

            kf4_correct(&t->kf, mx, my);
            t->px = meas[best].x;
            t->py = meas[best].y;
            t->hist_len++;
            t->disappeared = 0;

            used_t[i] = 1;
            used_m[best] = 1;
        }
    }

    /* 3. TRACKS NO EMPARELLATS: coasting o baixa. Compactació que preserva l'ordre. */
    {
        int w = 0;
        for (i = 0; i < T->n; i++) {
            track_t *t = &T->tr[i];
            if (i < n_before && !used_t[i]) {
                t->disappeared++;
                if (t->disappeared > DRIFT_MAX_DISAPPEARED) {
                    continue;   /* deregistre: no es copia */
                }
                /* Coasting: posició predita (com el codi original). */
                t->px = pred[i][0];
                t->py = pred[i][1];
                t->hist_len++;
            }
            if (w != i) {
                drift_copy(&T->tr[w], t, sizeof(track_t));
            }
            w++;
        }
        T->n = w;
    }

    /* 4. NOVES DETECCIONS */
    for (m = 0; m < n_meas; m++) {
        if (!used_m[m]) {
            register_track(T, meas[m].x, meas[m].y);
        }
    }

    /* 5. VELOCITAT del filtre (statePost[2..3]) */
    for (i = 0; i < T->n; i++) {
        T->tr[i].vx = T->tr[i].kf.statePost[2];
        T->tr[i].vy = T->tr[i].kf.statePost[3];
    }

    return 0;
}
