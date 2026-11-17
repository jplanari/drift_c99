/*
 * tracker.h - Tracker multi-objectiu amb pools estàtiques (replica KalmanTracker.cpp).
 *
 * Els tracks es mantenen en ordre creixent d'id, igual que el std::map original,
 * perquè l'associació greedy tingui el mateix ordre.
 */
#ifndef TRACKER_H
#define TRACKER_H

#include "drift_config.h"
#include "hs_detect.h"
#include "kf4.h"

typedef struct {
    int id;
    float px, py;          /* posició visual actual (mesura o predicció) */
    float vx, vy;          /* velocitat del filtre (statePost[2..3]) */
    int disappeared;       /* frames consecutius sense mesura */
    int hist_len;          /* nombre d'actualitzacions (equivalent a trail.size()) */
    kf4_t kf;
} track_t;

typedef struct {
    int next_id;
    int w, h;
    int n;                                 /* tracks actius */
    track_t tr[DRIFT_MAX_TRACKS];          /* ordenats per id creixent */
    int dropped_new;                       /* deteccions no registrades per pool plena */
} tracker_t;

void tracker_init(tracker_t *T, int w, int h);

/* Un pas de tracking amb les deteccions del frame. Retorna 0 o -1 si args invàlids. */
int tracker_update(tracker_t *T, const hs_point *meas, int n_meas);

#endif /* TRACKER_H */
