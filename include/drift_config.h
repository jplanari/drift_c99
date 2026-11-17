/*
 * drift_config.h - Configuració en temps de compilació del mòdul de detecció i tracking.
 *
 * Totes les mides són estàtiques (sense malloc/free). Els valors per defecte
 * coincideixen amb el codi C++ original (main.cpp: W=640, H=480). El format
 * definitiu de TIRI (1024x768, 8-14 um) encara està pendent d'ESA; es pot
 * sobreescriure amb -DDRIFT_IMG_W=1024 -DDRIFT_IMG_H=768.
 */
#ifndef DRIFT_CONFIG_H
#define DRIFT_CONFIG_H

#ifndef DRIFT_IMG_W
#define DRIFT_IMG_W 640
#endif

#ifndef DRIFT_IMG_H
#define DRIFT_IMG_H 480
#endif

/* Nombre màxim de deteccions per frame (pool d'entrada del tracker). */
#ifndef DRIFT_MAX_DETECTIONS
#define DRIFT_MAX_DETECTIONS 64
#endif

/* Nombre màxim de tracks simultanis (pool estàtica). */
#ifndef DRIFT_MAX_TRACKS
#define DRIFT_MAX_TRACKS 32
#endif

/* Frames sense mesura abans de donar de baixa un track (MAX_DISAPPEARED original). */
#define DRIFT_MAX_DISAPPEARED 15

/* Ratio de distància d'associació (DIST_THRESHOLD_RATIO original). */
#define DRIFT_DIST_RATIO 0.1

/* Capacitat de la pila de flood-fill. Un component més gran que això s'aborta. */
#ifndef DRIFT_STACK_CAP
#define DRIFT_STACK_CAP 65536
#endif

#endif /* DRIFT_CONFIG_H */
