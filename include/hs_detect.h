/*
 * hs_detect.h - Detecció de hotspots sobre un frame float32 (C99, memòria estàtica).
 *
 * Substitueix: cv::threshold (THRESH_BINARY), convertTo(CV_8U), cv::findContours
 * (RETR_EXTERNAL), cv::contourArea, cv::moments i cv::minMaxLoc de detector.h.
 *
 * Àrea: per a un component sense forats, cv::contourArea del contorn extern
 * (polígon per centres dels píxels de frontera) es calcula amb el teorema de
 * Pick: A = I + B/2 - 1 = N - B/2 - 1, on N = píxels del component i B = píxels
 * de frontera (amb almenys un veí 4-connectat fora del component o fora de camp).
 * Els forats no es descompten (com a RETR_EXTERNAL, el forat compta dins l'àrea
 * del contorn extern; aquí no s'afegeixen els píxels del forat: pendent).
 * Centroide: de píxels (coincideix amb el de moments poligonals per a formes
 * simètriques; pendent per a formes asimètriques).
 * Connectivitat 8.
 */
#ifndef HS_DETECT_H
#define HS_DETECT_H

#include <stdint.h>

typedef struct {
    float x;        /* centroide x, en píxels (de píxels) */
    float y;        /* centroide y, en píxels (de píxels) */
    float area;     /* àrea equivalent a cv::contourArea del contorn extern */
} hs_point;

/* Codis d'error (negatius). */
#define HS_OK              0
#define HS_ERR_ARG        -1
#define HS_ERR_STACK      -2  /* component més gran que DRIFT_STACK_CAP */

/*
 * Llinda img > thr, etiqueta components 8-connectats i en retorna els centroides
 * amb àrea >= min_area.
 *
 * img       : buffer de DRIFT_IMG_W * DRIFT_IMG_H float32, fila a fila.
 * out       : array de sortida de capacitat max_out.
 * dropped   : (opcional) nombre de components vàlids que no han cabut a out.
 *
 * Retorna el nombre de deteccions (>= 0) o un codi HS_ERR_* negatiu.
 */
int hs_detect(const float *img, float thr, uint32_t min_area,
              hs_point *out, int max_out, int *dropped);

/*
 * Màxim de la finestra quadrada de costat 2*rad centrada a (px,py), amb les
 * mateixes truncacions que maxHotspotTemp original. Si la finestra és buida
 * retorna 0.
 */
float hs_window_max(const float *img, float px, float py, int rad);

#endif /* HS_DETECT_H */
