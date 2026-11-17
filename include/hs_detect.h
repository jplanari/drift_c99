/*
 * hs_detect.h - Detecció de hotspots sobre un frame float32 (C99, memòria estàtica).
 *
 * Substitueix: cv::threshold (THRESH_BINARY), convertTo(CV_8U), cv::findContours
 * (RETR_EXTERNAL), cv::contourArea, cv::moments i cv::minMaxLoc de detector.h.
 *
 * Diferències conegudes respecte OpenCV (a validar amb el golden master):
 *   - L'àrea és el recompte de píxels del component (inclou forats), no l'àrea
 *     poligonal del contorn extern.
 *   - El centroide és el de píxels, no el dels moments poligonals del contorn.
 *   - Connectivitat 8 (a verificar contra findContours).
 */
#ifndef HS_DETECT_H
#define HS_DETECT_H

#include <stdint.h>

typedef struct {
    float x;        /* centroide x, en píxels */
    float y;        /* centroide y, en píxels */
    uint32_t area;  /* nombre de píxels del component */
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
