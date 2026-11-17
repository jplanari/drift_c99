/*
 * hs_image.h - Operacions d'imatge de suport (substitueixen cv::normalize NORM_MINMAX).
 */
#ifndef HS_IMAGE_H
#define HS_IMAGE_H

#include <stdint.h>

/*
 * Mapa lineal de [min(src), max(src)] a [0, 255] amb arrodoniment i saturació,
 * com cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8U). Si tots els valors
 * són iguals, dst queda a zero. Retorna 0, o -1 si els arguments són invàlids.
 */
int hs_normalize_minmax_u8(const float *src, uint8_t *dst, int n);

#endif /* HS_IMAGE_H */
