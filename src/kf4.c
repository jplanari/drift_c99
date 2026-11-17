/*
 * kf4.c - Vegeu kf4.h.
 */
#include "kf4.h"
#include "drift_util.h"

#include <math.h>

#define KF_Q_DIAG 1e-2f
#define KF_R_DIAG 1e-1f

void kf4_init(kf4_t *kf, float x, float y)
{
    int i, j;

    drift_zero(kf, sizeof(*kf));
    kf->statePre[0] = x;
    kf->statePre[1] = y;
    kf->statePost[0] = x;
    kf->statePost[1] = y;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            kf->errPost[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void kf4_predict(kf4_t *kf, float out[2])
{
    float fp[4][4];   /* F * errPost */
    int i, j;

    /* statePre = F * statePost */
    kf->statePre[0] = kf->statePost[0] + kf->statePost[2];
    kf->statePre[1] = kf->statePost[1] + kf->statePost[3];
    kf->statePre[2] = kf->statePost[2];
    kf->statePre[3] = kf->statePost[3];

    /* fp = F * errPost (suma de files) */
    for (j = 0; j < 4; j++) {
        fp[0][j] = kf->errPost[0][j] + kf->errPost[2][j];
        fp[1][j] = kf->errPost[1][j] + kf->errPost[3][j];
        fp[2][j] = kf->errPost[2][j];
        fp[3][j] = kf->errPost[3][j];
    }

    /* errPre = fp * F' + Q (suma de columnes) */
    for (i = 0; i < 4; i++) {
        kf->errPre[i][0] = fp[i][0] + fp[i][2];
        kf->errPre[i][1] = fp[i][1] + fp[i][3];
        kf->errPre[i][2] = fp[i][2];
        kf->errPre[i][3] = fp[i][3];
    }
    for (i = 0; i < 4; i++) {
        kf->errPre[i][i] += KF_Q_DIAG;
    }

    /* cv::KalmanFilter::predict() acaba amb statePost = statePre i errorCovPost = errorCovPre.
     * Així, un track sense correcció avança amb la seva velocitat a cada frame. */
    for (i = 0; i < 4; i++) {
        kf->statePost[i] = kf->statePre[i];
    }
    drift_copy(kf->errPost, kf->errPre, sizeof(kf->errPre));

    out[0] = kf->statePre[0];
    out[1] = kf->statePre[1];
}

int kf4_correct(kf4_t *kf, float zx, float zy)
{
    float s00, s01, s10, s11, det;
    float inv00, inv01, inv10, inv11;
    float k[4][2];
    float yx, yy;
    float a[4][4];   /* I - K*H */
    float ep[4][4];  /* a * errPre */
    int i, j, m;

    /* S = H*errPre*H' + R (2x2) */
    s00 = kf->errPre[0][0] + KF_R_DIAG;
    s01 = kf->errPre[0][1];
    s10 = kf->errPre[1][0];
    s11 = kf->errPre[1][1] + KF_R_DIAG;

    det = s00 * s11 - s01 * s10;
    if (fabsf(det) < 1e-30f) {
        return -1;
    }
    inv00 =  s11 / det;
    inv01 = -s01 / det;
    inv10 = -s10 / det;
    inv11 =  s00 / det;

    /* K = errPre*H'*S^-1 (4x2). errPre*H' = primeres dues columnes d'errPre. */
    for (i = 0; i < 4; i++) {
        k[i][0] = kf->errPre[i][0] * inv00 + kf->errPre[i][1] * inv10;
        k[i][1] = kf->errPre[i][0] * inv01 + kf->errPre[i][1] * inv11;
    }

    /* Innovació y = z - H*statePre */
    yx = zx - kf->statePre[0];
    yy = zy - kf->statePre[1];

    /* statePost = statePre + K*y */
    for (i = 0; i < 4; i++) {
        kf->statePost[i] = kf->statePre[i] + k[i][0] * yx + k[i][1] * yy;
    }

    /* A = I - K*H, on K*H només té les dues primeres columnes no nul·les */
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            float kh = 0.0f;
            if (j == 0) kh = k[i][0];
            else if (j == 1) kh = k[i][1];
            a[i][j] = ((i == j) ? 1.0f : 0.0f) - kh;
        }
    }

    /* errPost = A * errPre */
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            float acc = 0.0f;
            for (m = 0; m < 4; m++) {
                acc += a[i][m] * kf->errPre[m][j];
            }
            ep[i][j] = acc;
        }
    }
    drift_copy(kf->errPost, ep, sizeof(ep));

    return 0;
}
