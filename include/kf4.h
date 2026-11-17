/*
 * kf4.h - Filtre de Kalman lineal de 4 estats (x, y, vx, vy) amb mesura de posició.
 *
 * Replica la semàntica de cv::KalmanFilter(4,2,0) usada a KalmanTracker.cpp:
 *   predict : statePre = F*statePost ; errPre = F*errPost*F' + Q   (retorna statePre)
 *   correct : K = errPre*H'*(H*errPre*H' + R)^-1
 *             statePost = statePre + K*(z - H*statePre)
 *             errPost = (I - K*H)*errPre
 * Només statePost/errPost són l'estat "vigent": predict() no els modifica.
 *
 * F = [1 0 1 0; 0 1 0 1; 0 0 1 0; 0 0 0 1], H = [1 0 0 0; 0 1 0 0],
 * Q = 1e-2*I, R = 1e-1*I, errPost inicial = I (com createFilter original).
 *
 * Operacions matricials: implementades localment aquí, aïllades en kf4.c.
 * Pendent: substituir per LibmCS si la proposta ho requereix.
 */
#ifndef KF4_H
#define KF4_H

typedef struct {
    float statePre[4];
    float statePost[4];
    float errPre[4][4];
    float errPost[4][4];
} kf4_t;

/* Inicialitza amb posició (x,y) i velocitat zero (createFilter original). */
void kf4_init(kf4_t *kf, float x, float y);

/* Predicció. Escriu la posició predita (statePre[0..1]) a out[0..1]. */
void kf4_predict(kf4_t *kf, float out[2]);

/* Correcció amb mesura (zx,zy). Retorna 0 si és correcte, -1 si S és singular. */
int kf4_correct(kf4_t *kf, float zx, float zy);

#endif /* KF4_H */
