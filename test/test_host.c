/*
 * test_host.c - Proves unitàries de host (gcc). No és per a SPARC.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "drift_config.h"
#include "drift_rng.h"
#include "hs_detect.h"
#include "hs_image.h"
#include "kf4.h"
#include "tracker.h"

static int n_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); n_fail++; } \
    else { printf("ok:   %s\n", msg); } \
} while (0)

#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

static void test_rng(void)
{
    drift_rng_t r;
    double sum = 0.0, sum2 = 0.0;
    int i, n = 200000;
    int in_range = 1;

    drift_rng_seed(&r, 12345u);
    for (i = 0; i < n; i++) {
        float z = drift_rng_randn(&r, 0.0f, 2.0f);
        sum += z;
        sum2 += (double)z * z;
    }
    {
        double mean = sum / n;
        double var = sum2 / n - mean * mean;
        CHECK(fabs(mean) < 0.03, "rng: mitjana de randn(0,2) propera a 0");
        CHECK(fabs(sqrt(var) - 2.0) < 0.03, "rng: desviació de randn(0,2) propera a 2");
    }

    drift_rng_seed(&r, 7u);
    for (i = 0; i < 10000; i++) {
        float u = drift_rng_uniform(&r);
        if (!(u > 0.0f && u < 1.0f)) { in_range = 0; break; }
    }
    CHECK(in_range, "rng: uniforme estrictament dins (0,1)");
}

static void test_normalize(void)
{
    float src[4] = { 10.0f, 20.0f, 30.0f, 40.0f };
    uint8_t dst[4];
    int rc = hs_normalize_minmax_u8(src, dst, 4);
    CHECK(rc == 0 && dst[0] == 0 && dst[3] == 255, "normalize: min->0, max->255");
    CHECK(rc == 0 && dst[1] == 85 && dst[2] == 170, "normalize: valors intermedis lineals");

    {
        float flat[3] = { 5.0f, 5.0f, 5.0f };
        uint8_t d3[3];
        hs_normalize_minmax_u8(flat, d3, 3);
        CHECK(d3[0] == 0 && d3[1] == 0 && d3[2] == 0, "normalize: imatge plana dona zeros");
    }
}

static float img[DRIFT_IMG_H][DRIFT_IMG_W];

static void clear_img(float v)
{
    int y, x;
    for (y = 0; y < DRIFT_IMG_H; y++)
        for (x = 0; x < DRIFT_IMG_W; x++)
            img[y][x] = v;
}

static void test_detector(void)
{
    hs_point out[8];
    int dropped = 0;
    int n;

    /* Blob 5x5 a (2..6, 2..6), temperatura 2000 sobre fons 250.
     * N = 25, frontera B = 16, àrea de contorn = 25 - 8 - 1 = 16. */
    clear_img(250.0f);
    {
        int y, x;
        for (y = 2; y <= 6; y++)
            for (x = 2; x <= 6; x++)
                img[y][x] = 2000.0f;
    }
    n = hs_detect(&img[0][0], 1000.0f, 10, out, 8, &dropped);
    CHECK(n == 1, "detector: un blob 5x5 dona una detecció");
    CHECK(n == 1 && NEAR(out[0].x, 4.0f, 1e-5) && NEAR(out[0].y, 4.0f, 1e-5),
          "detector: centroide del blob 5x5 és (4,4)");
    CHECK(n == 1 && NEAR(out[0].area, 16.0f, 1e-5), "detector: àrea de contorn del blob 5x5 és 16");

    /* Disc de radi 2 (13 px): àrea de contorn 8 < 10, descartat (com OpenCV). */
    clear_img(250.0f);
    {
        int y, x;
        for (y = 0; y < 5; y++)
            for (x = 0; x < 5; x++) {
                int dx = x - 2, dy = y - 2;
                if (dx * dx + dy * dy <= 4) img[50 + y][50 + x] = 2000.0f;
            }
    }
    n = hs_detect(&img[0][0], 1000.0f, 10, out, 8, &dropped);
    CHECK(n == 0, "detector: disc de 13 px (àrea de contorn 8) es descarta com a OpenCV");

    /* Dos píxels en diagonal: connectivitat 8 => un sol component. */
    clear_img(250.0f);
    img[10][10] = 2000.0f;
    img[11][11] = 2000.0f;
    n = hs_detect(&img[0][0], 1000.0f, 0, out, 8, &dropped);
    CHECK(n == 1, "detector: píxels en diagonal formen un component (8-connectivitat)");

    /* Píxel aïllat de 1 px amb min_area = 10: descartat. */
    clear_img(250.0f);
    img[20][20] = 2000.0f;
    n = hs_detect(&img[0][0], 1000.0f, 10, out, 8, &dropped);
    CHECK(n == 0, "detector: component més petit que min_area es descarta");

    /* Llinda estricta: valor igual al llindar no és detecció (THRESH_BINARY). */
    clear_img(1000.0f);
    n = hs_detect(&img[0][0], 1000.0f, 1, out, 8, &dropped);
    CHECK(n == 0, "detector: valor == llindar no es detecta (estricte >)");

    /* Overflow de max_out: compta dropped. */
    clear_img(250.0f);
    img[5][5] = 2000.0f;
    img[5][50] = 2000.0f;
    img[50][5] = 2000.0f;
    n = hs_detect(&img[0][0], 1000.0f, 0, out, 2, &dropped);
    CHECK(n == 2 && dropped == 1, "detector: max_out limita sortida i compta dropped");
}

static void test_window_max(void)
{
    float m;
    clear_img(250.0f);
    img[100][100] = 1234.0f;
    m = hs_window_max(&img[0][0], 105.0f, 100.0f, 15);
    CHECK(NEAR(m, 1234.0f, 1e-3), "window_max: troba el màxim dins la finestra");

    m = hs_window_max(&img[0][0], 100.0f, 100.0f, 15);
    CHECK(NEAR(m, 1234.0f, 1e-3), "window_max: centrada al píxel calent");

    m = hs_window_max(&img[0][0], 300.0f, 300.0f, 2);
    CHECK(NEAR(m, 250.0f, 1e-3), "window_max: finestra lluny del píxel calent");

    m = hs_window_max(&img[0][0], -100.0f, -100.0f, 3);
    CHECK(m == 0.0f, "window_max: finestra totalment fora de camp retorna 0");
}

static void test_kf_matches_closed_form(void)
{
    kf4_t kf;
    float out[2];
    float expect_pre_x, expect_pre_y;

    /* Inicial (10,20), v=0. Predicció amb v=0 => statePre = (10,20). */
    kf4_init(&kf, 10.0f, 20.0f);
    kf4_predict(&kf, out);
    CHECK(NEAR(out[0], 10.0f, 1e-6) && NEAR(out[1], 20.0f, 1e-6),
          "kf: predicció inicial amb v=0 és la posició inicial");

    /* Després de la primera correcció, la velocitat ha de ser no nul·la i en la
     * direcció del moviment. */
    kf4_correct(&kf, 11.0f, 21.0f);
    CHECK(kf.statePost[0] > 10.0f && kf.statePost[0] < 11.0f,
          "kf: correcció cap a la mesura (x) sense superar-la");
    CHECK(kf.statePost[2] > 0.0f, "kf: velocitat vx positiva després de moviment +1");

    /* Predicció següent: statePre = F*statePost. */
    expect_pre_x = kf.statePost[0] + kf.statePost[2];
    expect_pre_y = kf.statePost[1] + kf.statePost[3];
    kf4_predict(&kf, out);
    CHECK(NEAR(out[0], expect_pre_x, 1e-5) && NEAR(out[1], expect_pre_y, 1e-5),
          "kf: statePre = F * statePost");
}

static void test_kf_coasting_semantics(void)
{
    /* Replica OpenCV: predict() copia statePre a statePost, així que sense correct
     * el track avança amb la seva velocitat a cada frame. */
    kf4_t kf;
    float a[2], b[2], c[2];
    float vx;

    kf4_init(&kf, 50.0f, 50.0f);
    kf4_predict(&kf, a);
    kf4_correct(&kf, 52.0f, 50.0f);
    kf4_predict(&kf, a);
    vx = kf.statePost[2];
    kf4_predict(&kf, b);
    kf4_predict(&kf, c);
    CHECK(NEAR(b[0] - a[0], vx, 1e-5) && NEAR(c[0] - b[0], vx, 1e-5),
          "kf: sense correcció, el track avança amb la velocitat (semàntica OpenCV)");
}

static void test_tracker_lifecycle(void)
{
    tracker_t T;
    hs_point det;
    int f;
    int id0;

    tracker_init(&T, 640, 480);

    /* Detecció que es mou +2 px/frame en x. */
    for (f = 0; f < 12; f++) {
        det.x = 100.0f + 2.0f * (float)f;
        det.y = 200.0f;
        det.area = 20;
        tracker_update(&T, &det, 1);
    }
    CHECK(T.n == 1, "tracker: una detecció contínua manté un sol track");
    id0 = T.tr[0].id;
    CHECK(T.tr[0].vx > 1.0f && T.tr[0].vx < 3.0f,
          "tracker: velocitat vx convergeix cap a +2 px/frame");

    /* 15 frames sense mesura: el track encara hi és (disappeared <= 15). */
    for (f = 0; f < DRIFT_MAX_DISAPPEARED; f++) {
        tracker_update(&T, NULL, 0);
    }
    CHECK(T.n == 1, "tracker: track sobreviu 15 frames sense mesura");

    /* 16è frame sense mesura: baixa. */
    tracker_update(&T, NULL, 0);
    CHECK(T.n == 0, "tracker: track es dona de baixa a la 16a absència");

    /* Nou objecte rep un id nou, creixent. */
    det.x = 300.0f; det.y = 300.0f; det.area = 20;
    tracker_update(&T, &det, 1);
    CHECK(T.n == 1 && T.tr[0].id > id0, "tracker: id nou creixent després de baixa");
}

static void test_tracker_pool_full(void)
{
    tracker_t T;
    hs_point dets[DRIFT_MAX_DETECTIONS];
    int i;

    tracker_init(&T, 640, 480);
    for (i = 0; i < DRIFT_MAX_DETECTIONS; i++) {
        dets[i].x = (float)(10 + 15 * i);
        dets[i].y = 10.0f;
        dets[i].area = 20;
    }
    tracker_update(&T, dets, DRIFT_MAX_DETECTIONS);
    CHECK(T.n == DRIFT_MAX_TRACKS, "tracker: pool plena limita tracks actius");
    CHECK(T.dropped_new == DRIFT_MAX_DETECTIONS - DRIFT_MAX_TRACKS,
          "tracker: deteccions sobrants es comptabilitzen com a dropped");
}

int main(void)
{
    test_rng();
    test_normalize();
    test_detector();
    test_window_max();
    test_kf_matches_closed_form();
    test_kf_coasting_semantics();
    test_tracker_lifecycle();
    test_tracker_pool_full();

    if (n_fail == 0) {
        printf("\nTOT CORRECTE\n");
        return 0;
    }
    printf("\n%d PROVES FALLIDES\n", n_fail);
    return 1;
}
