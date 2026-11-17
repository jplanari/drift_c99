// golden_gen.cpp - Genera el golden master amb el codi C++ original (OpenCV).
//
// Escriu, a <outdir>:
//   frame_NNN.bin  : frames float32 little-endian, W*H, fila a fila (entrada per al port C99)
//   ref_det.csv    : frame,x,y         deteccions de detectHotspots (ordre original)
//   ref_trk.csv    : frame,id,x,y,vx,vy estat de tracker.trackedHotspots després de update()
//
// Compilació (Ubuntu, OpenCV 4):
//   g++ -std=c++11 -O2 -I$ORIG/src golden/golden_gen.cpp $ORIG/src/*.cpp \
//       -o golden_gen $(pkg-config --cflags --libs opencv4)
//   mkdir -p golden_out && ./golden_gen golden_out
//
// ORIG és el directori src del repositori postprocir_cpp.
//
// Escenari determinista (no usa cv::randn): quatre discs calents que es mouen,
// un que travessa la vora (wrap-around), un amb buit de 25 frames (baixa del
// tracker) i un amb buit de 5 frames (coasting), més píxels calents d'àrea 1
// que han de ser descartats per tinyBlobSize.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "Detector.h"
#include "KalmanTracker.h"

static const int W = 640;
static const int H = 480;
static const int NF = 60;
static const double TEMP_THRESH = 1000.0;
static const int TINY = 10;

struct Disk {
    double x0, y0, vx, vy;   // px, px/frame
    double temp;             // K
    double r;                // px
    int gap_from, gap_to;    // frames sense disc (-1 si no n'hi ha)
};

static double wrap(double v, double m) {
    double r = std::fmod(v, m);
    if (r < 0) r += m;
    return r;
}

int main(int argc, char** argv) {
    const std::string outDir = (argc > 1) ? argv[1] : "golden_out";

    const Disk disks[] = {
        {  50.0, 100.0,  6.0,  0.0, 1500.0, 3.0, -1, -1},  // trajectòria recta
        { 500.0, 300.0, -3.0,  2.0, 1300.0, 4.0, -1, -1},  // travessa les vores
        { 300.0,  50.0,  0.0,  4.0, 1100.0, 2.0, 20, 44},  // buit de 25 frames -> baixa
        { 100.0, 400.0,  1.0, -1.0, 1400.0, 3.0, 30, 34},  // buit de 5 frames -> coasting
    };
    const int nDisks = static_cast<int>(sizeof(disks) / sizeof(disks[0]));

    const std::string detPath = outDir + "/ref_det.csv";
    const std::string trkPath = outDir + "/ref_trk.csv";
    FILE* fdet = std::fopen(detPath.c_str(), "w");
    FILE* ftrk = std::fopen(trkPath.c_str(), "w");
    if (!fdet || !ftrk) {
        std::fprintf(stderr, "No es pot escriure a %s (existeix el directori?)\n", outDir.c_str());
        return 1;
    }

    KalmanTracker tracker(W, H);
    cv::Mat img(H, W, CV_32F);
    uint32_t lcg = 12345u;

    for (int f = 0; f < NF; f++) {
        // Fons amb soroll determinista, sempre per sota del llindar.
        for (int y = 0; y < H; y++) {
            float* row = img.ptr<float>(y);
            for (int x = 0; x < W; x++) {
                lcg = lcg * 1664525u + 1013904223u;
                double u = static_cast<double>(lcg >> 8) / 16777216.0;
                row[x] = static_cast<float>(250.0 + 10.0 * (u - 0.5));
            }
        }

        // Discs calents.
        for (int d = 0; d < nDisks; d++) {
            const Disk& dk = disks[d];
            if (dk.gap_from >= 0 && f >= dk.gap_from && f <= dk.gap_to) continue;
            const double cx = wrap(dk.x0 + dk.vx * f, W);
            const double cy = wrap(dk.y0 + dk.vy * f, H);
            const int y0 = std::max(0, static_cast<int>(std::floor(cy - dk.r)) - 1);
            const int y1 = std::min(H - 1, static_cast<int>(std::ceil(cy + dk.r)) + 1);
            const int x0 = std::max(0, static_cast<int>(std::floor(cx - dk.r)) - 1);
            const int x1 = std::min(W - 1, static_cast<int>(std::ceil(cx + dk.r)) + 1);
            for (int y = y0; y <= y1; y++) {
                for (int x = x0; x <= x1; x++) {
                    const double ddx = x - cx, ddy = y - cy;
                    if (ddx * ddx + ddy * ddy <= dk.r * dk.r) {
                        img.at<float>(y, x) = static_cast<float>(dk.temp);
                    }
                }
            }
        }

        // Píxel calent aïllat (àrea 1 < TINY): ha de ser descartat.
        img.at<float>(H - 5, (10 + 7 * f) % W) = 1050.0f;

        // Escriu el frame per al port C99.
        char name[512];
        std::snprintf(name, sizeof(name), "%s/frame_%03d.bin", outDir.c_str(), f);
        FILE* fb = std::fopen(name, "wb");
        if (!fb) { std::fprintf(stderr, "No es pot escriure %s\n", name); return 1; }
        std::fwrite(img.ptr<float>(0), sizeof(float), static_cast<size_t>(W) * H, fb);
        std::fclose(fb);

        // Pipeline original.
        std::vector<cv::Point2f> meas = detectHotspots(f, img, TEMP_THRESH, TINY);
        for (const auto& p : meas) {
            std::fprintf(fdet, "%d,%.9g,%.9g\n", f, p.x, p.y);
        }

        tracker.update(meas);
        for (const auto& kv : tracker.trackedHotspots) {
            const Hotspot& hs = kv.second;
            std::fprintf(ftrk, "%d,%d,%.9g,%.9g,%.9g,%.9g\n",
                         f, kv.first, hs.position.x, hs.position.y,
                         hs.velocity.x, hs.velocity.y);
        }
    }

    std::fclose(fdet);
    std::fclose(ftrk);
    std::printf("OK: %d frames a %s\n", NF, outDir.c_str());
    return 0;
}
