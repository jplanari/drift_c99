// scenario_orig.cpp - Runs the ORIGINAL postProcIR_Cpp physics (DataGenerator, Satellite,
// loadScenario) on a scenario CSV and writes the same CSV as scenario_ref.cpp, so the two
// can be compared with compare_scenario.sh.
//
// This needs OpenCV (DataGenerator.h includes it) and the original sources. It is a TEST
// build only: the `private` -> `public` define below gives read access to the hotspot state,
// which the original class does not expose. Do not link this into the firmware.
//
// Build (on a machine with OpenCV 4):
//   ORIG=/path/to/postprocir_cpp
//   g++ -std=c++11 -O2 -I$ORIG/src golden/scenario_orig.cpp \
//       $ORIG/src/DataGenerator.cpp $ORIG/src/Satellite.cpp \
//       $(pkg-config --cflags --libs opencv4) -o scenario_orig
// Run:
//   ./scenario_orig scenario.csv out.csv [frames=300] [W=640] [H=480] [FPS=30]
//
// Scenarios with acc=1 are rejected by the loader's RNG path in the original, so use acc=0.

// Standard and OpenCV headers first, so the access hack does not touch them.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#define private public
#include "DataGenerator.h"
#include "Satellite.h"
#include "ScenarioLoader.h"
#undef private

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " scenario.csv out.csv [frames=300] [W=640] [H=480] [FPS=30]\n";
        return 1;
    }
    std::string scenarioFile = argv[1];
    std::string outFile = argv[2];
    int frames = argc > 3 ? std::atoi(argv[3]) : 300;
    int W = argc > 4 ? std::atoi(argv[4]) : 640;
    int H = argc > 5 ? std::atoi(argv[5]) : 480;
    int FPS = argc > 6 ? std::atoi(argv[6]) : 30;

    // Same setup as main.cpp (the original prints some diagnostics to stdout; ignored here).
    Satellite satellite(500.0, 30.0, W, H);
    float orbitSpeedKps = satellite.getOrbitalVelocityKmS();
    float driftPxPerFrame = satellite.calculateOrbitDrift(orbitSpeedKps, FPS);
    float pxToMachFactor = satellite.convertPxSpeedtoMs(1.0f, FPS) / 343.0f;

    DataGenerator gen(W, H, 0);
    if (!loadScenario(scenarioFile, gen, satellite, FPS)) {
        std::cerr << "ERROR: cannot open " << scenarioFile << "\n";
        return 1;
    }
    gen.setSatelliteVelocity(driftPxPerFrame, 0.0f);

    std::ofstream out(outFile.c_str());
    if (!out) {
        std::cerr << "ERROR: cannot write " << outFile << "\n";
        return 1;
    }
    out << "frame,obj,x,y,vx_rel,vy_rel,vx_abs,vy_abs,mach,temp_K\n";
    char buf[256];
    for (int t = 0; t < frames; ++t) {
        // Only physics: getNextFrame() is not called, so no cv::randn is consumed.
        gen.updatePhysics(pxToMachFactor);
        for (size_t i = 0; i < gen.hotspots.size(); ++i) {
            const SimHotspot& h = gen.hotspots[i];
            float satVx = gen.satVx;
            float satVy = gen.satVy;
            float mach = std::sqrt(h.vx * h.vx + h.vy * h.vy) * pxToMachFactor;
            std::snprintf(buf, sizeof(buf), "%d,%zu,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.2f\n",
                          t, i, h.x, h.y,
                          h.vx - satVx, h.vy - satVy,
                          h.vx, h.vy, mach, h.shockTemp);
            out << buf;
        }
    }
    return 0;
}
