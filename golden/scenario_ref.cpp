// scenario_ref.cpp - Reference trajectories for hotspot tracking test cases.
//
// Standalone C++ (no OpenCV) transcription of the physics used by postProcIR_Cpp:
//   - Satellite (src/Satellite.cpp): km/px, orbital drift, px/frame <-> m/s
//   - loadScenario (src/ScenarioLoader.h): scenario CSV parsing and m/s -> px/frame
//   - DataGenerator::updatePhysics (src/DataGenerator.cpp): motion, wrap, shock temperature
//
// Only deterministic scenarios are accepted: the random-walk acceleration term
// (acc=1) depends on the RNG of the original code and cannot be reproduced here.
//
// Output (CSV, one row per object per frame, after the physics step of that frame):
//   frame,obj,x,y,vx_rel,vy_rel,vx_abs,vy_abs,mach,temp_K
//     x,y          position in px (wrapped to the sensor)
//     vx_rel,vy_rel displacement per frame in px/frame = what the tracker should estimate
//     vx_abs,vy_abs absolute ground velocity in px/frame (= tracker velocity + satellite drift)
//     mach         |v_abs| * pxToMachFactor (same value the original uses for temperature)
//     temp_K       shock temperature (unchanged for fixedT=1)
//
// Build:  g++ -std=c++11 -O2 -Wall -Wextra golden/scenario_ref.cpp -o scenario_ref
// Run:    ./scenario_ref golden/scenarios/scenario.csv [frames=300] [W=640] [H=480] [FPS=30]

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------- Satellite
struct Satellite {
    float altitudeKm;
    float kmPerPixel;

    Satellite(double altKm, double fovDeg, int widthPx) : altitudeKm(static_cast<float>(altKm)) {
        float fov = static_cast<float>(fovDeg);
        float fovRad = fov * (M_PI / 180.0f);
        float groundWidthKm = 2.0f * altitudeKm * std::tan(fovRad / 2.0f);
        kmPerPixel = groundWidthKm / static_cast<float>(widthPx);
    }

    float getOrbitalVelocityKmS() const {
        const double EARTH_RADIUS_KM = 6371.0f;
        const double M = 5.972e24f;
        const double G = 6.67430e-11f;
        double orbitalRadius_m = (EARTH_RADIUS_KM + altitudeKm) * 1000.0f;
        double v_ms = std::sqrt((G * M) / orbitalRadius_m);
        return static_cast<float>(v_ms) / 1000.0f;
    }

    float calculateOrbitDrift(float orbitSpeedKps, int fps) const {
        float speedKmPerFrame = orbitSpeedKps / static_cast<float>(fps);
        return speedKmPerFrame / kmPerPixel;
    }

    float convertPxSpeedtoMs(float pxPerFrame, int fps) const {
        float speedKmPerFrame = pxPerFrame * kmPerPixel;
        float speedKmPerSecond = speedKmPerFrame * static_cast<float>(fps);
        return speedKmPerSecond * 1000.0f;
    }
};

// ---------------------------------------------------------------- Scenario
struct SimHotspot {
    float x, y, vx, vy;
    bool dampening, acceleration, fixedTemperature;
    double shockTemp;
    double beta;
};

// Mirrors loadScenario(): m/s -> px/frame with 0.001 / (kmPerPixel * fps).
static bool loadScenario(const std::string& filename, const Satellite& sat, int fps,
                         std::vector<SimHotspot>& out) {
    std::ifstream file(filename.c_str());
    if (!file.is_open()) return false;

    float conversionFactor = 0.001f / (sat.kmPerPixel * static_cast<float>(fps));
    std::string line;
    int lineNo = 0;
    while (std::getline(file, line)) {
        ++lineNo;
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string segment;
        std::vector<float> val;
        while (std::getline(ss, segment, ',')) val.push_back(std::stof(segment));

        if (val.size() >= 8) {
            SimHotspot h;
            h.x = val[0];
            h.y = val[1];
            h.vx = val[2] * conversionFactor;
            h.vy = val[3] * conversionFactor;
            h.dampening = (bool)val[4];
            h.acceleration = (bool)val[5];
            h.fixedTemperature = (bool)val[6];
            h.shockTemp = (double)val[7];
            h.beta = 80;
            if (h.acceleration) {
                std::cerr << "ERROR: " << filename << ":" << lineNo
                          << " acc=1 uses the original RNG and cannot be reproduced; set acc=0\n";
                std::exit(2);
            }
            out.push_back(h);
        }
    }
    return true;
}

// ---------------------------------------------------------------- Physics (DataGenerator.cpp)
static float obliqueShockPratio(float M, float beta, float g) {
    float beta_rad = M_PI * beta / 180;
    float Mn = M * std::sin(beta_rad);
    return 1 + (2 * g) / (g + 1) * (Mn * Mn - 1);
}

static float obliqueShockDratio(float M, float beta, float g) {
    float beta_rad = M_PI * beta / 180;
    float Mn = M * std::sin(beta_rad);
    return (g + 1) * Mn * Mn / ((g - 1) * Mn * Mn + 2);
}

static float obliqueShockTratio(float M, float beta, float g) {
    return obliqueShockPratio(M, beta, g) / obliqueShockDratio(M, beta, g);
}

struct Generator {
    int width, height;
    float satVx, satVy;
    std::vector<SimHotspot> hotspots;

    // Mirrors DataGenerator::updatePhysics without the random walk (acc must be 0).
    void updatePhysics(float pxToMachFactor, std::vector<float>& machOut) {
        const float GAMMA = 1.4f;
        const float T_AMBIENT = 250.0f;
        machOut.clear();

        for (auto& h : hotspots) {
            if (h.dampening) {
                h.vx *= 0.99f;
                h.vy *= 0.99f;
            }

            h.x += (h.vx - satVx);
            h.y += (h.vy - satVy);

            if (h.x < 0) h.x += width;
            else if (h.x >= width) h.x -= width;
            if (h.y < 0) h.y += height;
            else if (h.y >= height) h.y -= height;

            float absSpeed = std::sqrt(h.vx * h.vx + h.vy * h.vy);
            float M = absSpeed * pxToMachFactor;
            machOut.push_back(M);

            if (h.fixedTemperature) continue;

            if (M > 1.0f) {
                h.shockTemp = T_AMBIENT * obliqueShockTratio(M, (float)h.beta, GAMMA);
            } else {
                h.shockTemp = T_AMBIENT * (1.0f + 0.2f * M * M);
            }
            if (h.shockTemp < T_AMBIENT) h.shockTemp = T_AMBIENT;
        }
    }
};

// ---------------------------------------------------------------- main
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " scenario.csv [frames=300] [W=640] [H=480] [FPS=30]\n";
        return 1;
    }
    std::string scenarioFile = argv[1];
    int frames = argc > 2 ? std::atoi(argv[2]) : 300;
    int W = argc > 3 ? std::atoi(argv[3]) : 640;
    int H = argc > 4 ? std::atoi(argv[4]) : 480;
    int FPS = argc > 5 ? std::atoi(argv[5]) : 30;

    // Same constants as main.cpp
    Satellite satellite(500.0, 30.0, W);
    float orbitSpeedKps = satellite.getOrbitalVelocityKmS();
    float driftPxPerFrame = satellite.calculateOrbitDrift(orbitSpeedKps, FPS);
    float pxToMachFactor = satellite.convertPxSpeedtoMs(1.0f, FPS) / 343.0f;

    Generator gen;
    gen.width = W;
    gen.height = H;
    gen.satVx = driftPxPerFrame;
    gen.satVy = 0.0f;

    if (!loadScenario(scenarioFile, satellite, FPS, gen.hotspots)) {
        std::cerr << "ERROR: cannot open " << scenarioFile << "\n";
        return 1;
    }

    std::cerr << "km/px=" << satellite.kmPerPixel << " drift=" << driftPxPerFrame
              << " px/frame, 1 px/frame = Mach " << pxToMachFactor
              << ", objects=" << gen.hotspots.size() << "\n";

    std::printf("frame,obj,x,y,vx_rel,vy_rel,vx_abs,vy_abs,mach,temp_K\n");
    std::vector<float> mach;
    for (int t = 0; t < frames; ++t) {
        gen.updatePhysics(pxToMachFactor, mach);
        for (size_t i = 0; i < gen.hotspots.size(); ++i) {
            const SimHotspot& h = gen.hotspots[i];
            std::printf("%d,%zu,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.2f\n",
                        t, i, h.x, h.y,
                        h.vx - gen.satVx, h.vy - gen.satVy,
                        h.vx, h.vy, mach[i], h.shockTemp);
        }
    }
    return 0;
}
