// uh_probe -- exercise the UniversalHaptics backend layer without SOFA.
//
//   uh_probe [auto|touch|haption] [seconds] [stiffness N/m] [--touch NAME] [--haption ADDR]
//
// Detects a device exactly the way the SOFA component does (device="auto"),
// prints its pose in the common frame (metres, Y up) and, with stiffness > 0,
// renders a weak spring towards the start position (clamped to 2 N).

#include <UniversalHaptics/backend/HapticBackend.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

using namespace universalhaptics;

static std::atomic<bool> g_stop {false};
static void onSignal(int) { g_stop = true; }

int main(int argc, char** argv)
{
    std::string device = "auto", touchName, haptionAddr = "127.0.0.1#5000";
    double seconds = 5.0, stiffness = 0.0;
    int positional = 0;
    for (int i = 1; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--touch") && i + 1 < argc) touchName = argv[++i];
        else if (!std::strcmp(argv[i], "--haption") && i + 1 < argc) haptionAddr = argv[++i];
        else if (positional == 0) { device = argv[i]; ++positional; }
        else if (positional == 1) { seconds = std::atof(argv[i]); ++positional; }
        else if (positional == 2) { stiffness = std::atof(argv[i]); ++positional; }
    }
    std::signal(SIGINT, onSignal);

    auto targetFor = [&](const std::string& n) { return n == "touch" ? touchName : haptionAddr; };
    std::vector<std::string> order = device == "auto" ? availableBackendNames() : std::vector<std::string>{ device };
    std::vector<std::string> log;
    auto backend = detectDevice(order, targetFor, log);
    for (const auto& line : log) std::printf("  %s\n", line.c_str());
    if (!backend)
    {
        std::printf("No haptic device found.\n");
        return 1;
    }
    const DeviceInfo info = backend->info();
    std::printf("Using %s: %s [%s]\n", info.backend.c_str(), info.model.c_str(), info.identifier.c_str());

    std::mutex m;
    DeviceState latest;
    bool haveAnchor = false;
    std::array<double, 3> anchor {0, 0, 0};
    std::atomic<long> ticks {0};

    std::string error;
    const bool started = backend->start([&](const DeviceState& s, DeviceOutput& out) {
        ++ticks;
        std::lock_guard<std::mutex> lock(m);
        latest = s;
        if (!haveAnchor && ticks > 300) { anchor = s.position; haveAnchor = true; }
        if (haveAnchor && stiffness > 0)
        {
            double n = 0;
            for (int i = 0; i < 3; ++i) { out.force[i] = -stiffness * (s.position[i] - anchor[i]); n += out.force[i] * out.force[i]; }
            n = std::sqrt(n);
            if (n > 2.0) for (double& f : out.force) f *= 2.0 / n;
        }
    }, error);
    if (!started)
    {
        std::printf("start failed: %s\n", error.c_str());
        return 2;
    }

    std::printf("\n%6s %9s %9s %9s   %7s %7s %7s %7s  btn ready  rate\n", "t", "x[m]", "y[m]", "z[m]", "qx", "qy", "qz", "qw");
    const auto t0 = std::chrono::steady_clock::now();
    long lastTicks = 0;
    while (!g_stop)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        DeviceState s;
        { std::lock_guard<std::mutex> lock(m); s = latest; }
        const long now = ticks;
        std::printf("%6.2f %9.4f %9.4f %9.4f   %7.3f %7.3f %7.3f %7.3f  %3u %5d %5.0f Hz\n", t,
                    s.position[0], s.position[1], s.position[2],
                    s.orientation[0], s.orientation[1], s.orientation[2], s.orientation[3],
                    s.buttons, s.motorsReady ? 1 : 0, (now - lastTicks) / 0.5);
        std::fflush(stdout);
        lastTicks = now;
        if (t >= seconds) break;
    }
    backend->stop();
    backend->close();
    std::printf("\nStopped cleanly.\n");
    return 0;
}
