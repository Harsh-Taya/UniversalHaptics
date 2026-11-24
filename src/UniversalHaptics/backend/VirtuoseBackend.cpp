#include <UniversalHaptics/backend/VirtuoseBackend.h>

#include <chrono>
#include <cmath>
#include <cstdlib>

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <timeapi.h>
#else
#  include <pthread.h>
#  include <sched.h>
#endif

// ---------------------------------------------------------------------------
// Subset of VirtuoseAPI (v4.x), declared locally so no SDK header is needed.
// The API is plain C with the default calling convention.
// ---------------------------------------------------------------------------
namespace
{
using VirtContext = void*;
constexpr int INDEXING_ALL = 0;
constexpr int COMMAND_TYPE_IMPEDANCE = 3;
constexpr int VIRT_E_VIRTUOSE_DLL_NOT_FOUND = 13;
}

namespace universalhaptics
{

struct VirtuoseBackend::Api
{
    int         (*virtAPIVersion)(int*, int*) = nullptr;
    VirtContext (*virtOpen)(const char*) = nullptr;
    int         (*virtClose)(VirtContext) = nullptr;
    int         (*virtGetErrorCode)(VirtContext) = nullptr;
    char*       (*virtGetErrorMessage)(int) = nullptr;
    int         (*virtGetControlerVersion)(VirtContext, int*, int*) = nullptr;
    int         (*virtSetIndexingMode)(VirtContext, int) = nullptr;
    int         (*virtSetForceFactor)(VirtContext, float) = nullptr;
    int         (*virtSetSpeedFactor)(VirtContext, float) = nullptr;
    int         (*virtSetTimeStep)(VirtContext, float) = nullptr;
    int         (*virtSetBaseFrame)(VirtContext, float*) = nullptr;
    int         (*virtSetObservationFrame)(VirtContext, float*) = nullptr;
    int         (*virtSetCommandType)(VirtContext, int) = nullptr;
    int         (*virtSetPowerOn)(VirtContext, int) = nullptr;
    int         (*virtGetPosition)(VirtContext, float*) = nullptr;
    int         (*virtGetSpeed)(VirtContext, float*) = nullptr;
    int         (*virtGetButton)(VirtContext, int, int*) = nullptr;
    int         (*virtGetDeadMan)(VirtContext, int*) = nullptr;
    int         (*virtSetForce)(VirtContext, float*) = nullptr;
};

VirtuoseBackend::VirtuoseBackend()
    : m_api(std::make_unique<Api>())
{
    m_info.backend = "haption";
    m_info.torqueOutput = true;
}

VirtuoseBackend::~VirtuoseBackend()
{
    stop();
    close();
}

std::string VirtuoseBackend::errorMessage() const
{
    if (!m_api->virtGetErrorCode) return "VirtuoseAPI not loaded";
    const int code = m_api->virtGetErrorCode(m_context);
    if (code == VIRT_E_VIRTUOSE_DLL_NOT_FOUND) return "VirtuoseAPI library not found";
    const char* msg = m_api->virtGetErrorMessage ? m_api->virtGetErrorMessage(code) : nullptr;
    return "error " + std::to_string(code) + (msg ? std::string(" (") + msg + ")" : std::string());
}

bool VirtuoseBackend::loadLibrary(std::string& error)
{
    if (m_lib.isLoaded()) return true;

    std::vector<std::string> candidates;
    if (const char* env = std::getenv("UH_VIRTUOSE_LIB")) candidates.emplace_back(env);
#if defined(_WIN32)
    candidates.insert(candidates.end(), { "virtuoseAPI.dll", "VirtuoseDLL.dll" });
#else
    candidates.insert(candidates.end(), { "libvirtuoseAPI.so", "virtuoseAPI.so" });
#endif
    if (!m_lib.load(candidates, error))
    {
        error = "VirtuoseAPI library not found [" + error + "]";
        return false;
    }

    std::string missing;
    Api& a = *m_api;
    m_lib.resolve(a.virtAPIVersion, "virtAPIVersion", missing);
    m_lib.resolve(a.virtOpen, "virtOpen", missing);
    m_lib.resolve(a.virtClose, "virtClose", missing);
    m_lib.resolve(a.virtGetErrorCode, "virtGetErrorCode", missing);
    m_lib.resolve(a.virtGetErrorMessage, "virtGetErrorMessage", missing);
    m_lib.resolve(a.virtGetControlerVersion, "virtGetControlerVersion", missing);
    m_lib.resolve(a.virtSetIndexingMode, "virtSetIndexingMode", missing);
    m_lib.resolve(a.virtSetForceFactor, "virtSetForceFactor", missing);
    m_lib.resolve(a.virtSetSpeedFactor, "virtSetSpeedFactor", missing);
    m_lib.resolve(a.virtSetTimeStep, "virtSetTimeStep", missing);
    m_lib.resolve(a.virtSetBaseFrame, "virtSetBaseFrame", missing);
    m_lib.resolve(a.virtSetObservationFrame, "virtSetObservationFrame", missing);
    m_lib.resolve(a.virtSetCommandType, "virtSetCommandType", missing);
    m_lib.resolve(a.virtSetPowerOn, "virtSetPowerOn", missing);
    m_lib.resolve(a.virtGetPosition, "virtGetPosition", missing);
    m_lib.resolve(a.virtGetSpeed, "virtGetSpeed", missing);
    m_lib.resolve(a.virtGetButton, "virtGetButton", missing);
    m_lib.resolve(a.virtGetDeadMan, "virtGetDeadMan", missing);
    m_lib.resolve(a.virtSetForce, "virtSetForce", missing);
    if (!missing.empty())
    {
        error = m_lib.loadedName() + " is missing symbols: " + missing;
        m_lib.unload();
        return false;
    }
    return true;
}

bool VirtuoseBackend::open(const std::string& target, std::string& error)
{
    if (!loadLibrary(error)) return false;
    const Api& v = *m_api;
    const std::string address = target.empty() ? "127.0.0.1#5000" : target;

    m_context = v.virtOpen(address.c_str());
    if (!m_context)
    {
        error = "virtOpen(\"" + address + "\") failed: " + errorMessage()
              + ". Is the haptic service (SvcHaptic_* or virtuoseSIM, run as root) up on that port?";
        return false;
    }

    float identity[7] = {0, 0, 0, 0, 0, 0, 1};
    bool ok = true;
    ok &= v.virtSetIndexingMode(m_context, INDEXING_ALL) == 0;
    ok &= v.virtSetForceFactor(m_context, 1.0f) == 0;
    ok &= v.virtSetSpeedFactor(m_context, 1.0f) == 0;
    ok &= v.virtSetTimeStep(m_context, static_cast<float>(kTimeStep)) == 0;
    ok &= v.virtSetBaseFrame(m_context, identity) == 0;
    ok &= v.virtSetObservationFrame(m_context, identity) == 0;
    ok &= v.virtSetCommandType(m_context, COMMAND_TYPE_IMPEDANCE) == 0;
    if (!ok)
    {
        error = "configuring the Virtuose failed: " + errorMessage();
        v.virtClose(m_context);
        m_context = nullptr;
        return false;
    }

    int maj = 0, min = 0;
    m_info.identifier = address;
    m_info.model = "Virtuose";
    if (v.virtGetControlerVersion(m_context, &maj, &min) == 0)
        m_info.model += " (controller " + std::to_string(maj) + "." + std::to_string(min) + ")";
    m_info.nominalMaxForce = 0.0; // not reported by the API

    // Testing aid: headless VirtuoseSIM cannot press its dead-man button.
    const char* ign = std::getenv("UH_IGNORE_DEADMAN");
    m_ignoreDeadMan = ign && *ign && std::string(ign) != "0";
    if (m_ignoreDeadMan)
        error = "WARNING: UH_IGNORE_DEADMAN is set -- dead-man switch ignored. Use with the simulator only.";
    return true;
}

bool VirtuoseBackend::start(ServoCallback callback, std::string& error)
{
    if (!m_context) { error = "device not open"; return false; }
    if (m_running) return true;
    if (m_api->virtSetPowerOn(m_context, 1) != 0)
    {
        error = "virtSetPowerOn failed: " + errorMessage();
        return false;
    }
    m_callback = std::move(callback);
    m_running = true;
    m_thread = std::thread(&VirtuoseBackend::servoLoop, this);
    return true;
}

void VirtuoseBackend::servoLoop()
{
    const Api& v = *m_api;

#if defined(_WIN32)
    timeBeginPeriod(1);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#else
    sched_param sp {};
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp); // best effort; needs root/rtprio
#endif

    // Fixed rotation between the Virtuose handle frame and the common frame
    // (same convention HAPI used for this device): -90 deg about X.
    const double h = std::sqrt(0.5);
    const math::Quat handleFix { -h, 0.0, 0.0, h };

    const auto period = std::chrono::microseconds(static_cast<long>(kTimeStep * 1e6));
    auto next = std::chrono::steady_clock::now();

    while (m_running)
    {
        float p[7] = {0, 0, 0, 0, 0, 0, 1};
        float s[6] = {0, 0, 0, 0, 0, 0};
        int b1 = 0, b2 = 0, dead = 1;
        v.virtGetPosition(m_context, p);
        v.virtGetSpeed(m_context, s);
        v.virtGetButton(m_context, 1, &b1);
        v.virtGetButton(m_context, 2, &b2);
        v.virtGetDeadMan(m_context, &dead);

        // Virtuose frame -> common frame: (x, y, z)_common = (y, z, x)_virtuose
        DeviceState st;
        st.position = { p[1], p[2], p[0] };
        st.velocity = { s[1], s[2], s[0] };
        st.orientation = math::mul(math::Quat{ p[4], p[5], p[3], p[6] }, handleFix);
        math::normalize(st.orientation);
        st.buttons = (b1 ? 1u : 0u) | (b2 ? 2u : 0u);
        st.motorsReady = m_ignoreDeadMan || dead != 0;

        DeviceOutput out;
        if (m_callback) m_callback(st, out);

        // common frame -> Virtuose frame: (x, y, z)_virtuose = (z, x, y)_common
        float f[6] = { static_cast<float>(out.force[2]),  static_cast<float>(out.force[0]),  static_cast<float>(out.force[1]),
                       static_cast<float>(out.torque[2]), static_cast<float>(out.torque[0]), static_cast<float>(out.torque[1]) };
        v.virtSetForce(m_context, f);

        next += period;
        const auto now = std::chrono::steady_clock::now();
        if (next < now) next = now; // overran: don't try to catch up in a burst
        std::this_thread::sleep_until(next);
    }

    float zero[6] = {0, 0, 0, 0, 0, 0};
    v.virtSetForce(m_context, zero);

#if defined(_WIN32)
    timeEndPeriod(1);
#endif
}

void VirtuoseBackend::stop()
{
    if (m_running)
    {
        m_running = false;
        if (m_thread.joinable()) m_thread.join();
    }
    if (m_context && m_api->virtSetPowerOn) m_api->virtSetPowerOn(m_context, 0);
}

void VirtuoseBackend::close()
{
    if (m_context && m_api->virtClose) m_api->virtClose(m_context);
    m_context = nullptr;
}

DeviceInfo VirtuoseBackend::info() const
{
    return m_info;
}

} // namespace universalhaptics
