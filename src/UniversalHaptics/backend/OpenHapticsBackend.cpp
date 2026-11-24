#include <UniversalHaptics/backend/OpenHapticsBackend.h>

#include <cmath>
#include <cstdlib>
#include <vector>

// ---------------------------------------------------------------------------
// The subset of the OpenHaptics HD API we use, declared locally so the plugin
// compiles without the SDK headers. Values match OpenHaptics 3.x hdDefines.h.
// ---------------------------------------------------------------------------
#if defined(_WIN32) && !defined(_WIN64)
#  define UH_HDAPIENTRY __stdcall    // only matters for 32-bit Windows
#else
#  define UH_HDAPIENTRY
#endif

namespace
{
using HHD = unsigned int;
using HDerror = unsigned int;
using HDenum = unsigned int;
using HDSchedulerHandle = unsigned long;
using HDCallbackCode = unsigned int;
struct HDErrorInfo { HDerror errorCode; int internalErrorCode; HHD hHD; };
using HDSchedulerCallback = HDCallbackCode (UH_HDAPIENTRY*)(void*);

constexpr HDerror HD_SUCCESS                       = 0x0000;
constexpr HDerror HD_DEVICE_ALREADY_INITIATED      = 0x0301;
constexpr HDenum  HD_CURRENT_BUTTONS               = 0x2000;
constexpr HDenum  HD_CURRENT_VELOCITY              = 0x2051;
constexpr HDenum  HD_CURRENT_TRANSFORM             = 0x2052;
constexpr HDenum  HD_DEVICE_MODEL_TYPE             = 0x2501;
constexpr HDenum  HD_DEVICE_SERIAL_NUMBER          = 0x2504;
constexpr HDenum  HD_NOMINAL_MAX_CONTINUOUS_FORCE  = 0x2604;
constexpr HDenum  HD_CURRENT_FORCE                 = 0x2700;
constexpr HDenum  HD_FORCE_OUTPUT                  = 0x4000;
constexpr HDenum  HD_MAX_FORCE_CLAMPING            = 0x4001;
constexpr HDenum  HD_SOFTWARE_FORCE_LIMIT          = 0x4003;
constexpr HDenum  HD_CALIBRATION_OK                = 0x5000;
constexpr HHD     HD_INVALID_HANDLE                = 0xFFFFFFFF;
constexpr unsigned short HD_MAX_SCHEDULER_PRIORITY = 0xFFFF;
constexpr HDCallbackCode HD_CALLBACK_CONTINUE      = 1;

// OpenHaptics allows one scheduler per process.
bool g_schedulerRunning = false;
}

namespace universalhaptics
{

struct OpenHapticsBackend::Api
{
    HHD          (UH_HDAPIENTRY* hdInitDevice)(const char*) = nullptr;
    void         (UH_HDAPIENTRY* hdDisableDevice)(HHD) = nullptr;
    void         (UH_HDAPIENTRY* hdMakeCurrentDevice)(HHD) = nullptr;
    HHD          (UH_HDAPIENTRY* hdGetCurrentDevice)() = nullptr;
    void         (UH_HDAPIENTRY* hdBeginFrame)(HHD) = nullptr;
    void         (UH_HDAPIENTRY* hdEndFrame)(HHD) = nullptr;
    HDErrorInfo  (UH_HDAPIENTRY* hdGetError)() = nullptr;
    const char*  (UH_HDAPIENTRY* hdGetErrorString)(HDerror) = nullptr;
    void         (UH_HDAPIENTRY* hdEnable)(HDenum) = nullptr;
    void         (UH_HDAPIENTRY* hdGetIntegerv)(HDenum, int*) = nullptr;
    void         (UH_HDAPIENTRY* hdGetDoublev)(HDenum, double*) = nullptr;
    const char*  (UH_HDAPIENTRY* hdGetString)(HDenum) = nullptr;
    void         (UH_HDAPIENTRY* hdSetDoublev)(HDenum, const double*) = nullptr;
    HDenum       (UH_HDAPIENTRY* hdCheckCalibration)() = nullptr;
    void         (UH_HDAPIENTRY* hdStartScheduler)() = nullptr;
    void         (UH_HDAPIENTRY* hdStopScheduler)() = nullptr;
    HDSchedulerHandle (UH_HDAPIENTRY* hdScheduleAsynchronous)(HDSchedulerCallback, void*, unsigned short) = nullptr;
    void         (UH_HDAPIENTRY* hdUnschedule)(HDSchedulerHandle) = nullptr;
};

struct OpenHapticsBackend::ServoAccess
{
    static HDCallbackCode UH_HDAPIENTRY servo(void* userData)
    {
        auto* self = static_cast<OpenHapticsBackend*>(userData);
        const Api& hd = *self->m_api;

        hd.hdBeginFrame(self->m_device);

        double t[16];          // column-major 4x4, translation in mm
        double v[3];           // mm/s
        int buttons = 0;
        hd.hdGetDoublev(HD_CURRENT_TRANSFORM, t);
        hd.hdGetDoublev(HD_CURRENT_VELOCITY, v);
        hd.hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);

        DeviceState s;
        for (int i = 0; i < 3; ++i)
        {
            s.position[i] = t[12 + i] * 1e-3;
            s.velocity[i] = v[i] * 1e-3;
        }
        double r[3][3];
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                r[row][col] = t[col * 4 + row];
        s.orientation = math::fromMatrix(r);
        s.buttons = static_cast<unsigned int>(buttons);

        DeviceOutput out;
        if (self->m_callback) self->m_callback(s, out);

        double f[3] = { out.force[0], out.force[1], out.force[2] };
        const double n = std::sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
        if (self->m_maxForce > 0.0 && n > self->m_maxForce)
            for (double& c : f) c *= self->m_maxForce / n;
        hd.hdSetDoublev(HD_CURRENT_FORCE, f);

        hd.hdEndFrame(self->m_device);
        hd.hdGetError(); // clear any per-frame error so it doesn't stick
        return HD_CALLBACK_CONTINUE;
    }
};

OpenHapticsBackend::OpenHapticsBackend()
    : m_api(std::make_unique<Api>())
    , m_device(HD_INVALID_HANDLE)
    , m_servoHandle(0)
{
    m_info.backend = "touch";
}

OpenHapticsBackend::~OpenHapticsBackend()
{
    stop();
    close();
}

std::string OpenHapticsBackend::lastError() const
{
    if (!m_api->hdGetError) return "OpenHaptics not loaded";
    const HDErrorInfo e = m_api->hdGetError();
    if (e.errorCode == HD_SUCCESS) return {};
    std::string msg = "HD error 0x";
    static const char* hex = "0123456789abcdef";
    for (int sh = 12; sh >= 0; sh -= 4) msg += hex[(e.errorCode >> sh) & 0xF];
    if (m_api->hdGetErrorString)
        if (const char* str = m_api->hdGetErrorString(e.errorCode)) msg += std::string(" (") + str + ")";
    msg += ", internal " + std::to_string(e.internalErrorCode);
    return msg;
}

bool OpenHapticsBackend::loadLibrary(std::string& error)
{
    if (m_lib.isLoaded()) return true;

    std::vector<std::string> candidates;
    if (const char* env = std::getenv("UH_OPENHAPTICS_LIB")) candidates.emplace_back(env);
#if defined(_WIN32)
    candidates.insert(candidates.end(), { "hd.dll", "HD.dll" });
#else
    candidates.insert(candidates.end(), { "libHD.so.3.4", "libHD.so.3", "libHD.so" });
#endif
    if (!m_lib.load(candidates, error))
    {
        error = "OpenHaptics HD library not found [" + error + "]";
        return false;
    }

    std::string missing;
    Api& a = *m_api;
    m_lib.resolve(a.hdInitDevice, "hdInitDevice", missing);
    m_lib.resolve(a.hdDisableDevice, "hdDisableDevice", missing);
    m_lib.resolve(a.hdMakeCurrentDevice, "hdMakeCurrentDevice", missing);
    m_lib.resolve(a.hdGetCurrentDevice, "hdGetCurrentDevice", missing);
    m_lib.resolve(a.hdBeginFrame, "hdBeginFrame", missing);
    m_lib.resolve(a.hdEndFrame, "hdEndFrame", missing);
    m_lib.resolve(a.hdGetError, "hdGetError", missing);
    m_lib.resolve(a.hdGetErrorString, "hdGetErrorString", missing);
    m_lib.resolve(a.hdEnable, "hdEnable", missing);
    m_lib.resolve(a.hdGetIntegerv, "hdGetIntegerv", missing);
    m_lib.resolve(a.hdGetDoublev, "hdGetDoublev", missing);
    m_lib.resolve(a.hdGetString, "hdGetString", missing);
    m_lib.resolve(a.hdSetDoublev, "hdSetDoublev", missing);
    m_lib.resolve(a.hdCheckCalibration, "hdCheckCalibration", missing);
    m_lib.resolve(a.hdStartScheduler, "hdStartScheduler", missing);
    m_lib.resolve(a.hdStopScheduler, "hdStopScheduler", missing);
    m_lib.resolve(a.hdScheduleAsynchronous, "hdScheduleAsynchronous", missing);
    m_lib.resolve(a.hdUnschedule, "hdUnschedule", missing);
    if (!missing.empty())
    {
        error = m_lib.loadedName() + " is missing symbols: " + missing;
        m_lib.unload();
        return false;
    }
    return true;
}

bool OpenHapticsBackend::open(const std::string& target, std::string& error)
{
    if (!loadLibrary(error)) return false;
    const Api& hd = *m_api;

    if (g_schedulerRunning)
    {
        hd.hdStopScheduler();
        g_schedulerRunning = false;
    }

    // Pass the configuration name explicitly, like SOFA's Geomagic plugin.
    const std::string name = target.empty() ? std::string("Default Device") : target;
    const char* config = name.c_str();

    // OpenHaptics keeps a STACK of errors; drain all of it. Some driver versions push
    // unrelated errors (e.g. 0x0101 HD_INVALID_VALUE while reading their config) on top
    // of the real one, so the first code alone is misleading.
    auto drainErrors = [&hd]() {
        std::vector<HDErrorInfo> errs;
        for (int i = 0; i < 32; ++i)
        {
            const HDErrorInfo e = hd.hdGetError();
            if (e.errorCode == HD_SUCCESS) break;
            errs.push_back(e);
        }
        return errs;
    };
    auto describe = [&hd](const std::vector<HDErrorInfo>& errs) {
        static const char* hex = "0123456789abcdef";
        std::string out;
        for (const auto& e : errs)
        {
            if (!out.empty()) out += "; ";
            out += "0x";
            for (int sh = 12; sh >= 0; sh -= 4) out += hex[(e.errorCode >> sh) & 0xF];
            if (const char* str = hd.hdGetErrorString(e.errorCode)) out += std::string(" (") + str + ")";
            out += " [internal " + std::to_string(e.internalErrorCode) + "]";
        }
        return out;
    };
    auto has = [](const std::vector<HDErrorInfo>& errs, HDerror code) {
        for (const auto& e : errs) if (e.errorCode == code) return true;
        return false;
    };
    // 0x03xx = device / communication / initialisation errors: the device is not usable.
    auto hasDeviceError = [](const std::vector<HDErrorInfo>& errs) {
        for (const auto& e : errs)
            if ((e.errorCode & 0xFF00) == 0x0300 && e.errorCode != HD_DEVICE_ALREADY_INITIATED) return true;
        return false;
    };

    drainErrors(); // discard anything left from earlier calls
    m_device = hd.hdInitDevice(config);
    std::vector<HDErrorInfo> errs = drainErrors();

    if (has(errs, HD_DEVICE_ALREADY_INITIATED))
    {
        // Left over from a crashed run in this process: release and retry once.
        const HHD current = hd.hdGetCurrentDevice();
        if (current != HD_INVALID_HANDLE) hd.hdDisableDevice(current);
        drainErrors();
        m_device = hd.hdInitDevice(config);
        errs = drainErrors();
    }

    if (m_device == HD_INVALID_HANDLE || hasDeviceError(errs))
    {
        error = "hdInitDevice(\"" + name + "\") failed: " + (errs.empty() ? std::string("invalid handle, no error reported") : describe(errs));
        if (m_device != HD_INVALID_HANDLE) hd.hdDisableDevice(m_device);
        drainErrors();
        m_device = HD_INVALID_HANDLE;
        return false;
    }
    if (!errs.empty())
        error = "warning: hdInitDevice succeeded but the driver also reported: " + describe(errs);

    hd.hdMakeCurrentDevice(m_device);
    if (hd.hdCheckCalibration() != HD_CALIBRATION_OK)
        error += std::string(error.empty() ? "" : " | ") + "warning: device reports it needs calibration (run Touch_Setup / TouchCheckup)";

    m_info.identifier = name;
    if (const char* model = hd.hdGetString(HD_DEVICE_MODEL_TYPE)) m_info.model = model;
    if (const char* serial = hd.hdGetString(HD_DEVICE_SERIAL_NUMBER))
        if (*serial) m_info.identifier += std::string(" (sn ") + serial + ")";
    hd.hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &m_maxForce);
    m_info.nominalMaxForce = m_maxForce;
    m_info.torqueOutput = false;
    hd.hdGetError();
    return true;
}

bool OpenHapticsBackend::start(ServoCallback callback, std::string& error)
{
    if (m_device == HD_INVALID_HANDLE) { error = "device not open"; return false; }
    const Api& hd = *m_api;
    m_callback = std::move(callback);

    hd.hdMakeCurrentDevice(m_device);
    hd.hdEnable(HD_FORCE_OUTPUT);
    hd.hdEnable(HD_MAX_FORCE_CLAMPING);
    hd.hdEnable(HD_SOFTWARE_FORCE_LIMIT);

    m_servoHandle = hd.hdScheduleAsynchronous(&ServoAccess::servo, this, HD_MAX_SCHEDULER_PRIORITY);
    hd.hdStartScheduler();
    g_schedulerRunning = true;
    m_schedulerStarted = true;

    const std::string err = lastError();
    if (!err.empty())
    {
        error = "failed to start the OpenHaptics scheduler: " + err;
        stop();
        return false;
    }
    return true;
}

void OpenHapticsBackend::stop()
{
    if (!m_schedulerStarted || !m_api->hdStopScheduler) return;
    const Api& hd = *m_api;
    hd.hdStopScheduler();
    g_schedulerRunning = false;
    if (m_servoHandle) hd.hdUnschedule(m_servoHandle);
    m_servoHandle = 0;
    m_schedulerStarted = false;
    hd.hdGetError();
}

void OpenHapticsBackend::close()
{
    if (m_device != HD_INVALID_HANDLE && m_api->hdDisableDevice)
    {
        m_api->hdDisableDevice(m_device);
        m_api->hdGetError();
    }
    m_device = HD_INVALID_HANDLE;
}

DeviceInfo OpenHapticsBackend::info() const
{
    return m_info;
}

} // namespace universalhaptics
