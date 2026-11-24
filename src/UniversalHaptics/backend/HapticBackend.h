/******************************************************************************
*  UniversalHaptics -- device-independent haptics for SOFA                    *
*                                                                             *
*  Backend layer. Plain C++17: no SOFA, no vendor headers. Each backend loads *
*  its vendor SDK at runtime, so the plugin builds and runs on a machine that *
*  has only one SDK installed (or none).                                      *
******************************************************************************/
#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace universalhaptics
{

/// Device state in the COMMON FRAME shared by all backends:
///   right-handed, metres, X right, Y up, Z towards the user
///   (the OpenHaptics convention, which HAPI also used for the Haption).
struct DeviceState
{
    std::array<double, 3> position {0, 0, 0};          ///< m
    std::array<double, 4> orientation {0, 0, 0, 1};    ///< unit quaternion (x, y, z, w)
    std::array<double, 3> velocity {0, 0, 0};          ///< m/s
    unsigned int buttons = 0;                          ///< bit i = button i+1
    bool motorsReady = true;  ///< false e.g. when the Haption dead-man switch is released
};

/// Output requested by the servo callback, in the common frame.
struct DeviceOutput
{
    std::array<double, 3> force {0, 0, 0};   ///< N
    std::array<double, 3> torque {0, 0, 0};  ///< N.m (ignored by 3-DOF output devices)
};

struct DeviceInfo
{
    std::string backend;      ///< "touch" / "haption"
    std::string model;        ///< as reported by the SDK, if available
    std::string identifier;   ///< device name / address / serial
    double nominalMaxForce = 0.0;  ///< N, 0 if unknown
    bool torqueOutput = false;
};

/// Called by the backend's servo thread, ~1 kHz. Must be fast and must not block.
using ServoCallback = std::function<void(const DeviceState&, DeviceOutput&)>;

class HapticBackend
{
public:
    virtual ~HapticBackend() = default;

    /// Short id used in scenes: "touch" or "haption".
    virtual std::string name() const = 0;

    /// Load the vendor library. False (with reason) if the SDK is not installed.
    virtual bool loadLibrary(std::string& error) = 0;

    /// Connect to the device. `target` is backend specific:
    ///   touch   : OpenHaptics configuration name ("" = default device)
    ///   haption : "ip#port" of the haptic service ("" = 127.0.0.1#5000)
    virtual bool open(const std::string& target, std::string& error) = 0;

    /// Start the servo loop. Forces are sent only from inside the callback.
    virtual bool start(ServoCallback callback, std::string& error) = 0;

    /// Stop the servo loop (zero force is sent first). Safe to call twice.
    virtual void stop() = 0;

    /// Release the device. Safe to call twice.
    virtual void close() = 0;

    virtual DeviceInfo info() const = 0;
};

/// Create a backend by name ("touch" | "haption"). nullptr for unknown names.
std::unique_ptr<HapticBackend> createBackend(const std::string& name);

/// Names of all backends compiled into the plugin, in default detection order.
std::vector<std::string> availableBackendNames();

/// Try each backend in `order`; return the first one whose library loads AND whose
/// device opens. `targets` maps backend name -> target string. `log` receives one
/// line per attempt, explaining why each backend was skipped.
std::unique_ptr<HapticBackend> detectDevice(const std::vector<std::string>& order,
                                            const std::function<std::string(const std::string&)>& targetFor,
                                            std::vector<std::string>& log);

// ---------------------------------------------------------------- small math
namespace math
{
using Quat = std::array<double, 4>; // x, y, z, w

/// Hamilton product a*b (apply b first, then a).
inline Quat mul(const Quat& a, const Quat& b)
{
    return { a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
             a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
             a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
             a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2] };
}

/// Quaternion from a 3x3 rotation matrix given row-major m[r][c].
Quat fromMatrix(const double m[3][3]);

void normalize(Quat& q);
} // namespace math

} // namespace universalhaptics
