#include <UniversalHaptics/backend/HapticBackend.h>
#include <UniversalHaptics/backend/OpenHapticsBackend.h>
#include <UniversalHaptics/backend/VirtuoseBackend.h>

#include <algorithm>
#include <cctype>
#include <cmath>

namespace universalhaptics
{

namespace
{
std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}
}

std::unique_ptr<HapticBackend> createBackend(const std::string& rawName)
{
    const std::string name = lower(rawName);
    if (name == "touch" || name == "geomagic" || name == "phantom" || name == "openhaptics")
        return std::make_unique<OpenHapticsBackend>();
    if (name == "haption" || name == "virtuose")
        return std::make_unique<VirtuoseBackend>();
    return nullptr;
}

std::vector<std::string> availableBackendNames()
{
    return { "touch", "haption" };
}

std::unique_ptr<HapticBackend> detectDevice(const std::vector<std::string>& order,
                                            const std::function<std::string(const std::string&)>& targetFor,
                                            std::vector<std::string>& log)
{
    for (const auto& name : order)
    {
        auto backend = createBackend(name);
        if (!backend)
        {
            log.push_back(name + ": unknown device type (expected touch or haption)");
            continue;
        }
        std::string error;
        if (!backend->loadLibrary(error))
        {
            log.push_back(backend->name() + ": skipped, " + error);
            continue;
        }
        const std::string target = targetFor ? targetFor(backend->name()) : std::string();
        if (!backend->open(target, error))
        {
            log.push_back(backend->name() + ": no device, " + error);
            continue;
        }
        const DeviceInfo info = backend->info();
        log.push_back(backend->name() + ": found " + info.model + " [" + info.identifier + "]"
                      + (error.empty() ? std::string() : " -- " + error));
        return backend;
    }
    return nullptr;
}

namespace math
{
void normalize(Quat& q)
{
    const double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-12) { q = {0, 0, 0, 1}; return; }
    for (double& c : q) c /= n;
}

Quat fromMatrix(const double m[3][3])
{
    // Shepperd's method, numerically stable for all rotations.
    Quat q;
    const double tr = m[0][0] + m[1][1] + m[2][2];
    if (tr > 0.0)
    {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        q = { (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s };
    }
    else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
    {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        q = { 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s };
    }
    else if (m[1][1] > m[2][2])
    {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        q = { (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s };
    }
    else
    {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        q = { (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s };
    }
    normalize(q);
    return q;
}
} // namespace math

} // namespace universalhaptics
