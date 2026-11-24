/******************************************************************************
*  UniversalHaptics -- 3D Systems / Geomagic Touch backend (OpenHaptics HD)   *
******************************************************************************/
#pragma once

#include <UniversalHaptics/backend/HapticBackend.h>
#include <UniversalHaptics/backend/DynamicLibrary.h>

#include <atomic>
#include <mutex>

namespace universalhaptics
{

class OpenHapticsBackend : public HapticBackend
{
public:
    OpenHapticsBackend();
    ~OpenHapticsBackend() override;

    std::string name() const override { return "touch"; }
    bool loadLibrary(std::string& error) override;
    bool open(const std::string& target, std::string& error) override;
    bool start(ServoCallback callback, std::string& error) override;
    void stop() override;
    void close() override;
    DeviceInfo info() const override;

    struct Api;          // resolved HD function pointers (defined in the .cpp)
    struct ServoAccess;  // gives the scheduler callback access to private members

private:
    std::string lastError() const;

    DynamicLibrary m_lib;
    std::unique_ptr<Api> m_api;
    unsigned int m_device;          // HHD
    unsigned long m_servoHandle;    // HDSchedulerHandle
    bool m_schedulerStarted = false;
    ServoCallback m_callback;
    DeviceInfo m_info;
    double m_maxForce = 0.0;
};

} // namespace universalhaptics
