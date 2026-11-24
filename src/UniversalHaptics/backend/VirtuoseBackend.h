/******************************************************************************
*  UniversalHaptics -- Haption Virtuose backend (VirtuoseAPI)                 *
******************************************************************************/
#pragma once

#include <UniversalHaptics/backend/HapticBackend.h>
#include <UniversalHaptics/backend/DynamicLibrary.h>

#include <atomic>
#include <thread>

namespace universalhaptics
{

class VirtuoseBackend : public HapticBackend
{
public:
    VirtuoseBackend();
    ~VirtuoseBackend() override;

    std::string name() const override { return "haption"; }
    bool loadLibrary(std::string& error) override;
    bool open(const std::string& target, std::string& error) override;
    bool start(ServoCallback callback, std::string& error) override;
    void stop() override;
    void close() override;
    DeviceInfo info() const override;

    /// Servo period. VirtuoseAPI is told the same value via virtSetTimeStep.
    static constexpr double kTimeStep = 0.001;

    struct Api;

private:
    void servoLoop();
    std::string errorMessage() const;

    DynamicLibrary m_lib;
    std::unique_ptr<Api> m_api;
    void* m_context = nullptr;   // VirtContext
    std::thread m_thread;
    std::atomic<bool> m_running {false};
    bool m_ignoreDeadMan = false;
    ServoCallback m_callback;
    DeviceInfo m_info;
};

} // namespace universalhaptics
