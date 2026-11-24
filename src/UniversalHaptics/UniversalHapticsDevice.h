/******************************************************************************
*  UniversalHaptics -- device-independent haptics for SOFA                    *
*                                                                             *
*  UniversalHapticsDevice: one component for the Geomagic Touch (OpenHaptics) *
*  and the Haption Virtuose (VirtuoseAPI), chosen automatically at runtime.   *
*  Scene interface follows SofaHAPIHapticsDevice so SofaHAPI scenes port by   *
*  renaming the component.                                                    *
******************************************************************************/
#pragma once

#include <UniversalHaptics/config.h>
#include <UniversalHaptics/backend/HapticBackend.h>

#include <sofa/component/controller/Controller.h>
#include <sofa/component/haptics/ForceFeedback.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/type/Transform.h>

#include <atomic>
#include <memory>
#include <mutex>

namespace universalhaptics
{

class SOFA_UNIVERSALHAPTICS_API UniversalHapticsDevice : public sofa::component::controller::Controller
{
public:
    SOFA_CLASS(UniversalHapticsDevice, sofa::component::controller::Controller);

    using Rigid3Types = sofa::defaulttype::Rigid3Types;
    using Coord = Rigid3Types::Coord;
    using Vec3 = sofa::type::Vec3;
    using Quat = sofa::type::Quat<SReal>;
    using Transform = sofa::type::Transform<SReal>;
    using ForceFeedback = sofa::component::haptics::ForceFeedback;

    // ---------------------------------------------------------------- device selection
    sofa::Data<std::string> d_device;          ///< auto | touch | haption
    sofa::Data<std::string> d_driverName;      ///< SofaHAPI-compatible alias: Any | Phantom | Haption
    sofa::Data<std::string> d_detectionOrder;  ///< order tried by device="auto"
    sofa::Data<std::string> d_touchName;       ///< OpenHaptics configuration name ("" = default device)
    sofa::Data<std::string> d_haptionAddress;  ///< "ip#port" of the Virtuose haptic service

    // ---------------------------------------------------------------- SofaHAPI-compatible parameters
    sofa::Data<SReal> d_scale;           ///< device metres -> scene units
    sofa::Data<SReal> d_forceScale;      ///< scene force -> newtons sent to the device
    sofa::Data<SReal> d_torqueScale;     ///< scene torque -> N.m (6-DOF output devices only; 0 = off)
    sofa::Data<SReal> d_maxForce;        ///< safety clamp on |force| in newtons (0 = device limit only)
    sofa::Data<Vec3>  d_positionBase;    ///< device base position in the scene
    sofa::Data<Quat>  d_orientationBase; ///< device base orientation in the scene
    sofa::Data<Vec3>  d_positionTool;    ///< tool tip position in the device end-effector frame
    sofa::Data<Quat>  d_orientationTool; ///< tool orientation in the device end-effector frame
    sofa::Data<bool>  d_permanent;       ///< keep sending forces while the simulation is paused
    sofa::Data<int>   d_toolIndex;       ///< index written in the local Rigid3 MechanicalObject
    sofa::Data<bool>  d_writeState;      ///< write the tool pose into the local MechanicalObject (SofaHAPI behaviour)
    sofa::Data<bool>  d_emitButtonEvent; ///< propagate HapticDeviceEvent when buttons change
    sofa::Data<bool>  d_drawDevice;      ///< draw the tool frame
    sofa::Data<float> d_drawHandleSize;  ///< length of the drawn frame axes (scene units, 0 = auto)
    sofa::Data<float> d_drawForceScale;  ///< draw the output force, scaled (0 = off)

    // ---------------------------------------------------------------- outputs
    sofa::Data<Coord> d_positionDevice;  ///< tool pose in the scene (link MechanicalObject.position to it)
    sofa::Data<bool>  d_button1;
    sofa::Data<bool>  d_button2;
    sofa::Data<std::string> d_activeDevice; ///< backend actually in use ("" if none)
    sofa::Data<std::string> d_deviceInfo;   ///< model / address reported by the SDK
    sofa::Data<Vec3>  d_outputForce;     ///< last force sent, scene frame (before forceScale)
    sofa::Data<SReal> d_servoRate;       ///< measured servo loop rate, Hz

    sofa::SingleLink<UniversalHapticsDevice, ForceFeedback,
                     sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK> l_forceFeedback;

    void init() override;
    void bwdInit() override;
    void reinit() override;
    void cleanup() override;
    void handleEvent(sofa::core::objectmodel::Event* event) override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;
    void computeBBox(const sofa::core::ExecParams* params, bool onlyVisible = false) override;

    /// Open (or re-open) the device; called automatically by bwdInit().
    bool openDevice();
    /// Stop the servo loop and release the device.
    void closeDevice();

protected:
    UniversalHapticsDevice();
    ~UniversalHapticsDevice() override;

private:
    struct Params
    {
        Transform world_H_base;
        Transform end_H_tool;
        SReal scale = 1, forceScale = 1, torqueScale = 0, maxForce = 0;
        bool permanent = false;
    };

    struct Shared
    {
        DeviceState device;
        Transform world_H_tool;
        Vec3 forceWorld;
        bool valid = false;
    };

    void updateParams();
    void servoTick(const DeviceState& in, DeviceOutput& out);
    void updateFromDevice();
    std::vector<std::string> detectionOrder() const;

    std::unique_ptr<HapticBackend> m_backend;
    ForceFeedback* m_forceFeedback = nullptr;
    sofa::core::behavior::MechanicalState<Rigid3Types>* m_mstate = nullptr;

    std::mutex m_paramMutex;
    Params m_params;
    std::mutex m_sharedMutex;
    Shared m_shared;

    std::atomic<long long> m_lastAnimateNs {0};
    std::atomic<long> m_ticks {0};
    long m_ticksAtLastRate = 0;
    long long m_lastRateNs = 0;
    unsigned int m_lastButtons = 0;
    bool m_warnedStateSize = false;
};

} // namespace universalhaptics
