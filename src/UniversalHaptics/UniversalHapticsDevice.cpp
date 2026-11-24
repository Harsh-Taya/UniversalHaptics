#include <UniversalHaptics/UniversalHapticsDevice.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/accessor.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/UpdateMappingVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalPropagateOnlyPositionAndVelocityVisitor.h>
#include <sofa/type/BoundingBox.h>
#include <sofa/type/SpatialVector.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

namespace universalhaptics
{

namespace
{
long long nowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

/// Forces stay on for this long after the last simulation step, then fade to zero
/// (paused or stalled simulation must not leave a stale contact force on the user's hand).
constexpr long long kForceHoldNs = 200'000'000;
}

UniversalHapticsDevice::UniversalHapticsDevice()
    : d_device(initData(&d_device, std::string("auto"), "device",
        "Which device to use: auto | touch | haption. 'auto' tries detectionOrder and uses the first device that answers."))
    , d_driverName(initData(&d_driverName, std::string(""), "driverName",
        "SofaHAPI-compatible alias of 'device' (Any | Phantom | Haption). Overrides 'device' when set."))
    , d_detectionOrder(initData(&d_detectionOrder, std::string("touch haption"), "detectionOrder",
        "Backends tried by device=\"auto\", in order."))
    , d_touchName(initData(&d_touchName, std::string("Default Device"), "touchName",
        "OpenHaptics device configuration name, as set with Touch_Setup / Touch_HeadlessSetup."))
    , d_haptionAddress(initData(&d_haptionAddress, std::string("127.0.0.1#5000"), "haptionAddress",
        "Address of the Virtuose haptic service, ip#port (portToApi in SvcHaptic_*.conf)."))
    , d_scale(initData(&d_scale, 1.0_sreal, "scale", "Scale from device metres to scene units (e.g. 100 for centimetres)."))
    , d_forceScale(initData(&d_forceScale, 1.0_sreal, "forceScale", "Scale from scene force to newtons sent to the device."))
    , d_torqueScale(initData(&d_torqueScale, 0.0_sreal, "torqueScale", "Scale from scene torque to N.m sent to 6-DOF output devices. 0 disables torque."))
    , d_maxForce(initData(&d_maxForce, 3.0_sreal, "maxForce", "Safety clamp on the force magnitude sent to the device, in newtons (0 = device limit only)."))
    , d_positionBase(initData(&d_positionBase, Vec3(0, 0, 0), "positionBase", "Position of the device base in scene coordinates."))
    , d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the device base in scene coordinates."))
    , d_positionTool(initData(&d_positionTool, Vec3(0, 0, 0), "positionTool", "Position of the tool in the device end-effector frame."))
    , d_orientationTool(initData(&d_orientationTool, Quat(0, 0, 0, 1), "orientationTool", "Orientation of the tool in the device end-effector frame."))
    , d_permanent(initData(&d_permanent, false, "permanent", "Keep rendering forces while the simulation is paused."))
    , d_toolIndex(initData(&d_toolIndex, 0, "toolIndex", "Index of the tool in the local Rigid3 MechanicalObject."))
    , d_writeState(initData(&d_writeState, true, "writeState", "Write the tool pose into the Rigid3 MechanicalObject of this node (SofaHAPI behaviour)."))
    , d_emitButtonEvent(initData(&d_emitButtonEvent, true, "emitButtonEvent", "Propagate a HapticDeviceEvent through the scene when a button changes."))
    , d_drawDevice(initData(&d_drawDevice, false, "drawDevice", "Draw the tool frame."))
    , d_drawHandleSize(initData(&d_drawHandleSize, 0.0f, "drawHandleSize", "Length of the drawn frame axes in scene units (0 = 2 cm of device motion)."))
    , d_drawForceScale(initData(&d_drawForceScale, 0.0f, "drawForceScale", "Draw the output force, multiplied by this factor (0 = off)."))
    , d_positionDevice(initData(&d_positionDevice, "positionDevice", "Tool pose in scene coordinates (output)."))
    , d_button1(initData(&d_button1, false, "button1", "Button 1 state (output)."))
    , d_button2(initData(&d_button2, false, "button2", "Button 2 state (output)."))
    , d_activeDevice(initData(&d_activeDevice, std::string(""), "activeDevice", "Backend in use: touch | haption (output)."))
    , d_deviceInfo(initData(&d_deviceInfo, std::string(""), "deviceInfo", "Model and address reported by the device SDK (output)."))
    , d_outputForce(initData(&d_outputForce, Vec3(0, 0, 0), "outputForce", "Last force computed for the device, scene frame, before forceScale (output)."))
    , d_servoRate(initData(&d_servoRate, 0.0_sreal, "servoRate", "Measured servo loop rate in Hz (output)."))
    , l_forceFeedback(initLink("forceFeedBack", "ForceFeedback component to render. If empty, the first one found from the root is used."))
{
    this->f_listening.setValue(true);
    d_positionDevice.setReadOnly(true);
    d_activeDevice.setReadOnly(true);
    d_deviceInfo.setReadOnly(true);
    d_outputForce.setReadOnly(true);
    d_servoRate.setReadOnly(true);
    d_button1.setReadOnly(true);
    d_button2.setReadOnly(true);
}

UniversalHapticsDevice::~UniversalHapticsDevice()
{
    closeDevice();
}

void UniversalHapticsDevice::init()
{
    d_componentState.setValue(sofa::core::objectmodel::ComponentState::Loading);

    m_mstate = dynamic_cast<sofa::core::behavior::MechanicalState<Rigid3Types>*>(getContext()->getMechanicalState());
    if (d_writeState.getValue() && !m_mstate)
    {
        msg_info() << "No Rigid3 MechanicalObject in this node: the tool pose is only available through 'positionDevice'.";
    }

    if (!l_forceFeedback.empty())
        m_forceFeedback = l_forceFeedback.get();
    else
    {
        auto* node = dynamic_cast<sofa::simulation::Node*>(getContext());
        m_forceFeedback = node ? node->get<ForceFeedback>(sofa::core::objectmodel::BaseContext::SearchRoot) : nullptr;
    }
    if (m_forceFeedback)
    {
        msg_info() << "Rendering force feedback from " << m_forceFeedback->getPathName();
    }
    else
    {
        msg_warning() << "No ForceFeedback component found: the device will move the tool but render no force.";
    }

    updateParams();
}

void UniversalHapticsDevice::bwdInit()
{
    if (!m_backend) openDevice();
}

void UniversalHapticsDevice::reinit()
{
    updateParams();
}

void UniversalHapticsDevice::cleanup()
{
    closeDevice();
    Inherit1::cleanup();
}

void UniversalHapticsDevice::updateParams()
{
    Quat qb = d_orientationBase.getValue();
    qb.normalize();
    Quat qt = d_orientationTool.getValue();
    qt.normalize();

    Params p;
    p.world_H_base.set(d_positionBase.getValue(), qb);
    p.end_H_tool.set(d_positionTool.getValue(), qt);
    p.scale = d_scale.getValue();
    p.forceScale = d_forceScale.getValue();
    p.torqueScale = d_torqueScale.getValue();
    p.maxForce = d_maxForce.getValue();
    p.permanent = d_permanent.getValue();

    std::lock_guard<std::mutex> lock(m_paramMutex);
    m_params = p;
}

std::vector<std::string> UniversalHapticsDevice::detectionOrder() const
{
    std::string choice = lower(d_device.getValue());
    if (d_driverName.isSet() && !d_driverName.getValue().empty())
    {
        const std::string drv = lower(d_driverName.getValue());
        if (drv == "any") choice = "auto";
        else if (drv == "phantom" || drv == "geomagic" || drv == "touch" || drv == "openhaptics") choice = "touch";
        else if (drv == "haption" || drv == "virtuose") choice = "haption";
        else choice = drv;
    }
    if (choice != "auto" && !choice.empty())
        return { choice };

    std::vector<std::string> order;
    std::string token;
    std::istringstream in(d_detectionOrder.getValue());
    while (in >> token)
    {
        token.erase(std::remove(token.begin(), token.end(), ','), token.end());
        if (!token.empty()) order.push_back(lower(token));
    }
    return order.empty() ? availableBackendNames() : order;
}

bool UniversalHapticsDevice::openDevice()
{
    closeDevice();

    const auto order = detectionOrder();
    std::vector<std::string> log;
    m_backend = detectDevice(order,
        [this](const std::string& backend) {
            return backend == "touch" ? d_touchName.getValue() : d_haptionAddress.getValue();
        }, log);

    std::ostringstream report;
    for (const auto& line : log) report << "\n  " << line;

    if (!m_backend)
    {
        msg_error() << "No haptic device found (tried: " << d_device.getValue()
                    << (d_driverName.isSet() ? " / driverName=" + d_driverName.getValue() : std::string()) << ")"
                    << report.str();
        d_activeDevice.setValue("");
        d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return false;
    }

    const DeviceInfo info = m_backend->info();
    msg_info() << "Haptic device detection:" << report.str();

    std::string error;
    if (!m_backend->start([this](const DeviceState& in, DeviceOutput& out) { servoTick(in, out); }, error))
    {
        msg_error() << "Could not start the " << info.backend << " servo loop: " << error;
        m_backend->close();
        m_backend.reset();
        d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return false;
    }

    d_activeDevice.setValue(info.backend);
    d_deviceInfo.setValue(info.model + " [" + info.identifier + "]");
    m_lastRateNs = nowNs();
    m_ticksAtLastRate = m_ticks;

    // Give the servo loop a moment to publish a first pose before the scene reads it.
    for (int i = 0; i < 100; ++i)
    {
        {
            std::lock_guard<std::mutex> lock(m_sharedMutex);
            if (m_shared.valid) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    updateFromDevice();

    d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
    return true;
}

void UniversalHapticsDevice::closeDevice()
{
    if (!m_backend) return;
    m_backend->stop();   // joins the servo thread: no servoTick runs after this
    m_backend->close();
    m_backend.reset();
    std::lock_guard<std::mutex> lock(m_sharedMutex);
    m_shared.valid = false;
}

// Runs in the device's servo thread (~1 kHz). No SOFA Data access here.
void UniversalHapticsDevice::servoTick(const DeviceState& in, DeviceOutput& out)
{
    ++m_ticks;

    Params p;
    {
        std::lock_guard<std::mutex> lock(m_paramMutex);
        p = m_params;
    }

    const Vec3 pos(in.position[0] * p.scale, in.position[1] * p.scale, in.position[2] * p.scale);
    const Quat q(in.orientation[0], in.orientation[1], in.orientation[2], in.orientation[3]);
    const Transform base_H_end(pos, q);
    const Transform world_H_tool = p.world_H_base * base_H_end * p.end_H_tool;

    Vec3 forceWorld(0, 0, 0), torqueWorld(0, 0, 0);
    const bool simulationRunning = (nowNs() - m_lastAnimateNs.load()) < kForceHoldNs;
    if (m_forceFeedback && in.motorsReady && (p.permanent || simulationRunning))
    {
        sofa::type::SpatialVector<SReal> twist(Vec3(0, 0, 0), Vec3(0, 0, 0));
        sofa::type::SpatialVector<SReal> wrench(Vec3(0, 0, 0), Vec3(0, 0, 0));
        m_forceFeedback->computeWrench(world_H_tool, twist, wrench);
        forceWorld = wrench.getForce();
        torqueWorld = wrench.getTorque();
    }

    // scene frame -> device base frame, then to physical units
    const Quat& qBase = p.world_H_base.getOrientation();
    Vec3 f = qBase.inverseRotate(forceWorld) * p.forceScale;
    Vec3 t = qBase.inverseRotate(torqueWorld) * p.torqueScale;

    bool finite = true;
    for (int i = 0; i < 3; ++i) finite = finite && std::isfinite(f[i]) && std::isfinite(t[i]);
    if (!finite) { f = Vec3(0, 0, 0); t = Vec3(0, 0, 0); }

    const SReal n = f.norm();
    if (p.maxForce > 0 && n > p.maxForce) f *= p.maxForce / n;

    out.force = { f[0], f[1], f[2] };
    out.torque = { t[0], t[1], t[2] };

    std::lock_guard<std::mutex> lock(m_sharedMutex);
    m_shared.device = in;
    m_shared.world_H_tool = world_H_tool;
    m_shared.forceWorld = forceWorld;
    m_shared.valid = true;
}

void UniversalHapticsDevice::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {
        m_lastAnimateNs = nowNs();
        if (m_backend) updateFromDevice();
    }
    Inherit1::handleEvent(event);
}

// Runs in the simulation thread at the start of every time step.
void UniversalHapticsDevice::updateFromDevice()
{
    Shared s;
    {
        std::lock_guard<std::mutex> lock(m_sharedMutex);
        s = m_shared;
    }
    if (!s.valid) return;

    Coord c;
    c.getCenter() = s.world_H_tool.getOrigin();
    c.getOrientation() = s.world_H_tool.getOrientation();
    d_positionDevice.setValue(c);
    d_outputForce.setValue(s.forceWorld);
    d_button1.setValue((s.device.buttons & 1u) != 0);
    d_button2.setValue((s.device.buttons & 2u) != 0);

    const long long now = nowNs();
    if (now - m_lastRateNs > 1'000'000'000LL)
    {
        const long ticks = m_ticks;
        d_servoRate.setValue(SReal(ticks - m_ticksAtLastRate) * 1e9 / SReal(now - m_lastRateNs));
        m_ticksAtLastRate = ticks;
        m_lastRateNs = now;
        msg_info_when(f_printLog.getValue()) << "servo " << int(d_servoRate.getValue()) << " Hz"
            << " | tool " << c.getCenter()
            << " | force (scene) " << s.forceWorld << " |F|=" << s.forceWorld.norm()
            << " | buttons " << s.device.buttons << (s.device.motorsReady ? "" : " | motors off (dead-man)");
    }

    if (m_forceFeedback)
    {
        Transform ref = s.world_H_tool;
        m_forceFeedback->setReferencePosition(ref);
    }

    // SofaHAPI behaviour: drive the local Rigid3 MechanicalObject directly.
    // Unlike SofaHAPI, every vector is size-checked first: SOFA creates an EMPTY
    // freePosition vector when the scene has no free-motion loop, and writing
    // index 0 of it is what made SofaHAPI segfault.
    if (d_writeState.getValue() && m_mstate)
    {
        const auto idx = static_cast<std::size_t>(std::max(0, d_toolIndex.getValue()));
        sofa::helper::WriteAccessor<sofa::Data<Rigid3Types::VecCoord>> x =
            *m_mstate->write(sofa::core::vec_id::write_access::position);
        if (idx < x.size())
        {
            x[idx] = c;
            sofa::helper::WriteAccessor<sofa::Data<Rigid3Types::VecCoord>> xfree =
                *m_mstate->write(sofa::core::vec_id::write_access::freePosition);
            if (idx < xfree.size()) xfree[idx] = c;
        }
        else if (!m_warnedStateSize)
        {
            msg_warning() << "toolIndex " << idx << " is outside the MechanicalObject (size " << x.size()
                          << "); the pose is still published in 'positionDevice'.";
            m_warnedStateSize = true;
        }

        if (idx < x.size())
        {
            sofa::simulation::mechanicalvisitor::MechanicalPropagateOnlyPositionAndVelocityVisitor
                propagate(sofa::core::mechanicalparams::defaultInstance());
            propagate.execute(getContext());
            sofa::simulation::UpdateMappingVisitor updateMappings(sofa::core::execparams::defaultInstance());
            updateMappings.execute(getContext());
        }
    }

    if (d_emitButtonEvent.getValue() && s.device.buttons != m_lastButtons)
    {
        sofa::core::objectmodel::HapticDeviceEvent event(
            static_cast<unsigned int>(std::max(0, d_toolIndex.getValue())),
            c.getCenter(), c.getOrientation(), static_cast<unsigned char>(s.device.buttons));
        getContext()->getRootContext()->propagateEvent(sofa::core::execparams::defaultInstance(), &event);
    }
    m_lastButtons = s.device.buttons;
}

void UniversalHapticsDevice::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid) return;
    if (!d_drawDevice.getValue() && d_drawForceScale.getValue() == 0.0f) return;

    auto* dt = vparams->drawTool();
    dt->saveLastState();
    dt->disableLighting();

    const Coord& c = d_positionDevice.getValue();
    const Vec3 o = c.getCenter();
    const float len = d_drawHandleSize.getValue() > 0 ? d_drawHandleSize.getValue()
                                                      : static_cast<float>(0.02 * d_scale.getValue());
    if (d_drawDevice.getValue())
    {
        const float r = len * 0.05f;
        dt->drawArrow(o, o + c.getOrientation().rotate(Vec3(len, 0, 0)), r, sofa::type::RGBAColor::red());
        dt->drawArrow(o, o + c.getOrientation().rotate(Vec3(0, len, 0)), r, sofa::type::RGBAColor::green());
        dt->drawArrow(o, o + c.getOrientation().rotate(Vec3(0, 0, len)), r, sofa::type::RGBAColor::blue());
    }
    if (d_drawForceScale.getValue() != 0.0f)
    {
        const Vec3 f = d_outputForce.getValue() * d_drawForceScale.getValue();
        if (f.norm() > 1e-9)
            dt->drawArrow(o, o + f, len * 0.05f, sofa::type::RGBAColor::yellow());
    }
    dt->restoreLastState();
}

void UniversalHapticsDevice::computeBBox(const sofa::core::ExecParams*, bool)
{
    const Vec3 o = d_positionDevice.getValue().getCenter();
    const SReal r = 0.05 * d_scale.getValue();
    f_bbox.setValue(sofa::type::BoundingBox(Vec3(o[0] - r, o[1] - r, o[2] - r), Vec3(o[0] + r, o[1] + r, o[2] + r)));
}

void registerUniversalHapticsDevice(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
        "Haptic device driver for the Geomagic Touch (OpenHaptics) and the Haption Virtuose (VirtuoseAPI), "
        "selected automatically at runtime. Scene interface follows SofaHAPIHapticsDevice.")
        .add<UniversalHapticsDevice>());
}

} // namespace universalhaptics
