/******************************************************************************
*  UniversalHaptics -- plugin entry points                                    *
******************************************************************************/
#include <UniversalHaptics/config.h>
#include <UniversalHaptics/backend/HapticBackend.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>

namespace universalhaptics
{
extern void registerUniversalHapticsDevice(sofa::core::ObjectFactory* factory);

extern "C" {
    SOFA_UNIVERSALHAPTICS_API void initExternalModule();
    SOFA_UNIVERSALHAPTICS_API const char* getModuleName();
    SOFA_UNIVERSALHAPTICS_API const char* getModuleVersion();
    SOFA_UNIVERSALHAPTICS_API const char* getModuleLicense();
    SOFA_UNIVERSALHAPTICS_API const char* getModuleDescription();
    SOFA_UNIVERSALHAPTICS_API void registerObjects(sofa::core::ObjectFactory* factory);
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
        sofa::helper::system::PluginManager::getInstance().registerPlugin(MODULE_NAME);
    }
}

const char* getModuleName() { return MODULE_NAME; }
const char* getModuleVersion() { return MODULE_VERSION; }
const char* getModuleLicense() { return "LGPL"; }
const char* getModuleDescription()
{
    return "One haptic device component for the Geomagic Touch (OpenHaptics) and the Haption Virtuose "
           "(VirtuoseAPI), detected at runtime on Linux and Windows. Vendor SDKs are loaded dynamically.";
}

void registerObjects(sofa::core::ObjectFactory* factory)
{
    registerUniversalHapticsDevice(factory);
}
} // namespace universalhaptics
