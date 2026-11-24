#include <UniversalHaptics/backend/DynamicLibrary.h>

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

namespace universalhaptics
{

DynamicLibrary::~DynamicLibrary()
{
    unload();
}

bool DynamicLibrary::load(const std::vector<std::string>& candidates, std::string& error)
{
    unload();
    error.clear();
    for (const auto& name : candidates)
    {
#if defined(_WIN32)
        HMODULE h = ::LoadLibraryA(name.c_str());
        if (h)
        {
            m_handle = reinterpret_cast<void*>(h);
            m_name = name;
            return true;
        }
        error += (error.empty() ? "" : "; ") + name + " (error " + std::to_string(::GetLastError()) + ")";
#else
        void* h = ::dlopen(name.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (h)
        {
            m_handle = h;
            m_name = name;
            return true;
        }
        const char* why = ::dlerror();
        error += (error.empty() ? "" : "; ") + std::string(why ? why : name.c_str());
#endif
    }
    return false;
}

void* DynamicLibrary::symbol(const char* name) const
{
    if (!m_handle) return nullptr;
#if defined(_WIN32)
    return reinterpret_cast<void*>(::GetProcAddress(reinterpret_cast<HMODULE>(m_handle), name));
#else
    return ::dlsym(m_handle, name);
#endif
}

void DynamicLibrary::unload()
{
    if (!m_handle) return;
#if defined(_WIN32)
    ::FreeLibrary(reinterpret_cast<HMODULE>(m_handle));
#else
    ::dlclose(m_handle);
#endif
    m_handle = nullptr;
    m_name.clear();
}

} // namespace universalhaptics
