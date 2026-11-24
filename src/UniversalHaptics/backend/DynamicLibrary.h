/******************************************************************************
*  UniversalHaptics -- runtime loading of vendor SDKs (Linux + Windows)       *
******************************************************************************/
#pragma once

#include <string>
#include <vector>

namespace universalhaptics
{

/// Minimal dlopen / LoadLibrary wrapper. Tries each candidate file name in turn.
class DynamicLibrary
{
public:
    DynamicLibrary() = default;
    ~DynamicLibrary();
    DynamicLibrary(const DynamicLibrary&) = delete;
    DynamicLibrary& operator=(const DynamicLibrary&) = delete;

    /// Returns true on the first candidate that loads. On failure `error` lists every attempt.
    bool load(const std::vector<std::string>& candidates, std::string& error);
    bool isLoaded() const { return m_handle != nullptr; }
    const std::string& loadedName() const { return m_name; }

    /// Resolve a symbol; nullptr if missing.
    void* symbol(const char* name) const;

    /// Resolve into a typed function pointer. Returns false (and appends to `missing`) if absent.
    template <class Fn>
    bool resolve(Fn& out, const char* name, std::string& missing) const
    {
        out = reinterpret_cast<Fn>(symbol(name));
        if (!out)
        {
            if (!missing.empty()) missing += ", ";
            missing += name;
        }
        return out != nullptr;
    }

    void unload();

private:
    void* m_handle = nullptr;
    std::string m_name;
};

} // namespace universalhaptics
