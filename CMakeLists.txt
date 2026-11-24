cmake_minimum_required(VERSION 3.22)
project(UniversalHaptics VERSION 0.1)

# Device-independent haptics for SOFA: Geomagic Touch (OpenHaptics) + Haption Virtuose (VirtuoseAPI).
# No vendor SDK is needed at build time: both are loaded at runtime (dlopen / LoadLibrary).

find_package(Sofa.Config REQUIRED)
sofa_find_package(Sofa.Component.Controller REQUIRED)
sofa_find_package(Sofa.Component.Haptics REQUIRED)
sofa_find_package(Sofa.Component.StateContainer REQUIRED)
sofa_find_package(Sofa.Simulation.Core REQUIRED)
find_package(Threads REQUIRED)

option(UNIVERSALHAPTICS_BUILD_PROBE "Build uh_probe, a command-line device test that does not need SOFA" ON)

set(SRC_DIR src/UniversalHaptics)

set(BACKEND_HEADERS
    ${SRC_DIR}/backend/HapticBackend.h
    ${SRC_DIR}/backend/DynamicLibrary.h
    ${SRC_DIR}/backend/OpenHapticsBackend.h
    ${SRC_DIR}/backend/VirtuoseBackend.h
)
set(BACKEND_SOURCES
    ${SRC_DIR}/backend/HapticBackend.cpp
    ${SRC_DIR}/backend/DynamicLibrary.cpp
    ${SRC_DIR}/backend/OpenHapticsBackend.cpp
    ${SRC_DIR}/backend/VirtuoseBackend.cpp
)
set(HEADER_FILES
    ${SRC_DIR}/config.h.in
    ${SRC_DIR}/UniversalHapticsDevice.h
    ${BACKEND_HEADERS}
)
set(SOURCE_FILES
    ${SRC_DIR}/initUniversalHaptics.cpp
    ${SRC_DIR}/UniversalHapticsDevice.cpp
    ${BACKEND_SOURCES}
)
set(README_FILES README.md)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
target_link_libraries(${PROJECT_NAME}
    Sofa.Component.Controller
    Sofa.Component.Haptics
    Sofa.Component.StateContainer
    Sofa.Simulation.Core
    Threads::Threads
    ${CMAKE_DL_LIBS})
if(WIN32)
    target_link_libraries(${PROJECT_NAME} winmm)   # timeBeginPeriod for a 1 ms servo period
endif()

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
)

if(UNIVERSALHAPTICS_BUILD_PROBE)
    # Same backend code, no SOFA: checks that a device is detected and the servo loop runs.
    add_executable(uh_probe tools/uh_probe.cpp ${BACKEND_SOURCES})
    target_include_directories(uh_probe PRIVATE src)
    target_compile_features(uh_probe PRIVATE cxx_std_17)
    target_link_libraries(uh_probe PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
    if(WIN32)
        target_link_libraries(uh_probe PRIVATE winmm)
    endif()
    install(TARGETS uh_probe RUNTIME DESTINATION bin COMPONENT applications)
endif()
