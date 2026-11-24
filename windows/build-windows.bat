@echo off
REM Builds UniversalHaptics against the SOFA v26.06 Windows binary release (no SOFA source build needed).
REM See README.md, section "Build on Windows". Set these paths first (or set them as environment variables):
REM   SOFA_ROOT  folder of the unpacked SOFA_v26.06.00_Win64 release (contains bin\runSofa.exe)
REM   EIGEN_DIR  folder of the unpacked Eigen 3.4.0 (contains Eigen\Core)
REM   BOOST_DIR  folder of the unpacked Boost 1.74.0 (contains boost\version.hpp)
REM Optional:
REM   VCVARS_VER  MSVC toolset version to use, e.g. 14.29 (leave unset for the newest installed)
setlocal
if not defined SOFA_ROOT set "SOFA_ROOT=C:\SOFA_v26.06.00_Win64"
if not defined EIGEN_DIR set "EIGEN_DIR=C:\deps\eigen-3.4.0"
if not defined BOOST_DIR set "BOOST_DIR=C:\deps\boost_1_74_0"

set "REPO=%~dp0.."
set "BUILD=%REPO%\build-windows"

if not exist "%SOFA_ROOT%\bin\runSofa.exe" (echo SOFA_ROOT is wrong: %SOFA_ROOT% & exit /b 1)
if not exist "%EIGEN_DIR%\Eigen\Core"      (echo EIGEN_DIR is wrong: %EIGEN_DIR% & exit /b 1)
if not exist "%BOOST_DIR%\boost\version.hpp" (echo BOOST_DIR is wrong: %BOOST_DIR% & exit /b 1)

echo === 1/4 Visual Studio C++ compiler (x64)
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
set "VSDIR="
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -property installationPath`) do set "VSDIR=%%i"
if not defined VSDIR (echo Visual Studio not found & exit /b 1)
if defined VCVARS_VER (
  call "%VSDIR%\VC\Auxiliary\Build\vcvarsall.bat" x64 -vcvars_ver=%VCVARS_VER%
) else (
  call "%VSDIR%\VC\Auxiliary\Build\vcvarsall.bat" x64
)
where cl >nul 2>&1 || (echo cl.exe not available: install the "Desktop development with C++" workload, or set VCVARS_VER & exit /b 1)

REM CMake wants forward slashes
set "SOFA_F=%SOFA_ROOT:\=/%"
set "EIGEN_F=%EIGEN_DIR:\=/%"
set "BOOST_F=%BOOST_DIR:\=/%"
set "REPO_F=%REPO:\=/%"

echo === 2/4 CMake configure
cmake -S "%REPO%" -B "%BUILD%" -G "NMake Makefiles" ^
  -DCMAKE_BUILD_TYPE=Release ^
  "-DCMAKE_PREFIX_PATH=%SOFA_F%/lib/cmake" ^
  "-DEIGEN3_INCLUDE_DIR=%EIGEN_F%" ^
  "-DBoost_DIR=%REPO_F%/windows/cmake" ^
  "-DBOOST_INCLUDE_DIR=%BOOST_F%" ^
  -DCMAKE_POLICY_DEFAULT_CMP0167=NEW ^
  "-DCMAKE_MAP_IMPORTED_CONFIG_RELEASE=MinSizeRel;Release" || exit /b 2

echo === 3/4 Compile
cmake --build "%BUILD%" || exit /b 3

echo === 4/4 Copy UniversalHaptics.dll next to runSofa.exe
set "FOUND="
for /r "%BUILD%" %%f in (UniversalHaptics.dll) do if exist "%%f" (
  copy /Y "%%f" "%SOFA_ROOT%\bin\" >nul && set "FOUND=1"
)
if not defined FOUND (echo UniversalHaptics.dll not found in %BUILD% & exit /b 4)
echo.
echo Done: %SOFA_ROOT%\bin\UniversalHaptics.dll
echo uh_probe.exe is in %BUILD%
exit /b 0
