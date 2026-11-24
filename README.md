# UniversalHaptics

A SOFA plugin that drives the **Geomagic Touch** and the **Haption Virtuose** through **one** component, `UniversalHapticsDevice`. It finds whichever device is connected at runtime, on **Linux and Windows**.

It sits at the same level as SOFA's Geomagic plugin and SofaHAPI, and uses the same scene fields as `SofaHAPIHapticsDevice`. It does **not** need HAPI, H3DUtil or a lab fork of SOFA: it builds against standard upstream **SOFA v26.06**.

B.Tech project, IIT Delhi (2026). Supervisors: Prof. Subodh Kumar, Prof. Anil Sharma.

## Test status

Tested by hand with the real devices, October–November 2026, SOFA v26.06:

| | Geomagic Touch | Haption Virtuose Desktop 6D |
|---|---|---|
| **Linux** (Ubuntu 24.04) | ✅ device test (`uh_probe`): ~1020 Hz, spring force felt<br>✅ SOFA scene: tool follows the stylus, contact force felt | ✅ device test (`uh_probe`): ~1000 Hz, spring force felt<br>✅ SOFA scene: contact force felt |
| **Windows** (10/11, MSVC) | ✅ device test (`uh_probe`): ~940–1000 Hz, spring force felt<br>✅ SOFA scene: tool follows the stylus, contact force felt | ⚠️ **not yet run with the device.** The Haption code builds on Windows, and everything it shares with the Touch (SOFA component, runtime DLL loading, servo loop) is tested there. Not yet run: the Haption-specific part (`virtuoseAPI.dll`, the Windows haptic service) |

Before relying on it, see **Known issues** below.

Versions used:
- **SOFA:** v26.06 (Linux: built from source; Windows: the official `SOFA_v26.06.00_Win64` release)
- **Touch:** OpenHaptics 3.5.0; Touch Device Driver 2025.12.10 (Linux), 2023.10.11 (Windows)
- **Haption:** VirtuoseAPI 4.04, haptic service 6.9 (Desktop 6D)
- **Windows build:** MSVC 14.29, CMake 4.2, Eigen 3.4.0, Boost 1.74.0

## How it works

```
SOFA scene
  └─ UniversalHapticsDevice   SOFA component: tool pose into the scene, contact force (ForceFeedback) out
        │
        ▼
     HapticBackend           plain C++17, no SOFA; common frame: metres, X right, Y up, Z towards the user
        ├─ OpenHapticsBackend  → hd.dll / libHD.so                 → Geomagic Touch
        └─ VirtuoseBackend     → virtuoseAPI.dll / libvirtuoseAPI.so → Haption Virtuose
```

Both vendor libraries are loaded **at runtime** (`LoadLibrary` / `dlopen`). So:
- the plugin **compiles without any vendor SDK installed**
- one build works on a PC with only OpenHaptics, only VirtuoseAPI, or both
- `device="auto"` tries each backend and uses the first device that answers

(SOFA's own Geomagic plugin links OpenHaptics at build time. In the official SOFA v26.06 Windows release it was built without OpenHaptics, so it cannot drive a Touch there. This plugin can.)

## Repository contents

| Path | What |
|---|---|
| `src/UniversalHaptics/UniversalHapticsDevice.*` | the SOFA component |
| `src/UniversalHaptics/initUniversalHaptics.cpp`, `config.h.in` | plugin registration |
| `src/UniversalHaptics/backend/` | device layer: `HapticBackend` (interface + auto-detection), `DynamicLibrary` (runtime loading), `OpenHapticsBackend` (Touch), `VirtuoseBackend` (Haption) |
| `tools/uh_probe.cpp` | command-line device test, **no SOFA needed** |
| `scenes/UniversalHaptics-RigidSphere.scn` | example: tool touching a sphere (tested with the Touch) |
| `scenes/UniversalHaptics-RigidSphere-Haption.scn` | the same, with the workspace offset the Haption needs |
| `windows/` | helper script and a small Boost CMake file for the Windows build |
| `CMakeLists.txt`, `UniversalHapticsConfig.cmake.in` | build |

**Not included** (proprietary; get them from the vendors): OpenHaptics, the Touch driver, VirtuoseAPI (`virtuoseAPI.dll` / `libvirtuoseAPI.so`), Haption's haptic service.

## 1. Install the device software

**Geomagic Touch** (3D Systems support article "Haptic Device Drivers"):
1. Install the **Touch Device Driver**, then **OpenHaptics** (3.5).
2. Pair the Touch with **Touch Smart Setup** (Windows) or **Touch_HeadlessSetup** (Linux). Keep the name **`Default Device`**.
3. Check it with the vendor's diagnostic tool (Touch Diagnostic / TouchCheckup), then close that tool.
4. Ethernet Touch: the PC port needs a **link-local** address (169.254.x.x). Windows does this by itself on an enabled port. On Linux, use a NetworkManager profile with `ipv4.method link-local`.

**Haption Virtuose** (files from Haption for your unit):
1. Install VirtuoseAPI and the **haptic service** (`SvcHaptic_<model>`).
2. Give the PC's Ethernet port a **static IP** on the controller's subnet. The controller defaults to 192.168.100.53, so use e.g. 192.168.100.61 / 255.255.255.0.
3. After every power-up: **calibrate** (the controller display then shows `71`), start the service, and press the **Force Feedback button** on the arm to turn the motors on.
4. Note the service's **API port**: `portToApi` is 5000 on our Linux install; Haption's manual gives 53210 as the Windows default.

## 2. Build on Linux (inside the SOFA source tree; this is how it was tested)

1. Put this folder at `sofa/src/applications/plugins/UniversalHaptics`.
2. Add to `sofa/src/applications/plugins/CMakeLists.txt`:
   ```cmake
   sofa_add_subdirectory(plugin UniversalHaptics UniversalHaptics)
   ```
3. Reconfigure SOFA with `-DPLUGIN_UNIVERSALHAPTICS=ON` and build. You get `lib/libUniversalHaptics.so` and `bin/uh_probe`.

## 3. Build on Windows (against the SOFA binary release; this is how it was tested)

No SOFA source build is needed.

1. **Tools:** Visual Studio 2019 or 2022 with **"Desktop development with C++"**, and **CMake ≥ 3.22**. SOFA v26.06 needs C++20, which MSVC 14.29 and newer support.
2. **SOFA:** download `SOFA_v26.06.00_Win64.zip` from the SOFA v26.06.00 GitHub release, and unzip it, e.g. to `C:\SOFA_v26.06.00_Win64`.
3. **Eigen 3.4.0 and Boost 1.74.0** (header files only). The SOFA zip doesn't include them, but its headers use them:
   - `eigen-3.4.0.zip` from gitlab.com/libeigen/eigen
   - `boost_1_74_0.zip` from archives.boost.io

   Unpack them with `tar`, which is much faster than Explorer for Boost's ~15,000 header files:
   ```bat
   mkdir C:\deps
   tar -xf eigen-3.4.0.zip -C C:\deps
   tar -xf boost_1_74_0.zip -C C:\deps boost_1_74_0/boost
   ```
4. **Build:** in cmd, set the paths if they differ from the defaults shown, then run the script:
   ```bat
   set SOFA_ROOT=C:\SOFA_v26.06.00_Win64
   set EIGEN_DIR=C:\deps\eigen-3.4.0
   set BOOST_DIR=C:\deps\boost_1_74_0
   set VCVARS_VER=14.29          (only if your newest MSVC toolset is incomplete, as on our lab PC)
   windows\build-windows.bat
   ```
   It finds Visual Studio, runs CMake (NMake, Release), compiles, and copies `UniversalHaptics.dll` into `SOFA_ROOT\bin`, so `runSofa.exe` finds it. `uh_probe.exe` ends up in `build-windows\`.

   `windows/cmake/BoostConfig.cmake` exists because CMake 4.x no longer ships its own FindBoost, and a plain Boost download has no CMake files. It provides the header-only `Boost::boost` target that SOFA asks for.

   This script is a cleaned-up version of the one used for the 4 Nov 2026 test: the same CMake settings, but with paths you can set instead of the lab PC's fixed ones.

## 4. Tell the plugin where the vendor libraries are

The plugin looks for `hd.dll` / `libHD.so` and `virtuoseAPI.dll` (or `VirtuoseDLL.dll`) / `libvirtuoseAPI.so` on the normal search path. If they're not there, set the full path:

```bat
set UH_OPENHAPTICS_LIB=C:\OpenHaptics\Developer\3.5.0\lib\x64\Release\hd.dll
set UH_VIRTUOSE_LIB=C:\path\to\virtuoseAPI.dll
```
On Linux, use `export` instead of `set`. On Windows, `virtuoseAPI.dll` must be a build that matches your compiler (e.g. `VC2019/x64/ReleaseMD`).

## 5. Test the device without SOFA: `uh_probe`

```bash
uh_probe                            # auto-detect, print the pose for 5 s
uh_probe touch 10                   # Touch only, 10 s
uh_probe touch 15 50                # Touch, 15 s, weak 50 N/m spring towards the start (max 2 N)
uh_probe haption 10 --haption 127.0.0.1#5000     # Haption; use your service's API port
uh_probe haption 15 50 --haption 127.0.0.1#5000  # Haption with the weak spring
```
It prints which backend was used (and why the others were skipped), then the position (m), orientation (quaternion), buttons, `ready` and the servo rate. Expect ~1000 Hz and `ready` = 1. With a spring, the handle should **pull back** towards where it started.

## 6. Run a SOFA scene

```bash
runSofa scenes/UniversalHaptics-RigidSphere.scn           # Touch
runSofa scenes/UniversalHaptics-RigidSphere-Haption.scn   # Haption (set haptionAddress to your service's port)
```
Press Play. The tool should follow the device, and pushing it into the sphere should push back.

## Component: `UniversalHapticsDevice`

| Field | Default | Meaning |
|---|---|---|
| `device` | `auto` | `auto`, `touch` or `haption` |
| `driverName` | | SofaHAPI-compatible alias: `Any`, `Phantom` or `Haption`. Overrides `device` |
| `detectionOrder` | `touch haption` | order tried by `auto` |
| `touchName` | `Default Device` | OpenHaptics device name |
| `haptionAddress` | `127.0.0.1#5000` | `ip#port` of the Haption service's API port |
| `scale` | 1 | device metres → scene units (100 = cm) |
| `forceScale` | 1 | scene force → newtons |
| `torqueScale` | 0 | scene torque → N·m on 6-DOF output devices (Haption); 0 = off |
| `maxForce` | 3 | safety clamp, newtons |
| `positionBase`, `orientationBase` | identity | device base in the scene |
| `positionTool`, `orientationTool` | identity | tool in the end-effector frame |
| `permanent` | false | keep forces on while the simulation is paused |
| `writeState` | true | write the pose into this node's Rigid3 MechanicalObject (SofaHAPI behaviour) |
| `toolIndex` | 0 | which entry of that MechanicalObject |
| `emitButtonEvent` | true | send a `HapticDeviceEvent` when a button changes |
| `forceFeedBack` (link) | first found | the `ForceFeedback` (e.g. `LCPForceFeedback`) to render |
| `drawDevice`, `drawHandleSize`, `drawForceScale` | | debug drawing |
| `printLog` | false | one status line per second: servo rate, tool position, force, buttons |
| **outputs:** `positionDevice`, `button1`, `button2`, `activeDevice`, `deviceInfo`, `outputForce`, `servoRate` | | |

There are two ways to wire it into a scene:
- **Geomagic style:** `<MechanicalObject template="Rigid3" position="@hapticDevice.positionDevice"/>`, as in the example scenes.
- **SofaHAPI style:** put the component in the node of the Rigid3 MechanicalObject it drives.

**Porting a SofaHAPI scene:** rename `SofaHAPIHapticsDevice` to `UniversalHapticsDevice`, and `RequiredPlugin SofaHAPI` to `UniversalHaptics`. The other fields keep their meaning and values.

**Porting a Geomagic scene:** replace `GeomagicDriver` with `UniversalHapticsDevice`, and change `scale="1"` to `scale="100"`. This plugin reports metres; the Geomagic driver reported centimetres.

## Safety

- **`maxForce` defaults to 3 N.** Raise it deliberately: the Haption Virtuose is much stronger than the Touch.
- **Forces drop to zero** 200 ms after the last simulation step, unless `permanent="1"`.
- **The Haption only gets forces while its dead-man switch reports "pressed".** The Force Feedback button on the arm turns the motors off.
- **`UH_IGNORE_DEADMAN=1` is for the Haption simulator (VirtuoseSIM) only.** Never set it with the real device. The plugin prints a warning whenever it is set.

## Known issues

- **Haption on Windows:** not yet run with the device (see Test status). On Windows, check the service's API port (5000 or 53210) and allow it through the firewall.
- **Haption axis directions** have not been verified on the real arm. The axis mapping follows HAPI's convention for the Virtuose: common (x, y, z) = Virtuose (y, z, x). Move the handle right, up and towards you, and check the tool moves the same way. If one is flipped, fix it in `VirtuoseBackend.cpp`.
- **Example scenes' orientation:** they use `orientationBase="0 0.707 0 -0.707"` (a 90° turn about the vertical axis), copied from SOFA's `Geomagic-RigidSphere.scn`. On Windows the motion looked rotated relative to the screen. Use `orientationBase="0 0 0 1"` if you want no rotation.
- **`VirtuoseBackend::servoLoop`:**
  - The dead-man flag starts as "pressed" (`dead = 1`), so if reading the dead-man switch *fails*, forces are not blocked. It should start at 0.
  - The return values of the Virtuose position/speed reads are not checked, so a failed read could make the pose jump to the origin for one tick.

  Both are known, and not yet fixed because they couldn't be re-tested on the device.
- **Servo rate on Windows** was ~940 Hz with the Touch (one dip to ~810 Hz), against ~1020 Hz on Linux.
- **The Haption reports positions from its base**, not its workspace centre. The Haption scene uses `positionBase="19 -15 2"` to bring the tool next to the sphere. `forceCoef` in the scenes is not yet tuned.

## License

LGPL-2.1, like SOFA. See `LICENSE`.
