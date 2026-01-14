# Monocular SLAM

A ground-up monocular Visual SLAM engine written in C++17/CUDA targeting real-time performance on the KITTI odometry benchmark.

![Platform](https://img.shields.io/badge/platform-Windows-blue)
![Language](https://img.shields.io/badge/language-C%2B%2B17%20%7C%20CUDA-orange)
![Build](https://img.shields.io/badge/build-CMake%203.20%2B-green)

---

## Features

- **ORB feature extraction** via OpenCV with GPU-accelerated Hamming distance matching (custom CUDA kernel)
- **Monocular initialization** via Essential matrix decomposition and triangulation
- **Constant-velocity motion model** for efficient frame-to-frame pose prediction
- **PnP-RANSAC pose estimation** (SQPNP solver) with inlier-guided refinement
- **Sliding-window local bundle adjustment** using Ceres Solver with Huber loss (analytical Jacobians, 10-keyframe window)
- **Relocalization** against the full global map on tracking loss
- **Real-time 3D visualization** via [Rerun](https://rerun.io/) -- trajectory, map point cloud, keypoints, and ground-truth overlay
- KITTI odometry dataset support out of the box

---

## Architecture

```
+---------------+    ORB features     +------------------+
|   main.cpp    | ------------------> |     Tracker      |
| KITTI loader  |                     |  (tracker.cpp)   |
+---------------+                     +--------+---------+
                                               | keyframes
                                  +------------v------------+
                                  |     Map / MapPoint      |
                                  | (map.cpp, map_point.cpp)|
                                  +------------+------------+
                                               |
                                  +------------v------------+
                                  |   LocalBA (Ceres)       |
                                  |   (local_ba.cpp)        |
                                  +------------+------------+
                                               |
                                  +------------v------------+
                                  | Visualizer (Rerun SDK)  |
                                  |  (visualizer.cpp)       |
                                  +-------------------------+
```

**GPU pipeline:** each frame's ORB descriptors are uploaded to the GPU; the CUDA kernel computes pairwise Hamming distances in shared memory (one block per query descriptor, 256 threads) and returns top-2 matches for Lowe ratio filtering -- all before any CPU-side matching logic runs.

---

## Requirements

| Dependency | Version | Notes |
|---|---|---|
| MSVC | 19.x (VS 2022) | C++17 required |
| CUDA Toolkit | 12.x | Compute Capability 8.6 (RTX 30xx / RTX 40xx) |
| CMake | 3.20+ | |
| vcpkg | latest | for OpenCV, Ceres, Eigen |
| OpenCV | 4.x | core, features2d, calib3d, highgui |
| Ceres Solver | 2.x | eigensparse + schur features |
| Eigen3 | 3.4+ | |
| Rerun SDK | 0.22.1 | fetched automatically via CMake FetchContent |

> **Other GPU architectures:** edit `CMAKE_CUDA_ARCHITECTURES` in [CMakeLists.txt](CMakeLists.txt) (line 26). Use `75` for Turing (RTX 20xx), `89` for Ada (RTX 40xx), etc.

---

## Building

### 1. Install vcpkg dependencies

```bat
cd C:\Users\<you>\vcpkg
vcpkg install opencv4[core,features2d,calib3d,highgui] --triplet x64-windows
vcpkg install ceres[eigensparse,schur] --triplet x64-windows
vcpkg install eigen3 --triplet x64-windows
vcpkg integrate install
```

### 2. Configure and build

Open the project folder in **VS Code** with the CMake Tools extension installed, then:

```
Ctrl+Shift+P  ->  CMake: Configure
Ctrl+Shift+P  ->  CMake: Build
```

Or from a Developer Command Prompt:

```bat
cmake -B build -DCMAKE_TOOLCHAIN_FILE=C:/Users/<you>/vcpkg/scripts/buildsystems/vcpkg.cmake ^
      -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

The Rerun SDK is downloaded and built automatically on first configure (~2-5 min).

---

## Dataset

Download the [KITTI Odometry Dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) (grayscale images + calibration). Place sequences under:

```
VSLAM/
+-- data/
    +-- dataset/
        +-- poses/
        |   +-- 00.txt          <- ground-truth poses (optional, for GT overlay)
        +-- sequences/
            +-- 00/
                +-- calib.txt
                +-- times.txt
                +-- image_0/
                    +-- 000000.png
                    +-- ...
```

---

## Usage

```bat
cd C:\...\VSLAM
build\Release\vslam.exe --sequence data/dataset/sequences/00
```

**Options:**

| Flag | Default | Description |
|---|---|---|
| `--sequence <path>` | required | Path to a KITTI sequence directory |
| `--start <N>` | 0 | First frame index |
| `--end <N>` | last | Last frame index (inclusive) |
| `--no-viz` | off | Disable Rerun visualization |

**Ground-truth overlay:** if a matching pose file exists at `data/dataset/poses/<seq_id>.txt`, it is automatically loaded and shown as an orange line in the Rerun viewer.

### Rerun Viewer

Start the viewer before (or after) launching the SLAM process:

```bat
rerun
```

The engine connects to `127.0.0.1:9876` by default. The 3D panel is auto-created under the `world/camera/` entity tree and shows:

- **Blue line** -- estimated camera trajectory
- **Orange line** -- KITTI ground-truth trajectory
- **White point cloud** -- active map points
- **Purple dots** -- current-frame ORB keypoints (2D image panel)

---

## Project Structure

```
VSLAM/
+-- CMakeLists.txt              # Build configuration
+-- vcpkg.json                  # vcpkg manifest
+-- include/
|   +-- slam/
|   |   +-- camera.hpp          # Pinhole camera model
|   |   +-- frame.hpp           # Frame: image, keypoints, pose
|   |   +-- map_point.hpp       # 3D landmark
|   |   +-- map.hpp             # Keyframe + map point store
|   |   +-- tracker.hpp         # Front-end tracker interface
|   |   +-- local_ba.hpp        # Bundle adjustment interface
|   |   +-- visualizer.hpp      # Rerun visualizer interface
|   +-- cuda/
|       +-- hamming_matcher.cuh # GPU Hamming matcher API
+-- src/
|   +-- camera.cpp
|   +-- frame.cpp
|   +-- map_point.cpp
|   +-- map.cpp
|   +-- tracker.cpp             # Initialization, tracking, keyframe logic
|   +-- local_ba.cpp            # Ceres cost functions + sliding-window BA
|   +-- visualizer.cpp          # Rerun entity logging
|   +-- main.cpp                # KITTI loader + main loop
+-- cuda/
|   +-- hamming_matcher.cu      # CUDA kernel
+-- .vscode/                    # VS Code tasks, launch, IntelliSense config
```

---

## Performance Notes

- Target: **>60 FPS** on a laptop RTX 3050 (KITTI sequence 00, 1241x376)
- Bundle adjustment runs asynchronously on keyframe insertion; the front-end tracker is not blocked
- The BA window is capped at 10 keyframes; the oldest pose is held fixed to resolve gauge freedom
- No loop closure -- long sequences will drift; adding a DBoW place recognition back-end is the natural next step