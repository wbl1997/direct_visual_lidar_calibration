# direct_visual_lidar_calibration

This package provides a toolbox for LiDAR-camera calibration that is: 

- **Generalizable**: It can handle various LiDAR and camera projection models including spinning and non-repetitive scan LiDARs, and pinhole, fisheye, and omnidirectional projection cameras.
- **Target-less**: It does not require a calibration target but uses the environment structure and texture for calibration.
- **Single-shot**: At a minimum, only one pairing of a LiDAR point cloud and a camera image is required for calibration. Optionally, multiple LiDAR-camera data pairs can be used for improving the accuracy.
- **Manual initialization**: The initial LiDAR-camera pose is selected manually, followed by automatic fine registration.
- **Accurate and robust**: It employs a pixel-level direct LiDAR-camera registration algorithm that is more robust and accurate compared to edge-based indirect LiDAR-camera registration.

**Documentation: [https://koide3.github.io/direct_visual_lidar_calibration/](https://koide3.github.io/direct_visual_lidar_calibration/)**  
**Docker hub: [koide3/direct_visual_lidar_calibration](https://hub.docker.com/repository/docker/koide3/direct_visual_lidar_calibration)**

[![Build](https://github.com/koide3/direct_visual_lidar_calibration/actions/workflows/push.yaml/badge.svg)](https://github.com/koide3/direct_visual_lidar_calibration/actions/workflows/push.yaml) [![Docker Image Size (latest by date)](https://img.shields.io/docker/image-size/koide3/direct_visual_lidar_calibration)](https://hub.docker.com/repository/docker/koide3/direct_visual_lidar_calibration)

![213393920-501f754f-c19f-4bab-af82-76a70d2ec6c6](https://user-images.githubusercontent.com/31344317/213427328-ddf72a71-9aeb-42e8-86a5-9c2ae19890e3.jpg)

[Video](https://www.youtube.com/watch?v=7TM7wGthinc&feature=youtu.be)

## Dependencies

- [ROS1/ROS2](https://www.ros.org/)
- [PCL](https://pointclouds.org/)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://gtsam.org/)
- [Ceres](http://ceres-solver.org/)
- [Iridescence](https://github.com/koide3/iridescence)

## Getting started

1. [Installation](https://koide3.github.io/direct_visual_lidar_calibration/installation/) / [Docker images](https://koide3.github.io/direct_visual_lidar_calibration/docker/)
2. [Data collection](https://koide3.github.io/direct_visual_lidar_calibration/collection/)
3. [Calibration example](https://koide3.github.io/direct_visual_lidar_calibration/example/)
4. [Program details](https://koide3.github.io/direct_visual_lidar_calibration/programs/)

## CPU build on Ubuntu 20.04

This repository version uses the manual initial-pose workflow. The core C++ programs do not require CUDA, an NVIDIA GPU, PyTorch, or SuperGlue.

The following instructions are for Ubuntu 20.04 with ROS Noetic. The workspace used during verification was `/home/pc/code/direct_calib_ws`.

### Dependencies

Install the ROS and system packages:

```bash
sudo apt update
sudo apt install -y \
  cmake build-essential \
  libceres-dev \
  libpcl-dev libopencv-dev libboost-all-dev \
  libomp-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev
```

`rosdep` can install the ROS package dependencies:

```bash
sudo rosdep init  # run once; skip if already initialized
rosdep update
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

GTSAM and Iridescence are not available as standard Ubuntu 20.04 packages in a minimal installation. Install them from source:

```bash
cd ~/src
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2a9
mkdir -p build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j"$(nproc)"
sudo make install

cd ~/src
git clone https://github.com/koide3/iridescence.git --recursive
cd iridescence
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
sudo make install
```

### Build with Catkin

Put this package below the Catkin `src` directory, then build in the workspace root:

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/koide3/direct_visual_lidar_calibration.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
```

If the repository already exists elsewhere, use a symlink instead of cloning it again:

```bash
ln -s /home/pc/code/direct_visual_lidar_calibration \
  ~/catkin_ws/src/direct_visual_lidar_calibration
```

### Calibration workflow

```bash
source ~/catkin_ws/devel/setup.bash
rosrun direct_visual_lidar_calibration preprocess \
  /path/to/input_bag /path/to/preprocessed
rosrun direct_visual_lidar_calibration initial_guess_manual \
  /path/to/preprocessed
rosrun direct_visual_lidar_calibration calibrate \
  /path/to/preprocessed
```

`initial_guess_manual` opens the point-cloud/image viewer. Select corresponding 3D and 2D points, estimate the initial pose, and save it before running `calibrate`.

The manual-initialization viewer starts from the camera projection view defined by `init_T_lidar_camera` in `calib.json`. It aligns the point-cloud view orientation and perspective field of view with the camera, and realigns the view after a new pose is estimated. The aligned view remains interactive: left-drag rotates, middle/right-drag translates, and the scroll wheel zooms. The result viewer (`process.sh 3`) applies the same alignment to the selected initial or final transformation. Fine registration (`process.sh 4`) starts from the initial camera pose while keeping the view interactive during calibration.

### One-command helper script

`process.sh` wraps the three commonly used commands. Run it from this package directory:

```bash
./process.sh 1 /path/to/input.bag /path/to/preprocessed \
  /camera/image /lidar/points \
  plumb_bob fx,fy,cx,cy k1,k2,p1,p2,k3

# Faro map example (PLY + JPG)
./process.sh 1 /home/pc/code/direct_calib_ws/datasets/faro/map.ply \
  /home/pc/code/direct_calib_ws/datasets/faro/image00.jpg \
  /home/pc/code/direct_calib_ws/datasets/faro_preprocessed \
  plumb_bob 2242.45,2242.69,1579.98,1172.42 \
  -0.0272569,0.00207913,-0.00411347,-0.00208982,-0.00291759

./process.sh 2 /path/to/preprocessed
./process.sh 3 /path/to/preprocessed
./process.sh 4 /path/to/preprocessed
```

The first command preprocesses a ROS1 bag or, when the input is a `.ply` file, a map and image pair. The second opens the manual initial-pose tool, the third opens the result viewer, and the fourth performs NID-based fine registration. The script automatically sources ROS Noetic and the Catkin workspace when `/opt/ros/noetic/setup.bash` and `devel/setup.bash` are available.

### Problems found during verification

- The first Catkin configure completed the ROS, Eigen, Boost, PCL, and OpenCV checks but stopped at `Could not find CeresConfig.cmake`.
- On Ubuntu 20.04, the missing development package is `libceres-dev`; install it with the dependency command above and rerun `catkin_make`.
- The current environment required sudo authentication for APT, so package installation could not be completed automatically by the build agent.
- Warnings about missing VTK helper executables were emitted while discovering PCL. They did not stop CMake configuration; install/reinstall the VTK development packages only if PCL visualization later fails.

When sudo is unavailable, the same dependencies can be installed in a user prefix. In the verified build, GTSAM 4.2a9, Ceres 2.1.0, Iridescence, and fmt were built under `/home/pc/code/direct_calib_deps` and installed to `~/.local`. The Catkin build then used:

```bash
source /opt/ros/noetic/setup.bash
export CMAKE_PREFIX_PATH="$HOME/.local:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
cd /home/pc/code/direct_calib_ws
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

The repository CMake configuration explicitly loads the `fmt` and `glfw3` imported targets so these user-prefix libraries link correctly. `process.sh` adds `~/.local/lib` automatically when it runs.

The verified build produced these executables:

```text
devel/lib/direct_visual_lidar_calibration/preprocess
devel/lib/direct_visual_lidar_calibration/preprocess_map
devel/lib/direct_visual_lidar_calibration/initial_guess_manual
devel/lib/direct_visual_lidar_calibration/calibrate
devel/lib/direct_visual_lidar_calibration/viewer
```

## License

This package is released under the MIT license.

## Publication

Koide et al., General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox, ICRA2023, [[PDF]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)

## Contact

Kenji Koide, National Institute of Advanced Industrial Science and Technology (AIST), Japan
