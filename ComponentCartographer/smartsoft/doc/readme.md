## Component Cartographer

   * Component Cartographer is used to (1) Build map of the environment and (2) Localizing the robot given map of the environment using laser and IMU sensors.
	     				 
   * Based on [Google Cartographer](https://github.com/googlecartographer/cartographer "Cartographer GitHub").
   
## Component Model

   <img src="../../model/ComponentCartographerComponentDefinition.jpg" alt="drawing" width="800"/>
   
-----------------
## Dependencies
**Install Cartographer (Below installation steps are for Ubuntu 16.04)**

   from: [Cartographer Installation](https://google-cartographer.readthedocs.io/en/latest/)

   
```sh
sudo apt-get update
sudo apt-get install cmake -y
sudo apt-get install -y \
    clang \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    ninja-build \
    python-sphinx
```
```sh
VERSION="1.13.0"

# Build and install Ceres.
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout tags/${VERSION}
mkdir build
cd build
cmake .. -G Ninja -DCXX11=ON
ninja
CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install

```
```sh
VERSION="v3.4.1"

# Build and install proto3.
git clone https://github.com/google/protobuf.git
cd protobuf
git checkout tags/${VERSION}
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  ../cmake
ninja
sudo ninja install
```
```sh
# Build and install Cartographer.
git clone https://github.com/googlecartographer/cartographer.git
gitcheckout 1.0.0
cd cartographer
mkdir build
cd build
cmake .. -G Ninja
ninja
CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install
```

## Usage

   * **Setup available sensors**

       * `use_laser` parameter must be true always
       * `use_odometry` parameter should be true if odometery to be used

   * **Using in Mapping mode**
       *  `pure_localization` must be set `false`
       *   Set `load_previous_state` to `false` if you want to start a new mapping session
       *   To continue the previous session of mapping `load_previous_state` should be true and set the path of `.pbstream` file in `pb_stream` parameter

   * **Using in Mapping mode**
       * Set `pure_localization` parameter to `true`
       * Set path of `.pbstream` file in `pb_stream` parameter
       *   Set `load_previous_state` to `true`

   * **Tuning the parameters**
       * `Configuration_files` directory contains the configuration files
       *  Please refer [Cartographer ROS documentation](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html)

## Workflow

**1. Initialization**

   * Check the sensors available for the operation and save their ids in `std::set<SensorId>`.
   
   * Check if the component is about to run in `Mapping` or `Localization` mode from the `.ini`. file.
   
   * Based on the mode read the corresponding `MapBuilderOptions` and `TrajectoryBuilderOptions` from the configuration files.
   
   * Initialize the `MapBuilder` using `MapBuilderOptions`.
   
   * Add a `AddTrajectoryBuilder` to `MapBuilder` using `TrajectoryBuilderOptions` and sensor saved in `std::set<SensorId>`.
   
   * Get `TrajectoryBuilderInterface` for the trajectroy builder add in the above step.
   
**2. Receive Sensor messages (in loop)**

   * Receive `CommMobileLaserScan` and convert it into `cartographer::sensor::TimedPointCloudData` and then add the converted message to `TrajectoryBuilderInterface`.
   * Receive Odometry message (`CommBasePose`) and convert it into `cartographer::sensor::OdometryData` and then add the converted message to `TrajectoryBuilderInterface`.
   
**3. Stop the Map builder and Save the Map**

   * Trigger the component to stop building and save the map.
   * Finish the current trajectory by calling `FinishTrajectory` and optimize it using `map_builder->pose_graph()->RunFinalOptimization()`.
   * Collect the submaps using `map_builder->pose_graph()->GetAllSubmapPoses()` and add them using `cartographer::io::PaintSubmapSlices`.
   * Finally convert the `cartographer::io::PaintSubmapSlicesResult` into `cartographer::io::Image` and then to `.pgm` format.
   
### Additional Reading

   * [Cartographer Documentation](https://google-cartographer.readthedocs.io/en/latest/ "Cartographer Documentation")