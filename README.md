# Trajectory Optimization based LocalPlanner
3D Voxels and Semantics based continous motion planner for Autonomous Driving. Uses 3D pointcloud and segmentation to create Voxels with semantic and sign distance field information and generate eucledean sign distance from voxel distance while using it to optimize a ploynomial based trajectory optimization problem. The Geometric data can be obtained using Lidar data along with RGB camera images to combine geometric information like vehicles and sematic information like lanes, road surface for planning safe and efficient paths. <br>
Currently, a few simulated scenarios are provided to emulate a basic on road obstacle scenarios. <br>
- SimpleScenario 
    - A simple scenario of a single cylindrical obstacle
           in the road center.
- MultiObstacleScenario
    - A scenario consisting of two obstacles placed next to each other to evaluate planner turning betwen obstacles.
- SlopedScenario
    - Added a slope to simple scenario to analyze how the planner plans in 3D and above the road surface.

### Requirements
- Eigen3
- Protobuf
- glog
- Ceres
- cmake 3.16 or greater

### Instructions
- Generate TSDF layers using `src/generate_tsdf.cpp`
- Convert the TSDF layers to ESDF using `src/gesdf_from_tsdf.cpp`
- Visualize the ESDF layers using `src/visualize_esdf_voxels.cpp`
- Planning - use target `src/planning.cpp` to use semantic planner. Provide ESDF layers's generated in last step as argument. 

>**Note** <br>
>CMake Targets for the above specified programs are provided in the CMakeLists.txt

### Planner
A document describing the planner (planner.pdf) contains the derivation for the planner.