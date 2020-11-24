# Trajectory Optimization based Local Planner
3D Voxels and Semantics based continuous time motion planner for Autonomous Driving. Uses 3D pointcloud and segmentation to create Voxels with semantic and TSDF information, and generates Euclidean Sign Distance Fields from TSDF while using it to optimize a ploynomial based trajectory optimization problem. The Geometric data can be obtained using Lidar along with RGB camera images to combine geometric information like mesh and sematic information like lanes, road surface for planning safe and efficient paths. <br>
## Simulated Scenes
Currently, a few simulated scenarios are provided to emulate a basic on road obstacle scenarios. <br>
### Simple scenario 
A simple scenario of a single cylindrical obstacle
           in the road center.
### Multi-Obstacle scenario
A scenario consisting of two obstacles placed next to each other to evaluate planner turning betwen obstacles.
### Sloped scenario
Added a slope to simple scenario to analyze how the planner plans in 3D and above the road surface.

## Requirements
The versions tested are specified in braces.
- Eigen3 (3.3.7)
- Protobuf
- Glog (0.4.0)
- Ceres (1.14.0)
- CMake (3.16.0)

## Instructions - Step By Step
### Outline
- Generate TSDF layers 
- Convert the TSDF layers
- Visualize the ESDF layers 
- Planning using ESDF Map
>**Note** <br>
>CMake Targets for the above specified programs are provided in the CMakeLists.txt


### Details
#### TSDF Generation
#### ESDF Generation from TSDF
#### ESDF Voxel Visualization
#### Planning

## Scripts
### Scene 1
### Scene 2
### Scene 3



### Planner
A document describing the planner (planner.pdf) contains the derivation for the planner.