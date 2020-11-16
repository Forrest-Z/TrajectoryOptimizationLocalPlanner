# TrajectoryOptimizationLocalPlanner
3D Voxels and Semantics based continous motion planner for Autonomous Driving. Uses 3D pointcloud and segmentation to create Voxels with semantic and sign distance field information and generate eucledean sign distance from voxel distance while using it to optimize a ploynomial based trajectory optimization problem. The Geometric data can be obtained using Lidar data along with RGB camera images to combine geometric information like vehicles and sematic information like lanes, road surface for planning safe and efficient paths. <br>
Currently, a few simulated scenarios are provided to emualte a basic on road obstacle scenarios. <br>
- SimpleScenario 
    - A simple scenrio of a single cylindrical obstacle
           in the road center.
- MultiObstacleScenario
    - A scenario containing two obstacles placed next to each other to evaluate planenr turning betwen obstacles.
- SlopedScenario
    - Added a slope to simple scenario to analyze how the planner plans in 3D and above the road surface.

### Requirements
- Eigen3
- Protobuf
- glog
- Ceres
- cmake 3.16 or greater

### Instructions
- Generate TSDF layers using *generate_tsdf* cmake target
- Convert the TSDF layers to ESDF using *esdf_from_tsdf* target
- Visualize the ESDF layers using *visualize_esdf_voxels* target
- Planning - use target *planning* to use semantic planner

### Planner
A document describing the planner (planner.pdf) contains the derivation for the planner.