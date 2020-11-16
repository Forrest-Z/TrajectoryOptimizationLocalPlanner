cmake_minimum_required(VERSION 3.16)
project(trajectory_planner)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR} /home/mansoor/Downloads/minkindr-master/minkindr/include ${GLOG_INCLUDE_DIR})
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set("${PROJECT_NAME}_SRCS"
        src/voxblox/alignment/icp.cc
        src/voxblox/core/block.cc
        src/voxblox/core/esdf_map.cc
        src/voxblox/core/tsdf_map.cc
        src/voxblox/integrator/esdf_integrator.cc
        src/voxblox/integrator/esdf_occ_integrator.cc
        src/voxblox/integrator/integrator_utils.cc
        src/voxblox/integrator/intensity_integrator.cc
        src/voxblox/integrator/tsdf_integrator.cc
        src/voxblox/io/mesh_ply.cc
        src/voxblox/io/sdf_ply.cc
        src/voxblox/mesh/marching_cubes.cc
        src/voxblox/simulation/objects.cc
        src/voxblox/simulation/simulation_world.cc
        src/voxblox/utils/camera_model.cc
        src/voxblox/utils/evaluation_utils.cc
        src/voxblox/utils/layer_utils.cc
        src/voxblox/utils/neighbor_tools.cc
        src/voxblox/utils/protobuf_utils.cc
        src/voxblox/utils/timing.cc
        src/voxblox/utils/voxel_utils.cc
        src/loco_planner/loco_main.cpp
        src/mav_trajectory_generation/motion_defines.cpp
        src/mav_trajectory_generation/polynomial.cpp
        src/mav_trajectory_generation/polynomial_timing_evaluation.cpp
        src/mav_trajectory_generation/rpoly/rpoly_ak1.cpp
        src/mav_trajectory_generation/segment.cpp
        src/mav_trajectory_generation/timing.cpp
        src/mav_trajectory_generation/trajectory.cpp
        src/mav_trajectory_generation/vertex.cpp
        src/scenes/BaseScene.cpp
        src/scenes/SimpleScene.cpp
        src/scenes/MultiObstacleScene.cpp
        src/scenes/SlopedScene.cpp
        )

set(PROTO_DEFNS proto/voxblox/Block.proto proto/voxblox/Layer.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

add_library(${PROJECT_NAME}_proto ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(generate_tsdf src/generate_tsdf.cpp ${${PROJECT_NAME}_SRCS})
add_executable(visualize_esdf_voxels src/visualize_esdf_voxels.cpp ${${PROJECT_NAME}_SRCS})
add_executable(esdf_from_tsdf src/esdf_from_tsdf.cpp ${${PROJECT_NAME}_SRCS})
add_executable(planning src/planner.cpp src/semantic_planner/SemanticPlanner.cpp ${${PROJECT_NAME}_SRCS})

target_link_libraries(generate_tsdf Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROJECT_NAME}_proto)
target_link_libraries(visualize_esdf_voxels Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROJECT_NAME}_proto ${CERES_LIBRARIES})
target_link_libraries(planning Eigen3::Eigen glog::glog protobuf::libprotobuf  ${PROJECT_NAME}_proto ${CERES_LIBRARIES} "stdc++fs")
target_link_libraries(esdf_from_tsdf Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROJECT_NAME}_proto)