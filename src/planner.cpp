#include <iostream>
#include "semantic_planner/SemanticPlanner.h"

int main(int argc, char *argv[]) {
    if (argc < 4) {
        std::cout<<"Usage: semantic_planner <obstacles_layer_path> <drivable_layer_path> <pointcloud_path>"<<std::endl;
        std::cout<<"obstacles_layer_path - Path to the ESDF layer for obstacles"<<std::endl;
        std::cout<<"drivable_layer_path - Path to the ESDF layer for driving zone"<<std::endl;
        std::cout<<"pointcloud_path -  Save planned path over the input pointcloud"<<std::endl;
        return -1;
    }
    // initialize start and goal positions
    std::vector<Eigen::VectorXd> waypoints;
    Eigen::Vector3d start(-9., 1.f, 0.3); //initialize depending upon scenario
    Eigen::Vector3d goal(7, -1.f, 0.3); //initialize depending upon scenario

    // planner
    semantic_planner::SemanticPlanner planner(argv[1], argv[2]);

    //plan path
    auto trajectory = planner.planTrajectory(start, goal, &waypoints );

    // save planned path over the input pointcloud
    planner.savePlannedPath(waypoints, argv[3] );
    std::cout << "Planned path appended to pointcloud file!" << std::endl;
    return 0;
}
