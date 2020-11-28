#include <iostream>
#include "semantic_planner/SemanticPlanner.h"

/**
 *
 * @param argument 3D double vector as comma separated string
 * @return  Eigen::Vector3d vector
 */
Eigen::Vector3d parseVector(std::string argument, char delim=',') {
    std::vector<double> position;
    size_t next;
    size_t start = 0;
    do {
        next = argument.find(delim, start);
        std::string elementStr = argument.substr(start, next - start);
        double element = std::stod(elementStr);// a row element
        position.push_back(element);// add the element at the end of row
        start = next + 1;
    } while ((next != std::string::npos));

    return Eigen::Vector3d(position.data()); // return value optimization- R value
}

int main(int argc, char *argv[]) {

    if (argc < 6) {
        std::cout<<"Usage: semantic_planner <start> <goal> <obstacles_layer_path> <drivable_layer_path> <pointcloud_path>"<<std::endl;
        std::cout<<"start - comma separated start position in 3D"<<std::endl;
        std::cout<<"goal - comma separated goal position in 3D"<<std::endl;
        std::cout<<"obstacles_layer_path - Path to the ESDF layer for obstacles"<<std::endl;
        std::cout<<"drivable_layer_path - Path to the ESDF layer for driving zone"<<std::endl;
        std::cout<<"pointcloud_path -  Save planned path highlighted over this pointcloud"<<std::endl;
        return -1;
    }

    // initialize start and goal positions
    std::vector<Eigen::VectorXd> waypoints;
    Eigen::Vector3d start(parseVector(argv[1])); //initialize depending upon scenario
    Eigen::Vector3d goal(parseVector(argv[2])); //initialize depending upon scenario

    // planner
    semantic_planner::SemanticPlanner planner(argv[3], argv[4]);

    //plan path
    auto trajectory = planner.planTrajectory(start, goal, &waypoints );

    // save planned path over the input pointcloud
    planner.savePlannedPath(waypoints, argv[5] );
    std::cout << "Planned path appended to pointcloud file!" << std::endl;
    return 0;
}
