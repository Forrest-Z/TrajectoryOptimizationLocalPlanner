#include <iostream>
#include "voxblox/core/voxel.h"
#include "scenes/SimpleScene.h"
#include "scenes/MultiObstacleScene.h"
#include "scenes/SlopedScene.h"
#include "scenes/BaseScene.h"

int main(int argc, char *argv[]) {
    if (argc < 4) {
        std::cout << "Usage: generate_tsdf <scene> <output_obstacles_layer> <output_drivable_layer>";
        std::cout << "scene - Scenario for mapping. Choose from [simple, multi, slope]" << std::endl;
        std::cout << "output_drivable_layer - Path to load the TSDF layer for driving zone" << std::endl;
        return -1;
    }
    scenes::BaseScene *scene;

    if (strcmp(argv[1], "simple") == 0) {
        scene = new scenes::SimpleScene();
    } else if (strcmp(argv[1], "multi") == 0) {
        scene = new scenes::MultiObstacleScene();
    } else if (strcmp(argv[1], "slope") == 0) {
        scene = new scenes::SlopedScene();
    } else {
        throw std::runtime_error("Invalid scenario specified!");
    }
    scene->setupScene();
    scene->saveSceneTSDF(argv[2], argv[3]);

    delete scene;
    return 0;
}
