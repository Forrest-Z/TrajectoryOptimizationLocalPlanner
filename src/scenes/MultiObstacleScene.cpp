//
// Created by mansoor on 13.11.20.
//

#include "scenes/MultiObstacleScene.h"

scenes::MultiObstacleScene::MultiObstacleScene() : obstacle_center_(4.0, -2.0, 0.0),
                                                   obstacle_radius_(1.5f),
                                                   obstacle_height_(3.f),
                                                   SimpleScene(Point(-2.0f, 2.f, 0.0f), 1.5f, 3.f) {
    // add a cylinder obstacle on road
    world_.addObject(std::unique_ptr<Object>(new Cylinder(
            obstacle_center_, obstacle_radius_, obstacle_height_, Color::Red())));
}