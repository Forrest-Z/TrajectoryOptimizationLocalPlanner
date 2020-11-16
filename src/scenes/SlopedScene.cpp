//
// Created by mansoor on 13.11.20.
//

#include "scenes/SlopedScene.h"

scenes::SlopedScene::SlopedScene() : num_steps_(4),
                                     slope_start_(9.0, 0.0, 0.2),
                                     slope_size_(8.0, 8.0, 0.35),
                                     slope_step_(0.5, 0.0, 0.35),
                                     SimpleScene(Point(-2.0, 2, 0.0), 1.5, 3) {

    auto slope_pos = slope_start_;
    for (size_t inc = 0; inc < num_steps_; inc++) {
        world_.addObject(std::unique_ptr<Object>(new Cube(slope_pos, slope_size_, road_color_)));
        slope_pos += slope_step_;
    }
}