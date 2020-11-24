//
// Created by mansoor on 13.11.20.
//

#include "scenes/SimpleScene.h"

scenes::SimpleScene::SimpleScene()
        : obstacle_center_(0.0, 0.0, 0.0),
          obstacle_radius_(1),
          obstacle_height_(3),
          road_center_(0.0, 0.0, 0.0),
          road_size_(30.0, 8.0, 0.2),
          road_color_(121, 104, 120),
          BaseScene() {
    _setupObjects();
}

scenes::SimpleScene::SimpleScene(const Point &obstacle_center,
                                 const FloatingPoint obstacle_radius,
                                 const FloatingPoint obstacle_height)
        : obstacle_center_(obstacle_center),
          obstacle_radius_(obstacle_radius),
          obstacle_height_(obstacle_height),
          road_center_(0.0, 0.0, 0.0),
          road_size_(30.0, 8.0, 0.2),
          road_color_(121, 104, 120),
          BaseScene() {
    _setupObjects();
}

scenes::SimpleScene::SimpleScene(const Point &obstacle_center,
                                 const FloatingPoint obstacle_radius,
                                 const FloatingPoint obstacle_height,
                                 const Point &road_center,
                                 const Point &road_size,
                                 const Color road_color,
                                 const Point min_bound, const Point max_bound,
                                 const size_t voxels_per_side,
                                 const FloatingPoint voxel_size,
                                 const FloatingPoint fov_h_rad,
                                 const FloatingPoint max_dist,
                                 const FloatingPoint sensor_noise,
                                 const Eigen::Vector2i depth_camera_resolution,
                                 const std::string pointcloud_filename)
        : obstacle_center_(obstacle_center),
          obstacle_radius_(obstacle_radius),
          obstacle_height_(obstacle_height),
          road_center_(road_center),
          road_size_(road_size),
          road_color_(road_color),
          BaseScene(min_bound,
                    max_bound,
                    voxels_per_side,
                    voxel_size,
                    fov_h_rad,
                    max_dist,
                    sensor_noise,
                    depth_camera_resolution, pointcloud_filename) {
    _setupObjects();
}


bool scenes::SimpleScene::isDrivable(const Point &p, const Color &c) {
    return c == road_color_;
}

void scenes::SimpleScene::_setupObjects() {
    // set ground level
    world_.addGroundLevel(0.0);

    // add a cylinder obstacle on road
    world_.addObject(std::unique_ptr<Object>(new Cylinder(
            obstacle_center_, obstacle_radius_, obstacle_height_, Color::Red())));

    // add road surface
    world_.addObject(std::unique_ptr<Object>(new Cube(
            road_center_, road_size_, road_color_)));
}
