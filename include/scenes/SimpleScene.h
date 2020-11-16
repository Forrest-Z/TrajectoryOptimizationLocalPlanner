//
// Created by mansoor on 13.11.20.
//

#ifndef TRAJECTORY_PLANNER_SIMPLESCENE_H
#define TRAJECTORY_PLANNER_SIMPLESCENE_H

#include "BaseScene.h"

namespace scenes {
    /**
     * A simple scenrio of a single cylindrical obstacle
     * in the middle of road.
     *
     * ++++++++++++++++++++++++++++++++++++++++++
     *
     *                  ___________
     *                 /           \
     * start >--------/    ++       \-----> goal
     *                   +    +
     *                   +    +
     *                     ++
     *
     * +++++++++++++++++++++++++++++++++++++++++++
     * @author Mansoor Nasir
     */
    class SimpleScene : public scenes::BaseScene {
    public:
        SimpleScene();

        SimpleScene(const Point &obstacle_center,
                    const FloatingPoint obstacle_radius,
                    const FloatingPoint obstacle_height);

        SimpleScene(const Point &obstacle_center,
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
                    const Eigen::Vector2i depth_camera_resolution);


    protected:
        virtual bool isDrivable(const Point &p, const Color &c);

    private:
        void _setupObjects();

     //member variables
    protected:
        Color road_color_;
    private:
        Point obstacle_center_;
        FloatingPoint obstacle_radius_;
        FloatingPoint obstacle_height_;
        Point road_center_;
        Point road_size_;
    };
}
#endif //TRAJECTORY_PLANNER_SIMPLESCENE_H
