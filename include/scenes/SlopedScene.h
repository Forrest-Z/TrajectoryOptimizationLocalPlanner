//
// Created by mansoor on 13.11.20.
//

#ifndef TRAJECTORY_PLANNER_SLOPE_SCENE_H
#define TRAJECTORY_PLANNER_SLOPE_SCENE_H

#include "SimpleScene.h"

namespace scenes {
    /**
     * @class SlopedScene
     * A relatively complex scenario that extends the basic scenario with
     * the addition of stairs as a slope to test the planner.
     * @extends scenes::SimpleScene
     * @author Mansoor Nasir
     */
    class SlopedScene : public scenes::SimpleScene {
    public:
        SlopedScene();

    private:
        Point slope_start_;
        Point slope_step_;
        Point slope_size_;
        size_t num_steps_;
    };
}
#endif //TRAJECTORY_PLANNER_SLOPE_SCENE_H
