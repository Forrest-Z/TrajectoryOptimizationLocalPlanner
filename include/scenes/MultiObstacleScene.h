//
// Created by mansoor on 13.11.20.
//

#ifndef TRAJECTORY_PLANNER_MULTI_SCENE_H
#define TRAJECTORY_PLANNER_MULTI_SCENE_H

#include "SimpleScene.h"

namespace scenes {
    /**
     * @class MultiObstacleScene
     * A Multipl obstacle scenario that extends the basic scenario with
     * the addition another obstacle to evaluate planner
     * turning between two obstacles.
     *
     * ++++++++++++++++++++++++++++++++++++++++++
     *          ++
     *        +    +
     *        +    +
     *          ++
     *               +-----planned path----- > G
     *  S --------- /    ++
     *                 +    +
     *                 +    +
     *                   ++
     * +++++++++++++++++++++++++++++++++++++++++++
     * @extends scenes::SimpleScene
     * @author Mansoor Nasir
     */
    class MultiObstacleScene : public scenes::SimpleScene {
    public:
        MultiObstacleScene();

    private:
        Point obstacle_center_;
        FloatingPoint obstacle_radius_;
        FloatingPoint obstacle_height_;
    };
}
#endif //TRAJECTORY_PLANNER_MULTI_SCENE_H
