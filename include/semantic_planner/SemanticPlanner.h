//
// Created by mansoor on 13.11.20.
//

#ifndef TRAJECTORY_PLANNER_SEMANTICPLANNER_H
#define TRAJECTORY_PLANNER_SEMANTICPLANNER_H

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"
#include "loco_planner/loco.h"
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::experimental::filesystem;

using namespace std;
using namespace voxblox;

#define kN 10 //polynomial degree
#define kD 3 //state space
#define kS 3 //number of segments

namespace semantic_planner {
    /**
     * @class SemanticPlanner
     * A Planner that used geometric and semantic information for optimizing polynimial trajectory of segments kS ,
     * order kN and dimentions kD. It uses ESDF grids for both obstacles and drivable zones, ESDF for obstacles are
     * maximized while ESDF for drivable zones are minimized.
     * @param obstacles_map ESDF Map for obstacles
     * @param drivable_map ESDF Map for drivable zrea
     * @author Mansoor Nasir
     */
    class SemanticPlanner {
    public:
        /**
         * @class SemanticPlanner
         * @param obstacles_map ESDF Map for obstacles
         * @param drivable_map ESDF Map for drivable zrea
         */
        SemanticPlanner(const EsdfMap &obstacles_map, const EsdfMap &drivable_map);

        /**
         * Same as above but instead loads the ESDF map from file.
         * @class SemanticPlanner
         * @param obstacles_layer_path path for file containing ESDF layer
         * @param drivable_layer_path ESDF Map path
         */
        SemanticPlanner(const std::string obstacles_layer_path, const std::string drivable_layer_path);

        /**
        * Setup planner's distance functions' for obstacles and drivable area and map resolution.
        */
        void setupPlanner();

        /**
        * Plans a trajectory from start to goal
        * @param start start position in dimension kD
        * @param goal goal position in dimension kD
        * @param dt Time differential to sample trajectory.
        * @param waypoints A list of waypoints sampled from trajectory at dt time offsets.
        */
        mav_trajectory_generation::Trajectory
        planTrajectory(const Eigen::Vector3d start, const Eigen::Vector3d goal, std::vector<Eigen::VectorXd> *waypoints,
                       FloatingPoint dt = 0.1, bool verbose = true);

        /**
        * Save the sampled waypoints as points in the scene pointcloud
        * @param waypoints sampled waypoints from trajectory
        * @param point_cloud_path Pointcloud file for the scene.
        */
        void savePlannedPath(const std::vector<Eigen::VectorXd> &waypoints, std::string point_cloud_path);


    private:
        /**
        * Get ESDF values at a position in 3D along with the gradient.
        * @param position position for which get esdf values are returned
        * @param gradient save gradient at position into this object
        * @param esdf_map A map containing ESDF voxel grid.
        * @param d default distance.
        */
        double _getMapDistanceAndGradient(
                const Eigen::Vector3d &position, Eigen::Vector3d *gradient, std::shared_ptr<voxblox::EsdfMap> esdf_map,
                double d);

        /**
        * Get ESDF values at a position along with the gradient. Forward call toe above method after
        * checking values.
        * @param position position for which get esdf values are returned
        * @param gradient save gradient at position into this object
        * @param esdf_map A map containing ESDF voxel grid.
        * @param d default distance.
        */

        double _getMapDistanceAndGradientVector(
                const Eigen::VectorXd &position, Eigen::VectorXd *gradient, std::shared_ptr<voxblox::EsdfMap> esdf_map,
                double d);

    private:
        // ESDF layers
        Layer<EsdfVoxel>::Ptr drivable_layer_;
        Layer<EsdfVoxel>::Ptr obstacles_layer_;

        // ESDF maps
        voxblox::EsdfMap::Ptr esdf_obstacles_map_;
        voxblox::EsdfMap::Ptr esdf_drivable_map_;

        // Planner
        loco_planner::Loco<kN> loco_;
        // conservative planner for obstacles.  unknown space means obstacle is 0 - meter awau
        static constexpr int DEFAULT_DISTANCE_OBSTACLES = 0.0;
        // conservative. Unkonwn points are far away from road.
        static constexpr int DEFAULT_DISTANCE_DRIVINGZONE = 3.0;
    };
}
#endif //TRAJECTORY_PLANNER_SEMANTICPLANNER_H
