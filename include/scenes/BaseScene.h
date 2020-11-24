//
// Created by mansoor on 13.11.20.
//

#ifndef TRAJECTORY_PLANNER_BASESCENE_H
#define TRAJECTORY_PLANNER_BASESCENE_H

#include <iostream>
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"

#include <iostream>
#include <fstream>

using namespace voxblox;

#define DEBUG_SCENE 1

namespace scenes {
    /**
     * Provides a base Implementation of a Scenario simulating a
     * scene.
     * Generates point cloud for the scene along with the pose and
     * uses it to construct TSDF.
     * @author Mansoor Nasir
     */
    class BaseScene {
    public:

        BaseScene(const std::string pointcloud_filename);

        BaseScene(const Point min_bound, const Point max_bound,
                  const size_t voxels_per_side, const FloatingPoint voxel_size,
                  const FloatingPoint fov_h_rad, const FloatingPoint max_dist,
                  const FloatingPoint sensor_noise, const Eigen::Vector2i depth_camera_resolution,
                  const std::string pointcloud_filename);


        BaseScene();

        /**
         * Resets the TSDF Layers
         */
        void reset();

        /**
         * Get TSDF layer for obstacles
         */
        const std::shared_ptr<Layer<TsdfVoxel>> &getObstaclesLayer() const;

        /**
         * Get TSDF layer for drivable zone.
         */
        const std::shared_ptr<Layer<TsdfVoxel>> &getDrivableZoneLayer() const;

        FloatingPoint getVoxelSize() const;

        /**
         * Generate a point cloud from camera path returned by getPoses() and
         * integrates to point cloud to obstacle and driving zone TSDF grids.
         */

        void setupScene();

        /**
         * Save the TSDF layers to file.
         */
        void saveSceneTSDF(const std::string obstacles_layer,
                           const std::string drivable_zone_layer) const;

        /**
         *
         * @param pointcloudFilename File path to save the output pointcloud
         */
        void setPointcloudFilename(const std::string &pointcloudFilename);

    private:
        /**
         *  A basic implementation of camera motion in a scene. The camera scans
         *  the scene along the straing path fron start with increment inc for
         *  num_poses and provides the camera parameters for each pose.
         */
        void getPoses(AlignedVector<Transformation> *poses,
                      const Point &start = Point(-13., 0, 3),
                      const Point &inc = Point(1, 0, 0),
                      std::size_t num_poses = 20);

    protected:
        /**
         *  whether this point belong to drivable surface semantic category
         */
        virtual bool isDrivable(const Point &p, const Color &c) = 0;

    protected:
        // world
        SimulationWorld world_;
        Point min_bound_;
        Point max_bound_;
        FloatingPoint voxel_size_;
        size_t voxels_per_side_;
        Color drivableZone_;
        std::string pointcloud_filename_;

    protected:

        //config
        TsdfIntegratorBase::Config tsdf_layer_config_;

        // camera config
        Eigen::Vector2i depth_camera_resolution_;
        FloatingPoint fov_h_rad_;
        FloatingPoint max_dist_;
        FloatingPoint sensor_noise_;

        // tsdf layers
        std::shared_ptr<Layer<TsdfVoxel>> obstacles_layer_;
        std::shared_ptr<Layer<TsdfVoxel>> drivable_zone_layer_;

        // tsdf integrators
        std::shared_ptr<SimpleTsdfIntegrator> obstacles_integrator_;
        std::shared_ptr<SimpleTsdfIntegrator> drivable_zone_integrator_;

        //other
        bool generated_;// whether esdf layers are initialized and updated
    };
}


#endif //TRAJECTORY_PLANNER_BASESCENE_H
