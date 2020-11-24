//
// Created by mansoor on 13.11.20.
//

#include "scenes/BaseScene.h"

scenes::BaseScene::BaseScene() : min_bound_(-5.0, -5.0, -1.0),
                                 max_bound_(5.0, 5.0, 6.0),
                                 voxels_per_side_(16),
                                 voxel_size_(0.10),
                                 fov_h_rad_(2.61799),
                                 max_dist_(10.0),
                                 sensor_noise_(0.0),
                                 generated_(false),
                                 pointcloud_filename_("pointcloud.txt"),
                                 depth_camera_resolution_(Eigen::Vector2i(320, 240)) {
    reset();
}

scenes::BaseScene::BaseScene(const std::string pointcloud_filename) : min_bound_(-5.0, -5.0, -1.0),
                                                                      max_bound_(5.0, 5.0, 6.0),
                                                                      voxels_per_side_(16),
                                                                      voxel_size_(0.10),
                                                                      fov_h_rad_(2.61799),
                                                                      max_dist_(10.0),
                                                                      sensor_noise_(0.0),
                                                                      generated_(false),
                                                                      pointcloud_filename_(pointcloud_filename),
                                                                      depth_camera_resolution_(
                                                                              Eigen::Vector2i(320, 240)) {
    reset();
}

scenes::BaseScene::BaseScene(const Point min_bound, const Point max_bound,
                             const size_t voxels_per_side, const FloatingPoint voxel_size,
                             const FloatingPoint fov_h_rad, const FloatingPoint max_dist,
                             const FloatingPoint sensor_noise, const Eigen::Vector2i depth_camera_resolution,
                             const std::string pointcloud_filename) :
        min_bound_(min_bound),
        max_bound_(max_bound),
        voxels_per_side_(voxels_per_side),
        voxel_size_(voxel_size),
        fov_h_rad_(fov_h_rad),
        max_dist_(max_dist),
        sensor_noise_(sensor_noise),
        depth_camera_resolution_(depth_camera_resolution),
        generated_(false), pointcloud_filename_(pointcloud_filename) {

    reset();
}

void scenes::BaseScene::reset() {
    world_.clear();
    world_.setBounds(min_bound_, max_bound_);

    // initialize layers
    tsdf_layer_config_.default_truncation_distance = 4 * voxel_size_;
    tsdf_layer_config_.integrator_threads = 1;

    obstacles_layer_.reset(new Layer<TsdfVoxel>(voxel_size_, voxels_per_side_));
    drivable_zone_layer_.reset(new Layer<TsdfVoxel>(voxel_size_, voxels_per_side_));

    // initialize tsdf integrators
    obstacles_integrator_.reset(new SimpleTsdfIntegrator(tsdf_layer_config_, obstacles_layer_.get()));
    drivable_zone_integrator_.reset(new SimpleTsdfIntegrator(tsdf_layer_config_, drivable_zone_layer_.get()));

    generated_ = false;
}

const std::shared_ptr<Layer<TsdfVoxel>> &scenes::BaseScene::getObstaclesLayer() const {
    return obstacles_layer_;
}

const std::shared_ptr<Layer<TsdfVoxel>> &scenes::BaseScene::getDrivableZoneLayer() const {
    return drivable_zone_layer_;
}

FloatingPoint scenes::BaseScene::getVoxelSize() const {
    return voxel_size_;
}

void scenes::BaseScene::setupScene() {
    //setup poses
    AlignedVector<Transformation> poses;
    getPoses(&poses);

#ifdef DEBUG_SCENE
    std::ofstream pointcloud_file;
    pointcloud_file.open(pointcloud_filename_);
#endif

    for (size_t i = 0; i < poses.size(); i++) {
        Pointcloud ptcloud, ptcloud_C_free, ptcloud_C_obstacles;
        Pointcloud free_points, obstacles_points;
        Colors colors;
        Colors free_colors, obstacles_colors;

        if (sensor_noise_ > 0) {
            world_.getNoisyPointcloudFromTransform(poses[i], depth_camera_resolution_, fov_h_rad_, max_dist_,
                                                   sensor_noise_, &ptcloud, &colors);
        } else {
            world_.getPointcloudFromTransform(poses[i], depth_camera_resolution_, fov_h_rad_, max_dist_, &ptcloud,
                                              &colors);
        }

        for (size_t i = 0; i < ptcloud.size(); ++i) {
            auto position = ptcloud[i];
            auto color = colors[i];

            if (isDrivable(position, color)) {
                free_points.push_back(position);
                free_colors.push_back(color);
                obstacles_points.push_back(position);
                obstacles_colors.push_back(color);
            } else {
                obstacles_points.push_back(position);
                obstacles_colors.push_back(color);
            }
#ifdef DEBUG_SCENE
            pointcloud_file << position(0) << ";" << position(1) << ";" << position(2) << ";";
            pointcloud_file << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << std::endl;
#endif

        }
        transformPointcloud(poses[i].inverse(), free_points, &ptcloud_C_free);
        transformPointcloud(poses[i].inverse(), obstacles_points, &ptcloud_C_obstacles);

        obstacles_integrator_->integratePointCloud(poses[i], ptcloud_C_obstacles, obstacles_colors);
        drivable_zone_integrator_->integratePointCloud(poses[i], ptcloud_C_free, free_colors);
        std::cout << "Integrated point cloud from pose " << i + 1 << "/" << poses.size() << std::endl;
    }
#ifdef DEBUG_SCENE
    pointcloud_file.close();
#endif
    generated_ = true;
}

void scenes::BaseScene::saveSceneTSDF(const std::string obstacles_layer_path,
                                      const std::string drivable_zone_layer_path) const {
    if (!generated_)
        throw std::runtime_error("TSDF not generated yet!");

    if (!obstacles_layer_path.empty()) {
        voxblox::io::SaveLayer(*obstacles_layer_, obstacles_layer_path);
    }

    if (!drivable_zone_layer_path.empty()) {
        voxblox::io::SaveLayer(*drivable_zone_layer_, drivable_zone_layer_path);
    }
}

void scenes::BaseScene::getPoses(AlignedVector<Transformation> *poses,
                                 const Point &start,
                                 const Point &inc,
                                 std::size_t num_poses) {
    CHECK_NOTNULL(poses);
    poses->clear();
    poses->reserve(num_poses);

    Point position(start);
    for (int i = 0; i < num_poses; i++) {
        // Generate a transformation to look at the center pose.
        position += inc;
        Point facing_direction(1.f, 0.f, 0.0f);

        FloatingPoint desired_yaw = 0.0;
        if (std::abs(facing_direction.x()) > 1e-4 ||
            std::abs(facing_direction.y()) > 1e-4) {
            desired_yaw = atan2(facing_direction.y(), facing_direction.x());
        }
        // Face the desired yaw and pitch forward a bit to get some of the fl/oor.
        Quaternion rotation = Quaternion(Eigen::AngleAxis<FloatingPoint>(-0.1, Point::UnitY())) *
                              Eigen::AngleAxis<FloatingPoint>(desired_yaw, Point::UnitZ());

        poses->emplace_back(Transformation(rotation, position));
    }
}

void scenes::BaseScene::setPointcloudFilename(const std::string &pointcloudFilename) {
    pointcloud_filename_ = pointcloudFilename;
}
