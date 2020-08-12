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

using namespace std;
using namespace voxblox;  // NOLINT


bool visualizeTsdfVoxels(const TsdfVoxel& voxel, const Point& /*coord*/,
                                Color* color) {
    CHECK_NOTNULL(color);
//    if((int)voxel.color.r==255 && (int)voxel.color.g== 255 && (int)voxel.color.b == 255) {
//    if(!(voxel.color == Color::Red() || voxel.color==Color(121, 104, 120) || voxel.color == Color(108,180,39))) {
//        cout << "Distance:" << voxel.distance << endl;
//        cout << "Weight:" << voxel.weight << endl;
//        //return false;
//    }

//    if (voxel.distance >=0.4f)
//        return false;
//


    constexpr float kMinWeight = 0.1;
    if (voxel.weight > kMinWeight) {
        *color = voxel.color;
        return true;
    }
    return false;
}

bool visualizeDistanceIntensityTsdfVoxels(const TsdfVoxel& voxel,
                                                 const Point& /*coord*/,
                                                 double* intensity) {
    CHECK_NOTNULL(intensity);
    constexpr float kMinWeight = 1e-3;
    if (voxel.weight > kMinWeight) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

bool visualizeDistanceIntensityEsdfVoxels(const EsdfVoxel& voxel,
                                          const Point& /*coord*/,
                                          double* intensity) {
    CHECK_NOTNULL(intensity);
    if (voxel.observed) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

template <typename VoxelType>
void createColorPointcloudFromLayer(
        const Layer<VoxelType>& layer,
        Pointcloud* ptcloud, Colors* colors) {
    CHECK_NOTNULL(ptcloud);
    CHECK_NOTNULL(colors);
    ptcloud->clear();
    colors->clear();
    BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);

    // Cache layer settings.
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    // Temp variables.
    Color color;
    // Iterate over all blocks.
    for (const BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<VoxelType>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block;
             ++linear_index) {
            Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            if (visualizeTsdfVoxels(block.getVoxelByLinearIndex(linear_index), coord,
                             &color)) {
                ptcloud->push_back(coord);
                colors->push_back(color);
            }
        }
    }
}

void write_pointcloud_to_file(std::string filepath, Pointcloud *ptcloud, Colors* colors) {
    ofstream myfile;
    myfile.open (filepath);

    for (size_t i = 0; i < ptcloud->size(); ++i) {
        auto position = (*ptcloud)[i];
        auto color = (*colors)[i];
        myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
        myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << endl;

        myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
        myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << endl;
    }
    myfile.close();
}

int main() {
    Layer<TsdfVoxel>::Ptr layer_from_file;
    io::LoadLayer<TsdfVoxel>("/home/mansoor/tsdf_layer", &layer_from_file);
    Pointcloud ptcloud;
    Colors colors;
    createColorPointcloudFromLayer(*layer_from_file, &ptcloud, &colors);
    write_pointcloud_to_file("/home/mansoor/tsdf_pointcloud.txt",&ptcloud, &colors);
    std::cout << "Done!" << std::endl;
    return 0;
}
