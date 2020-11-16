#include <iostream>
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/io/layer_io.h"
#include <iostream>

using namespace std;
using namespace voxblox;

bool visualizeVoxelsObstacles(const EsdfVoxel &voxel, Color *color) {
    if (voxel.observed) {
        if (voxel.distance > 1 && voxel.distance < 2) {
            *color = Color::Yellow();
            return true;
        } else if (voxel.distance > 0.5 && voxel.distance <= 1) {
            *color = Color::Orange();
            return true;
        } else if (voxel.distance > 0 && voxel.distance <= 0.5) {
            *color = Color::Red();
            return true;
        } else if (voxel.distance >= 2) {
            *color = Color::Green();
            return true;
        } else if (voxel.distance <= 0) {
            *color = Color::Gray();
            return true;
        }
    }
    return false;
}

bool visualizeVoxelsFree(const EsdfVoxel &voxel, Color *color) {
    if (voxel.observed) {

        if (voxel.distance > 1 && voxel.distance < 2) {
            *color = Color::Orange();
            return true;
        } else if (voxel.distance > 0.5 && voxel.distance <= 1) {
            *color = Color::Yellow();
            return true;
        } else if (voxel.distance > 0 && voxel.distance <= 0.5) {
            *color = Color::Green();
            return true;
        } else if (voxel.distance >= 2) {
            *color = Color::Red();
            return true;
        } else if (voxel.distance <= 0) {
            *color = Color::Gray();
            return true;
        }
    }
    return false;
}

template<typename VoxelType>
void createColorPointcloudFromLayer(
        const Layer<VoxelType> &layer,
        Pointcloud *ptcloud, Colors *colors, std::string category) {
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
    for (const BlockIndex &index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<VoxelType> &block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block;
             ++linear_index) {
            Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            if (category.compare("obstacles") != 0) {
                if (visualizeVoxelsObstacles(block.getVoxelByLinearIndex(linear_index), &color)) {
                    ptcloud->push_back(coord);
                    colors->push_back(color);
                }
            } else {
                if (visualizeVoxelsFree(block.getVoxelByLinearIndex(linear_index), &color)) {
                    ptcloud->push_back(coord);
                    colors->push_back(color);
                }
            }

        }
    }
}

void writePointcloudToFile(std::string filepath, Pointcloud *ptcloud, Colors *colors) {
    ofstream myfile;
    myfile.open(filepath);

    for (size_t i = 0; i < ptcloud->size(); ++i) {
        auto position = (*ptcloud)[i];
        auto color = (*colors)[i];

        myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
        myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << ";" << (int) color.a << endl;
    }
    myfile.close();
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: visualzie_esdf_voxels <input_layer_path> <layer_category>";
        std::cout<<"output_drivable_layer - Path to load the ESDF layer for visualization"<<std::endl;
        std::cout<<"layer_category - Layer class. [Obstacles layer or Driving zone layer] "<<std::endl;
        return -1;
    }

    Layer<EsdfVoxel>::Ptr layer_from_file;
    Pointcloud ptcloud;
    Colors colors;

    std::string esdf_path(argv[1]);
    std::string pointcloud_path = esdf_path.substr(0, esdf_path.find('.')) + "_pointcloud.txt";

    // load esdf layer
    io::LoadLayer<EsdfVoxel>(esdf_path, &layer_from_file);

    // create and save pointcloud for the esdf voxels where each poit is voxel center
    createColorPointcloudFromLayer(*layer_from_file, &ptcloud, &colors, argv[2]);
    writePointcloudToFile(pointcloud_path, &ptcloud, &colors);
    std::cout << "Saved Esdf Voxel Poincloud!" << std::endl;

    return 0;
}
