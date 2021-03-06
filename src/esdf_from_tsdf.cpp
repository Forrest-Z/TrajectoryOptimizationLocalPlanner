#include <iostream>
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/io/layer_io.h"

using namespace voxblox;

int main(int argc, char *argv[]) {

    if (argc < 4) {
        std::cout << "Usage: esdf_from_tsdf <input_layer> <output_layer> <category>";
        std::cout << "input_layer - Path to load the input TSDF layer" << std::endl;
        std::cout << "output_drivable_layer - Path to save the ESDF layer" << std::endl;
        std::cout << "category - Category of the ESDF map. Choose fro [obstacles, free]" << std::endl;
        return -1;
    }

    Layer<TsdfVoxel>::Ptr layer_from_file;
    io::LoadLayer<TsdfVoxel>(argv[1], &layer_from_file);
    EsdfMap::Config esdf_config;

    // Same number of voxels per side and size for ESDF as with TSDF
    esdf_config.esdf_voxels_per_side = layer_from_file->voxels_per_side();
    esdf_config.esdf_voxel_size = layer_from_file->voxel_size();

    // esdf integration config
    EsdfIntegrator::Config esdf_integrator_config;
    esdf_integrator_config.semantic_masking = strcmp(argv[3], "obstacles") == 0;
    EsdfMap esdf_map(esdf_config);
    EsdfIntegrator esdf_integrator(esdf_integrator_config,
                                   layer_from_file.get(),
                                   esdf_map.getEsdfLayerPtr());

    // esdf integration
    esdf_integrator.updateFromTsdfLayerBatch();
    const bool esdf_success = io::SaveLayer(esdf_map.getEsdfLayer(), argv[2]);

    if (esdf_success == false) {
        throw std::runtime_error("Failed to save ESDF");
    }
    std::cout << "Saved Esdf from tsdf!" << std::endl;
    return 0;
}
