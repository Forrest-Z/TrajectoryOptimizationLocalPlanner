#!/bin/bash
# remove previous output

rm -rf output
mkdir output

# switch to build directory
if [ -d build ]; then
  echo "moving to build directory"
  cd build
  make
else
  echo "Compiling code..."
  bash scripts/compile.sh
  cd build
fi

scenario=multi
# update with the provided arguments
if [[ $# -gt 0 ]]; then
  scenario=$1
fi

if [[ $# -gt 2 ]]; then
  start=$2
  goal=$3
else
  # configure default arguments start and goal positions
  case $scenario in
  simple)
    start="-7,-0.2,0.5"
    goal="7,-0.7,0.5"
    ;;
  multi)
    start="-9,0.7,0.5"
    goal="8,-0.7,0.5"
    ;;

  slope)
    start="-7,1,1"
    goal="8,1,1.3"
    ;;
  *)
    echo $"Invalid scenario specified: Choose from {simple|multi|slope}"
    exit 1
    ;;
  esac
fi

#exit
#generate tsdf and pointcloud
echo "generating tsdf's and scene pointcloud"
./generate_tsdf $scenario ../output/tsdf_obstacles_layer.layer ../output/tsdf_free_layer.layer ../output/pointcloud_${scenario}.txt

# generate esdf
echo "generating esdf's as pointcloud"
./esdf_from_tsdf ../output/tsdf_free_layer.layer ../output/esdf_free_layer.layer free
./esdf_from_tsdf ../output/tsdf_obstacles_layer.layer ../output/esdf_obstacles_layer.layer obstacles

# save esdf voxels as point cloud
echo "saving esdf's as pointcloud"
./visualize_esdf_voxels ../output/esdf_obstacles_layer.layer obstacles
./visualize_esdf_voxels ../output/esdf_free_layer.layer free

# planning
./planning $start $goal ../output/esdf_obstacles_layer.layer ../output/esdf_free_layer.layer ../output/pointcloud_${scenario}.txt
