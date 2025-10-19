# rm -rf ./build
mkdir -p build &&
cd build &&
cmake .. &&
make -j6 &&
# ./eigen_base
./sensor_extrinsic_viewer
date "+%Y-%m-%d %H:%M:%S"
