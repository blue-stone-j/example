# rm -rf ./build
mkdir -p build &&
cd build &&
cmake .. &&
make -j6 &&
# ./eigen_base
# ./sensor_extrinsic_viewer
./velocity_from_imu
date "+%Y-%m-%d %H:%M:%S"
