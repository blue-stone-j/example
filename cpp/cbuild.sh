
mkdir -p build &&
cd build &&
cmake .. &&
make -j10 &&
./src_omp

date "+%Y-%m-%d %H:%M:%S"