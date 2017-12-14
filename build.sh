sudo apt-get install libbullet-dev
sudo apt-get install libglfw3-dev


cd third_party && rm -rf *
git clone git@github.com:benikabocha/saba.git
cd saba && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=RELEASE .. && make -j2

cd ../..

git clone git@github.com:stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build && cmake .. && make -j2

cd ../../..

mkdir build && cd build
cmake .. && make
