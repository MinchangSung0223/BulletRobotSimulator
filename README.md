# bullet3 install
먼저 bullet3를 build해야 합니다. 
임의의 폴더에 bullet3를 내려받고 다음과정대로 build를 진행합니다.
```bash
  git clone https://github.com/bulletphysics/bullet3.git
  cd bullet3  
  export BULLET_HOME=${PWD}
  ./build_cmake_pybullet_double.sh
  cd build_cmake
  sudo make install
  sudo  ln -s $BULLET_HOME /opt/bullet3
  sudo cp /usr/local/lib/libBullet* /usr/lib/x86_64-linux-gnu
```
# cxxopts install
```bash
git clone https://github.com/jarro2783/cxxopts.git
cd cxxopts
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

# qt5 install
```bash
sudo apt-get install qt5-default
22.04 : sudo apt install qtbase5-dev qt5-qmake

sudo apt install qtdeclarative5-dev
```
# library install(jsoncpp, eigen3, spdlog)
```bash
sudo apt-get install libjsoncpp-dev libeigen3-dev libspdlog-dev
```


#  build

```bash
git clone https://github.com/MinchangSung0223/BulletRobotSimulator.git
cd BulletRobotSimulator
mkdir build
cd build
cmake ..
make j$(nproc)
./robotSim --urdf ../urdf/231103_CEAEASR_min/urdf/231103_CEAEASR_min.urdf --gui --control --qt
```




