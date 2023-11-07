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
```

# qt5 install
```bash
sudo apt-get install qt5-default
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
./robotSim
```


# 문제점
make install할 경우
```
/usr/local/lib
``
위의 폴더에 install됨, 그런데 ros2나 ros가 설치되어 있는경우

```
/usr/lib/x86_64-linux-gnu
```
위 폴더에 libBullet*.so.2.88 들이 설치되어 있음

따라서 충돌을 막기 위해 다음과 같이 설정
```
sudo cp  /usr/local/lib/libBullet* /usr/lib/x86_64-linux-gnu
```

