
# AMR Software

The robot software is a single C++ executable.

In the future I might make a ROS2 repository that uses the same components, but for now I want this project to be independent from ROS so it can be more easily debugged and tested. I also want to experiment with different software architectures since I have a fair amount of ROS experience already.

## Running the Code
1. Compile: `./compile.sh`
2. Run simulation `./sim.sh` or run robot code `./robot.sh`

## Dependencies
* GCC/G++ >= 8.0
* CMake >=3.13 
* LibSerial
* Eigen
* RapidJSON
* NLOpt
* Gazebo

### GCC/G++ Upgrade
1. Install the GCC/G++ version you want
```
sudo apt-get install gcc-8 g++-8
```
2. Use `update-alternatives` to change the default version used by the OS
```
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 80 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
```

Check out this link for more info: [Link](https://linuxize.com/post/how-to-install-gcc-on-ubuntu-20-04/)

### CMake Install / Upgrade
```
sudo apt-get install build-essential libssl-dev
wget https://github.com/Kitware/CMake/releases/download/v3.20.4/cmake-3.20.4.tar.gz
tar -zxvf cmake-3.20.4.tar.gz
cd tar -zxvf cmake-3.20.4
./bootstrap
make
sudo make install
```

### LibSerial Setup
1. Clone the LibSerial repo
2. Install the dependencies
3. Run the `./compile.sh` command
4. Run `sudo make install` from the build directory
5. LibSerial will be installed to `/usr/local/lib`
- In the future this will be part of a docker container and won't require setup.


### NLOpt Setup
1. Clone the NLOpt repo
2. Follow the installation instructions (standard CMake instructions)
```
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```
3. Run `sudo ldconfig` to refresh the package list
