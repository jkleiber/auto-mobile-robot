
# AMR Software

The robot software is a single C++ executable.

In the future I might make a ROS2 repository that uses the same components, but for now I want this project to be independent from ROS so it can be more easily debugged and tested. I also want to experiment with different software architectures since I have a fair amount of ROS experience already.

## Dependencies
* LibSerial
* Eigen
* RapidJSON

### LibSerial Setup
1. Clone the LibSerial repo
2. Install the dependencies
3. Run the `./compile.sh` command
4. Run `sudo make install` from the build directory
5. LibSerial will be installed to `/usr/local/lib`
- In the future this will be part of a docker container and won't require setup.