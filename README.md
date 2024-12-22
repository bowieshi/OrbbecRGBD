To run Orbbec Femto Bolt RGBD Camera:
Please run:
```
python gui.py
```
under the project root directory

Some prerequisite:

1. You need to install ROS2 Humble from https://docs.ros.org/en/humble/index.html
2. You need to download and compile Orbbec Femto Bolt Libaray from https://github.com/orbbec/OrbbecSDK
3. change the CMakeList in `camera_ws/src/camera/CMakeLists.txt` to link your Orbbec lib in you computer.