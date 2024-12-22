#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <filesystem>
#include <cstdio>
#include <memory>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "libobsensor/ObSensor.hpp"
#include "opencv2/opencv.hpp"
#include "utils.hpp"
#include "window.hpp"


using namespace std::chrono_literals;

std::mutex printerMutex;
class CameraIMUStream : public rclcpp::Node
{
    public:
        CameraIMUStream()
        : Node("CameraIMUStream")
        {
            devList = ctx.queryDeviceList();

            if(devList->deviceCount() == 0) {
                std::cerr << "Device not found!" << std::endl;
                return;
            }
                    // Create a device, 0 represents the index of the first device
            dev = devList->getDevice(0);
            try {
                // Get Gyroscope Sensor
                gyroSensor = dev->getSensorList()->getSensor(OB_SENSOR_GYRO);
                if(gyroSensor) {
                    // Get configuration list
                    auto profiles = gyroSensor->getStreamProfileList();
                    // Select the first profile to open stream
                    auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
                    gyroSensor->start(profile, [](std::shared_ptr<ob::Frame> frame) {
                        std::unique_lock<std::mutex> lk(printerMutex);
                        auto                         timeStamp = frame->systemTimeStamp();
                        auto                         index     = frame->index();
                        auto                         gyroFrame = frame->as<ob::GyroFrame>();
                        if(gyroFrame != nullptr && (index % 50) == 2) {  //( timeStamp % 500 ) < 2: Reduce printing frequency
                            auto value = gyroFrame->value();
                            std::cout << "Gyro Frame: \n\r{\n\r"
                                    << "  tsp = " << timeStamp << "\n\r"
                                    << "  temperature = " << gyroFrame->temperature() << "\n\r"
                                    << "  gyro.x = " << value.x << " rad/s"
                                    << "\n\r"
                                    << "  gyro.y = " << value.y << " rad/s"
                                    << "\n\r"
                                    << "  gyro.z = " << value.z << " rad/s"
                                    << "\n\r"
                                    << "}\n\r" << std::endl;
                        }
                    });
                }
                else {
                    std::cout << "get gyro Sensor failed ! " << std::endl;
                }
            }
            catch(ob::Error &e) {
                std::cerr << "current device is not support imu!" << std::endl;
                exit(EXIT_FAILURE);
            }

            // Get Acceleration Sensor
            accelSensor = dev->getSensorList()->getSensor(OB_SENSOR_ACCEL);
            if(accelSensor) {
                // Get configuration list
                auto profiles = accelSensor->getStreamProfileList();
                // Select the first profile to open stream
                auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
                accelSensor->start(profile, [](std::shared_ptr<ob::Frame> frame) {
                    std::unique_lock<std::mutex> lk(printerMutex);
                    auto                         timeStamp  = frame->systemTimeStamp();
                    auto                         index      = frame->index();
                    auto                         accelFrame = frame->as<ob::AccelFrame>();
                    if(accelFrame != nullptr && (index % 50) == 0) {
                        auto value = accelFrame->value();
                        std::cout << "Accel Frame: \n\r{\n\r"
                                << "  tsp = " << timeStamp << "\n\r"
                                << "  temperature = " << accelFrame->temperature() << "\n\r"
                                << "  accel.x = " << value.x << " m/s^2"
                                << "\n\r"
                                << "  accel.y = " << value.y << " m/s^2"
                                << "\n\r"
                                << "  accel.z = " << value.z << " m/s^2"
                                << "\n\r"
                                << "}\n\r" << std::endl;
                    }
                });
            }
            else {
                std::cout << "get Accel Sensor failed ! " << std::endl;
            }

            std::cout << "Press ESC to exit! " << std::endl;
        }

    private:

        std::filesystem::path currentPath, rootPath, configPath, savePath;
        ob::Context ctx;
        std::shared_ptr<ob::DeviceList> devList;
        // Create a device, 0 represents the index of the first device
        std::shared_ptr<ob::Device> dev;
        std::shared_ptr<ob::Sensor> gyroSensor  = nullptr;
        std::shared_ptr<ob::Sensor> accelSensor = nullptr;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraIMUStream>());
    rclcpp::shutdown();
    return 0;
}
