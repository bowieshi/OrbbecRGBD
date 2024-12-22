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



static bool  SYNC         = true;
static bool  started      = true;
static bool  hd2c         = false;
static bool  sd2c         = true;
static float alpha        = 0.5;
static int   windowWidth  = 0;
static int   windowHeight = 0;

class CameraDatastream : public rclcpp::Node
{
    public:
        CameraDatastream()
        : Node("cameraDatastream")
        {
            printf("CameraDatastream begins");
            colorPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color", 10);
            depthPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth", 10);
            
            device = pipeline.getDevice ();

            currentPath = std::filesystem::current_path();
            rootPath = currentPath;
            configPath = rootPath / "config" / "scan.yaml";

            if (readConfig() == -1) {
                RCLCPP_INFO(this->get_logger(), "[ERROR] Reading config file failed!!!!!!!");
            }
            std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

            ob::FormatConvertFilter formatConvertFilter;

            std::shared_ptr<ob::VideoStreamProfile> colorProfile  = nullptr;
            try {
                auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
                if(colorProfiles) {
                    colorProfile = colorProfiles->getVideoStreamProfile(width, height, OB_FORMAT_RGB, fps);
                }
                config->enableStream(colorProfile);
            }
            catch(ob::Error &e) {
                std::cerr << "Current device is not support color sensor!" << std::endl;
            }

            auto depthProfiles = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
            std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
            if(depthProfiles) {
                depthProfile = depthProfiles->getVideoStreamProfile(640, 576, OB_FORMAT_Y16, fps);
            }

            config->enableStream(depthProfile);
            config->setDepthScaleRequire(false);
            config->setAlignMode(ALIGN_D2C_SW_MODE);
            pipeline.enableFrameSync();

            Window app("viewer", colorProfile->width(), colorProfile->height(), RENDER_OVERLAY);

            auto startTime = std::chrono::high_resolution_clock::now();

            pipeline.start(config);
            printf("Pipeline started\n");

            saveCameraParams();

            frameCount = -1;
            while(app && rclcpp::ok()) {
                if (keyEventProcess(app, pipeline, config) == false) {
                    break;
                }
                // Wait for up to 100ms for a frameset in blocking mode.
                auto frameSet = pipeline.waitForFrames(1000);
                if(frameSet == nullptr) {
                    std::cout << "The frameset is null!" << std::endl;
                    continue;
                }
                frameCount++;

                auto colorFrame = frameSet->colorFrame();
                auto depthFrame = frameSet->depthFrame();
                if(colorFrame != nullptr && depthFrame != nullptr) {
                    app.addToRender({ colorFrame, depthFrame });
                    if(colorFrame != nullptr) {
                        if(colorFrame->format() != OB_FORMAT_RGB) {
                            if(colorFrame->format() == OB_FORMAT_MJPG) {
                                formatConvertFilter.setFormatConvertType(FORMAT_MJPG_TO_RGB888);
                            }
                            else if(colorFrame->format() == OB_FORMAT_UYVY) {
                                formatConvertFilter.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
                            }
                            else if(colorFrame->format() == OB_FORMAT_YUYV) {
                                formatConvertFilter.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
                            }
                            else {
                                std::cout << "Color format is not support!" << std::endl;
                                continue;
                            }
                            colorFrame = formatConvertFilter.process(colorFrame)->as<ob::ColorFrame>();
                        }
                        formatConvertFilter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
                        colorFrame = formatConvertFilter.process(colorFrame)->as<ob::ColorFrame>();

                        cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
                        
                        // uint32_t width = colorFrame->width();
                        // uint32_t height = colorFrame->height();
                        // uint8_t *data = (uint8_t *)colorFrame->data();
                        // uint32_t dataSize = colorFrame->dataSize();
                        // uint64_t timestamp = colorFrame-> timestamp();
                        uint64_t sysTimeStamp = colorFrame->systemTimeStamp();
                        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colorRawMat).toImageMsg();
                        msg->header.stamp = rclcpp::Time(sysTimeStamp);
                        msg->header.frame_id = std::to_string(frameCount);
                        // Publish the message
                        colorPublisher_->publish(*msg);

                    }

                    if(depthFrame != nullptr) {
                        cv::Mat depthMat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());
                        uint64_t sysTimeStamp = colorFrame->systemTimeStamp();
                        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depthMat).toImageMsg();
                        msg->header.stamp = rclcpp::Time(sysTimeStamp);
                        msg->header.frame_id = std::to_string(frameCount);
                        // Publish the message
                        depthPublisher_->publish(*msg);

                    }

                    auto currentTime = std::chrono::high_resolution_clock::now();
                    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
                    if (elapsedTime >= 1) {
                        float fps = (frameCount - lastFrameCount) / elapsedTime;
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read Frame Rate: %.2f FPS", fps);
                        startTime = currentTime;
                        lastFrameCount = frameCount;
                    }
                }
                else if (colorFrame == nullptr) {
                    std::cout << "Color frame is null!" << std::endl;
                }
                else {
                    std::cout << "Depth frame is null!" << std::endl;
                }
            }
            printf("CameraDatastream ends\n");
            pipeline.stop();
            app.close();
            printf("app closed\n");

        }

    private:
        int readConfig()
        {
            try {
                // Load the YAML file
                YAML::Node config = YAML::LoadFile(configPath);
                if (config["scene_name"]) {
                    savePath = rootPath.parent_path() / config["scene_name"].as<std::string>() / "motion";
                    std::filesystem::create_directory(savePath);
                }
                if (config["colorCamera.width"]) {
                    width = config["colorCamera.width"].as<int>();
                }
                if (config["colorCamera.height"]) {
                    height = config["colorCamera.height"].as<int>();
                }
                if (config["fps"]) {
                    fps = config["fps"].as<int>();
                }
                if (config["auto_exposure"]) {

                    bool autoExposure = config["auto_exposure"].as<bool>();
                    device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, autoExposure);
                    if (autoExposure == false) {
                        if (config["exposure"]) {
                            int32_t exposure = config["exposure"].as<int32_t>();
                            if(device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
                                OBIntPropertyRange range =device->getIntPropertyRange(OB_PROP_COLOR_EXPOSURE_INT);
                                if(exposure >=range.min && exposure <= range.max && ((exposure -range.min)%range.step == 0)) {
                                    device->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, exposure);
                                }
                                else {
                                    printf("exposure: %d is not in the range of [%d, %d] or not a multiple of %d\n", exposure, range.min, range.max, range.step);
                                    printf("set exposure error!!!!\n");
                                    return -1;
                                }
                            }
                            printf("exposure: %d\n", exposure);
                        }

                        if (config["gain"]) {
                            int32_t gain = config["gain"].as<int32_t>();
                            if(device->isPropertySupported(OB_PROP_COLOR_GAIN_INT, OB_PERMISSION_READ)) {
                                // Obtain the Color camera benefit value.
                                if(device->isPropertySupported(OB_PROP_COLOR_GAIN_INT, OB_PERMISSION_WRITE)) {
                                // Set the Color value of the camera.
                                    device->setIntProperty(OB_PROP_COLOR_GAIN_INT, gain);
                                }
                            }
                            printf("gain: %d\n", gain);
                        }
                    }
                }
                if (config["min_depth"]) {
                    if(device->isPropertySupported(OB_PROP_MIN_DEPTH_INT, OB_PERMISSION_WRITE)) {
                        // Set the minimum Depth in mm.
                        device->setIntProperty(OB_PROP_MIN_DEPTH_INT, config["min_depth"].as<int>());
                    }
                }
                if (config["max_depth"]) {
                    if(device->isPropertySupported(OB_PROP_MAX_DEPTH_INT, OB_PERMISSION_WRITE)) {
                        // Set the maximum Depth in mm.
                        device->setIntProperty(OB_PROP_MAX_DEPTH_INT, config["max_depth"].as<int>());
                    }
                }

            } catch (const YAML::Exception& e) {
                std::cerr << "Error: " << e.what() << std::endl;
                return -1;
            }
            return 0;
        }

        void saveCameraParams()
        {

            auto camera_param = pipeline.getCameraParam();
            
            try {
                YAML::Node camera_config = YAML::LoadFile(configPath);
                camera_config["colorCamera.fx"] = camera_param.rgbIntrinsic.fx;
                camera_config["colorCamera.fy"] = camera_param.rgbIntrinsic.fy;
                camera_config["colorCamera.cx"] = camera_param.rgbIntrinsic.cx;
                camera_config["colorCamera.cy"] = camera_param.rgbIntrinsic.cy;
                camera_config["colorCamera.width"] = camera_param.rgbIntrinsic.width;
                camera_config["colorCamera.height"] = camera_param.rgbIntrinsic.height;
                camera_config["colorCamera.k1"] = camera_param.rgbDistortion.k1;
                camera_config["colorCamera.k2"] = camera_param.rgbDistortion.k2;
                camera_config["colorCamera.k3"] = camera_param.rgbDistortion.p1;
                camera_config["colorCamera.k4"] = camera_param.rgbDistortion.p2;
                camera_config["colorCamera.k5"] = camera_param.rgbDistortion.k3;
                camera_config["colorCamera.k6"] = camera_param.rgbDistortion.k4;
                camera_config["colorCamera.p1"] = camera_param.rgbDistortion.k5;
                camera_config["colorCamera.p2"] = camera_param.rgbDistortion.k6;


                camera_config["depthCamera.fx"] = camera_param.depthIntrinsic.fx;
                camera_config["depthCamera.fy"] = camera_param.depthIntrinsic.fy;
                camera_config["depthCamera.cx"] = camera_param.depthIntrinsic.cx;
                camera_config["depthCamera.cy"] = camera_param.depthIntrinsic.cy;
                camera_config["depthCamera.width"] = camera_param.depthIntrinsic.width;
                camera_config["depthCamera.height"] = camera_param.depthIntrinsic.height;
                camera_config["depthCamera.k1"] = camera_param.depthDistortion.k1;
                camera_config["depthCamera.k2"] = camera_param.depthDistortion.k2;
                camera_config["depthCamera.k3"] = camera_param.depthDistortion.p1;
                camera_config["depthCamera.k4"] = camera_param.depthDistortion.p2;
                camera_config["depthCamera.k5"] = camera_param.depthDistortion.k3;
                camera_config["depthCamera.k6"] = camera_param.depthDistortion.k4;
                camera_config["depthCamera.p1"] = camera_param.depthDistortion.k5;
                camera_config["depthCamera.p2"] = camera_param.depthDistortion.k6;

                std::ofstream fout(configPath);
                fout << camera_config;
            } catch (const YAML::Exception& e) {
                std::cerr << "Error: " << e.what() << std::endl;
            } catch (const std::filesystem::filesystem_error& e) {
                std::cerr << "Filesystem Error: " << e.what() << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "General Error: " << e.what() << std::endl;
            }
        }

        // key press event processing
        bool keyEventProcess(Window &app, ob::Pipeline &pipe, std::shared_ptr<ob::Config> config) {
            ////Get the key value
            int key = app.waitKey(10);
            if(key == '+' || key == '=') {
                // Press the + key to increase alpha
                alpha += 0.1f;
                if(alpha >= 1.0f) {
                    alpha = 1.0f;
                }
                app.setAlpha(alpha);
            }
            else if(key == '-' || key == '_') {
                // press - key to decrease alpha
                alpha -= 0.1f;
                if(alpha <= 0.0f) {
                    alpha = 0.0f;
                }
                app.setAlpha(alpha);
            }
            else if(key == 'D' || key == 'd') {
                // Press the D key to switch the hardware D2C
                try {
                    if(!hd2c) {
                        started = false;
                        pipe.stop();
                        hd2c = true;
                        sd2c = false;
                        config->setAlignMode(ALIGN_D2C_HW_MODE);
                        pipe.start(config);
                        started = true;
                    }
                    else {
                        started = false;
                        pipe.stop();
                        hd2c = false;
                        sd2c = false;
                        config->setAlignMode(ALIGN_DISABLE);
                        pipe.start(config);
                        started = true;
                    }
                }
                catch(std::exception &e) {
                    std::cerr << "Property not support" << std::endl;
                }
            }
            else if(key == 'S' || key == 's') {
                // Press the S key to switch the software D2C
                try {
                    if(!sd2c) {
                        started = false;
                        pipe.stop();
                        sd2c = true;
                        hd2c = false;
                        config->setAlignMode(ALIGN_D2C_SW_MODE);
                        pipe.start(config);
                        started = true;
                    }
                    else {
                        started = false;
                        pipe.stop();
                        hd2c = false;
                        sd2c = false;
                        config->setAlignMode(ALIGN_DISABLE);
                        pipe.start(config);
                        started = true;
                    }
                }
                catch(std::exception &e) {
                    std::cerr << "Property not support" << std::endl;
                }
            }
            else if(key == 'F' || key == 'f') {
                // Press the F key to switch synchronization
                SYNC = !SYNC;
                if(SYNC) {
                    try {
                        // enable SYNChronization
                        pipe.enableFrameSync();
                    }
                    catch(...) {
                        std::cerr << "sync not support" << std::endl;
                    }
                }
                else {
                    try {
                        // turn off SYNC
                        pipe.disableFrameSync();
                    }
                    catch(...) {
                        std::cerr << "sync not support" << std::endl;
                    }
                }
            }
            else if(key == 'e' or key == 'E') {
                return false;
            }
            return true;
        }


        // void timer_callback()
        // {
        //     auto message = std_msgs::msg::String();
        //     message.data = "Hello, world! " + std::to_string(count_++);
        //     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        //     publisher_->publish(message);
        // }
        int frameCount, lastFrameCount;
        std::filesystem::path currentPath, rootPath, configPath, savePath;
        ob::Pipeline pipeline;
        std::shared_ptr<ob::Device> device;
        int width = 1280, height = 960, fps = 30;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPublisher_;
};

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "signal_handler(signum=%d)", signum);
    rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    rclcpp::spin(std::make_shared<CameraDatastream>());
    rclcpp::shutdown();
    return 0;
}
