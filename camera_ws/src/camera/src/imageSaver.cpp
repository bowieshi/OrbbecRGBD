#include <functional>
#include <memory>
#include <fstream>
#include <filesystem>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>


class colorSaver : public rclcpp::Node
{
    public:
        colorSaver()
        : Node("colorSaver")
        {
            colorCount = 0;
            depthCount = 0;

            currentPath = std::filesystem::current_path();


            std::string dirName = getCurrentTimeAsString();
            dirPath = currentPath / "motion" / dirName; // Adjust the path as needed

            if (std::filesystem::create_directory(dirPath)) {
                std::cout << "Directory created: " << dirPath << std::endl;
                colorPath = dirPath / "images";
                depthPath = dirPath / "depth";
                std::filesystem::create_directory(colorPath);
                std::filesystem::create_directory(depthPath);
            } else {
                std::cerr << "Directory already exists or could not be created." << std::endl;
            }
            

            
            callback_group_subscriber1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_subscriber2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            
            // Each of these callback groups is basically a thread
            // Everything assigned to one of them gets bundled into the same thread
            auto sub1_opt = rclcpp::SubscriptionOptions();
            sub1_opt.callback_group = callback_group_subscriber1_;
            auto sub2_opt = rclcpp::SubscriptionOptions();
            sub2_opt.callback_group = callback_group_subscriber2_;

            colorSubscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/color", 30, std::bind(&colorSaver::colorCallback, this, std::placeholders::_1), sub1_opt);
            depthSubscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/depth", 30, std::bind(&colorSaver::depthCallback, this, std::placeholders::_1), sub2_opt);


        }

    private:
        std::string getCurrentTimeAsString() {
            // Get the current time
            auto now = std::chrono::system_clock::now();
            auto time_t_now = std::chrono::system_clock::to_time_t(now);
            
            // Convert to local time
            std::tm local_tm = *std::localtime(&time_t_now);
            
            // Format the time as a string
            std::ostringstream oss;
            oss << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S"); // Format: YYYY-MM-DD_HH-MM-SS
            return oss.str();
        }

        std::string formatNumber(int number) {
            std::ostringstream oss;
            oss << std::setw(6) << std::setfill('0') << number; // Set width to 6 and fill with '0'
            return oss.str();
        }
        void colorCallback(const sensor_msgs::msg::Image img)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            int frame_id = std::stoi(img.header.frame_id);

            std::string filename = colorPath / ("color_" + formatNumber(frame_id) + ".jpg");
            cv::imwrite(filename, cv_ptr->image);
            colorCount ++;
            // RCLCPP_INFO(this->get_logger(), "%d color image saved to %s", colorCount, filename.c_str());

            auto currentTime = std::chrono::high_resolution_clock::now();
            auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
            if (elapsedTime >= 1) {
                float fps = (colorCount - lastFrameCount) / elapsedTime;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Write Frame Rate: %.2f FPS", fps);
                startTime = currentTime;
                lastFrameCount = colorCount;
            }
            
        }
        void depthCallback(const sensor_msgs::msg::Image img)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            int frame_id = std::stoi(img.header.frame_id);

            std::string filename = depthPath / ("depth_" + formatNumber(frame_id) + ".png");
            cv::imwrite(filename, cv_ptr->image);
            depthCount ++;
            // RCLCPP_INFO(this->get_logger(), "%d depth image saved to %s", depthCount, filename.c_str());
            
        }
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorSubscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscription_;
        std::filesystem::path dirPath, currentPath, colorPath, depthPath;
        int colorCount, depthCount;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
        int frameCount, lastFrameCount = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<colorSaver>());
    rclcpp::shutdown();
    return 0;
}