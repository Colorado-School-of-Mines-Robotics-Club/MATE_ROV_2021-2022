#include <assert.h>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <compressed_image_transport/compression_common.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/opencv.hpp>

#include <nanogui/nanogui.h>

#include <eigen3/Eigen/Core>

class BS_GUI : public rclcpp::Node {
public:
    BS_GUI() : Node("bs_gui") {
        std::cout << "we constructing" << std::endl;
        // camera streams
        // TODO: consider making these multithreaded
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr)> callback0 = std::bind(&BS_GUI::camera_callback, this, 0, std::placeholders::_1);
        camera_image_subscriptions.emplace(std::make_pair(0, this->create_subscription<sensor_msgs::msg::CompressedImage>("cam0_image", 10, callback0)));
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr)> callback1 = std::bind(&BS_GUI::camera_callback, this, 1, std::placeholders::_1);
        camera_image_subscriptions.emplace(std::make_pair(1, this->create_subscription<sensor_msgs::msg::CompressedImage>("cam1_image", 10, callback1)));
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr)> callback2 = std::bind(&BS_GUI::camera_callback, this, 2, std::placeholders::_1);
        camera_image_subscriptions.emplace(std::make_pair(2, this->create_subscription<sensor_msgs::msg::CompressedImage>("cam2_image", 10, callback2)));
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr)> callback3 = std::bind(&BS_GUI::camera_callback, this, 3, std::placeholders::_1);
        camera_image_subscriptions.emplace(std::make_pair(3, this->create_subscription<sensor_msgs::msg::CompressedImage>("cam3_image", 10, callback3)));

        // camera controllers
        camera_control_publishers.emplace(std::make_pair(0, this->create_publisher<std_msgs::msg::Bool>("cam0_control", 10)));
        camera_control_publishers.emplace(std::make_pair(1, this->create_publisher<std_msgs::msg::Bool>("cam1_control", 10)));
        camera_control_publishers.emplace(std::make_pair(2, this->create_publisher<std_msgs::msg::Bool>("cam2_control", 10)));
        camera_control_publishers.emplace(std::make_pair(3, this->create_publisher<std_msgs::msg::Bool>("cam3_control", 10)));

        // monitor joystick
        std::function<void(const sensor_msgs::msg::Joy::SharedPtr)> joy_callback = std::bind(&BS_GUI::joystick_callback, this, std::placeholders::_1);
        joystick_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

        // texture streams for changing buffers
        // this->cam0_image = std::make_shared<SDL_Texture>(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING, 320, 240));
        // this->cam1_image = std::make_shared<SDL_Texture>(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING, 320, 240));
        // this->cam2_image = std::make_shared<SDL_Texture>(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING, 320, 240));
        // this->cam3_image = std::make_shared<SDL_Texture>(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING, 320, 240));
    }

private:
    cv::Mat* msgtomat(const sensor_msgs::msg::CompressedImage::SharedPtr& msg) {
        cv::Mat bgr(cv::Size(1,msg->data.size()), CV_8UC3, msg->data.data());
        cv::Mat* rgba = new cv::Mat(cv::Size(1,msg->data.size()), CV_8UC4, msg->data.data());
        cv::cvtColor(cv::imdecode(bgr, cv::IMREAD_ANYCOLOR), *rgba, cv::COLOR_BGR2RGBA);
        return rgba;
    }

    void camera_callback(int camera_idx, const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat* RGBA_IMAGE = msgtomat(msg);

        //todo: display image to screen
    }

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        *this->joystick.get() = *msg.get();
    }

    std::unordered_map<int, std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>>> camera_image_subscriptions;
    std::unordered_map<int, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>> camera_control_publishers;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joystick_subscription;

    std::shared_ptr<sensor_msgs::msg::Joy> joystick;
};

void target(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BS_GUI>());
    rclcpp::shutdown();
}


int main(int argc, char ** argv) {
    nanogui::init();

    std::shared_ptr<nanogui::Screen> screen = std::make_shared<nanogui::Screen>(nanogui::Vector2i(100,100), "Bruh");
    // add widgets
    std::shared_ptr<nanogui::Button> button = std::make_shared<nanogui::Button>(screen.get());
    screen->setVisible(true);
    screen->performLayout();
    
    // ros thread
    std::thread t(std::bind(&target, argc, argv));

    // run screen's mainloop
    nanogui::mainloop();
    return 0;
}