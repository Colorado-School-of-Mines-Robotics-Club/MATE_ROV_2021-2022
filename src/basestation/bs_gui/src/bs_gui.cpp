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
#include <std_msgs/msg/bool.hpp>

#include <opencv2/opencv.hpp>

#include <SDL.h>

class BS_GUI : public rclcpp::Node {
public:
    BS_GUI() : Node("bs_gui") {
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

        // texture streams for changing buffers
        this->cam0_image = std::make_shared<SDL_Texture>(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING, 320, 240));
        this->cam1_image = std::make_shared<SDL_Texture>(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING, 320, 240));
        this->cam2_image = std::make_shared<SDL_Texture>(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING, 320, 240));
        this->cam3_image = std::make_shared<SDL_Texture>(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING, 320, 240));
    }


private:
    // responsibility is on you to dispose of pixel buffer after it is changed :)
    cv::Mat* compressed_imgmsg_to_sdl_texture(SDL_Texture* tex, const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat buf(cv::Size(1,msg->data.size()), CV_8UC1, msg->data.data());
        cv::Mat* image = new cv::Mat(cv::imdecode(buf, cv::IMREAD_ANYCOLOR));

        // or however else you want to do this
        SDL_UpdateTexture(tex, NULL, (void*)image->data, buf.step1());
        return image;
    }

    void camera_callback(int camera_idx, const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        camera_idx;
        msg;
    }

    std::shared_ptr<SDL_Renderer> renderer;
    // can't do this, compiler doesnt like this (undefined struct issue)
    // FIXME
    std::shared_ptr<SDL_Texture> cam0_image;
    std::shared_ptr<SDL_Texture> cam1_image;
    std::shared_ptr<SDL_Texture> cam2_image;
    std::shared_ptr<SDL_Texture> cam3_image;

    std::unordered_map<int, std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>>> camera_image_subscriptions;
    std::unordered_map<int, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>> camera_control_publishers;
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BS_GUI>());
    rclcpp::shutdown();
    return 0;
}
