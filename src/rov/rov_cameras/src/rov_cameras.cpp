#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <compressed_image_transport/compression_common.h>

#include <string>
#include <assert.h>
#include <vector>
#include <unordered_map>
#include <thread>
#include <glob.h>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <sstream>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

const signed int NUM_THREAD = 4;

class ROV_Cameras : public rclcpp::Node {
public:
    ROV_Cameras() : Node(std::string("rov_cameras")) {
        this->collect_cameras();

        // create camera image publishers
        this->image_publishers.emplace(std::make_pair(0, this->create_publisher<sensor_msgs::msg::CompressedImage>("cam0_image",10)));
        this->image_publishers.emplace(std::make_pair(1, this->create_publisher<sensor_msgs::msg::CompressedImage>("cam1_image",10)));
        this->image_publishers.emplace(std::make_pair(2, this->create_publisher<sensor_msgs::msg::CompressedImage>("cam2_image",10)));
        this->image_publishers.emplace(std::make_pair(3, this->create_publisher<sensor_msgs::msg::CompressedImage>("cam3_image",10)));

        // Create worker threads
        this->workers.push_back(std::thread(std::bind(&ROV_Cameras::camera_callback, this, 0, std::reference_wrapper<bool>(this->running))));
        this->workers.push_back(std::thread(std::bind(&ROV_Cameras::camera_callback, this, 1, std::reference_wrapper<bool>(this->running))));
        this->workers.push_back(std::thread(std::bind(&ROV_Cameras::camera_callback, this, 2, std::reference_wrapper<bool>(this->running))));
        this->workers.push_back(std::thread(std::bind(&ROV_Cameras::camera_callback, this, 3, std::reference_wrapper<bool>(this->running))));

        // ensure this ROS node does not exit before destructor is called, do nothing every 100ms :)
        this->create_wall_timer(100ms, std::bind(&ROV_Cameras::do_nothing, this));
    }

    ~ROV_Cameras() {
        this->running = false;
        usleep(40*1000); 
        // ensure threads are shutdown, join before destruction
        for(std::size_t i = 0; i < this->workers.size(); i ++) {
            workers[i].join();
        }
    }

private:
    void do_nothing() {

    }

    void camera_callback(int camera, std::reference_wrapper<bool> shouldBeRunning) {
        // ensure this index is valid
        if(this->cameras.size()-1 < static_cast<std::size_t>(camera))
            return;
        if(!this->cameras[camera]->isOpened()) {
            std::cout << "camera not opened" << std::endl;
            return;
        }
        cv::Mat image(240, 320, CV_8UC3);
        // cv::Mat image;
        while(shouldBeRunning.get()) {
            usleep(33 * 1000); // sleep for at least 33 milliseconds (~30fps)
            // get frame
            if(this->cameras[camera]->read(image)) {
                // to locally test cameras uncomment these lines
                // cv::imshow("test", image);
                // cv::waitKey(1);
                sensor_msgs::msg::CompressedImage msg;
                std_msgs::msg::Header header = std_msgs::msg::Header();
                header.set__stamp(this->now());
                cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
                img_bridge.toCompressedImageMsg(msg);
                this->image_publishers[camera]->publish(msg);
            } else {
                // camera didnt grab frame??? 
                // TODO: release camera, reconstruct object, insert object, try again (on a timer)
            }
        }
    }

    void collect_cameras() {
        // collect cameras from /dev/video? with glob
        glob_t glob_result;
        memset(&glob_result, 0, sizeof(glob_result));
        int ret_val = glob("/dev/video?", GLOB_TILDE, NULL, &glob_result);

        // error handling
        if(ret_val != 0) {
            globfree(&glob_result);
            std::stringstream ss;
            ss << "glob failed with return value " << ret_val << std::endl;
            throw std::runtime_error(ss.str());
        }

        // collect filenames of cameras in /dev/
        std::vector<std::string> filenames;
        for(size_t i = 0; i < glob_result.gl_pathc; i++) {
            filenames.push_back(std::string(glob_result.gl_pathv[i]));
        }

        // glob cleanup
        globfree(&glob_result);

        // test filenames to see if camera is available
        for(std::string camera_path : filenames) {
            std::string pipeline = std::string("v4l2src device="+camera_path+" io-mode=2 ! avdec_mjpeg ! videoconvert ! video/x-raw,format=BGR,height=240,width=320,framerate=30/1 ! appsink");
            std::shared_ptr<cv::VideoCapture> camera = std::make_shared<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);
            usleep(1000 * 1000); // ensure camera is captured and opened
            if(!camera->isOpened()) {
                std::cout << camera_path << " is not a camera" << std::endl;
                RCLCPP_DEBUG(this->get_logger(), "%s is not a camera or could not be opened.", camera_path);
            } else {
                std::cout << camera_path << " is a camera" << std::endl;
                camera->set(cv::CAP_PROP_FRAME_WIDTH,320);
                camera->set(cv::CAP_PROP_FRAME_HEIGHT,240);
                RCLCPP_DEBUG(this->get_logger(), "%s successfully opened as video capture object", camera_path);
                this->cameras.push_back(camera);
            }
        }
        // ensure we don't have too many cameras (allow less than 4)
        assert(this->cameras.size() <= 4);
    }

    // auto encodeImage(cv::Mat image, std::string& frame_id) {
    //     std_msgs::msg::Header header;
    //     header.frame_id = frame_id;
    //     header.stamp = this->now();
    //     return cv_bridge::CvImage(header, "bgr8", image).toCompressedImageMsg(cv_bridge::JPG);
    // }

    std::vector<std::thread> workers;
    std::vector<std::shared_ptr<cv::VideoCapture>> cameras;
    std::unordered_map<int, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>>> image_publishers;
    bool running = true;
};


int main(int argc, char ** argv) {   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROV_Cameras>());
    rclcpp::shutdown();
}