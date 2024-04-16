#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

struct CarTrackingMessage {
    uint8_t id;
    uint16_t position_x;
    uint16_t position_y;
    float orientation;
};

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor") {
        this->subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", 10,
            std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));

        this->publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("debug/image_raw/compressed", 10);
        this->declare_parameter<uint8_t>("id", 0);
        this->declare_parameter<int>("min_hsv_value", 210);
        this->get_parameter("id", id);
        this->get_parameter("min_hsv_value", min_hsv_value);
        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Could not create socket");
        }
        memset(&multicast_addr, 0, sizeof(multicast_addr));
        multicast_addr.sin_family = AF_INET;
        multicast_addr.sin_addr.s_addr = inet_addr("239.255.255.250"); // Multicast address
        multicast_addr.sin_port = htons(5565); // Port number
    }   

private:
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            auto cv_ptr = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            processImage(cv_ptr);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }

    void processImage(cv::Mat &image) {
        // Correct lens distortion
        cv::Mat image_undistorted;
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 635.061323, 0, 325.876763, 0, 636.236402, 252.600209, 0, 0, 1);
        cv::Mat distortion_coefficients = (cv::Mat_<double>(5, 1) << -0.504028, 0.251295, -0.006413, 0.002310, 0);
        cv::undistort(image, image_undistorted, camera_matrix, distortion_coefficients);

        // Convert to HSV and filter for white points
        cv::Mat hsv, mask;
        cv::cvtColor(image_undistorted, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(80, 0, min_hsv_value), cv::Scalar(150, 100, 255), mask);

        // Find contours and white points
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> white_points;
        const double MAX_WHITE_AREA = 30.0;
        for (auto &contour : contours) {
            if (cv::contourArea(contour) < MAX_WHITE_AREA) {
                auto M = cv::moments(contour);
                if (M.m00 != 0) {
                    int cx = int(M.m10 / M.m00);
                    int cy = int(M.m01 / M.m00);
                    white_points.push_back(cv::Point(cx, cy));
                }
            }
        }

        cv::Point front_points[2];
        cv::Point rear_point;
        const double MAX_PERP_DISTANCE = 34.0;
        for (auto &p1 : white_points) {
            for (auto &p2 : white_points) {
                if (p1 != p2 && distance(p1, p2) < 12) {
                    for (auto &p3 : white_points) {
                        if (p3 != p1 && p3 != p2) {
                            if (perp_distance(p3, p1, p2) < MAX_PERP_DISTANCE && distance(p1, p3) < MAX_PERP_DISTANCE and distance(p2, p3) < MAX_PERP_DISTANCE) {
                                front_points[0] = p1;
                                front_points[1] = p2;
                                rear_point = p3;
                            }
                        }
                    }
                }
            }
        }

        if (rear_point != cv::Point(0,0)) {
          cv::Point front_point = cv::Point((front_points[0].x + front_points[1].x) / 2, (front_points[0].y + front_points[1].y) / 2);
          double orientation = clip_angle(atan2(front_point.y - rear_point.y, front_point.x - rear_point.x));
          cv::Point position = rear_point;

          std::vector<cv::Point> points = {front_points[0], front_points[1], rear_point};
          cv::polylines(image_undistorted, points, true, cv::Scalar(0, 255, 0), 2);

          //RCLCPP_INFO(this->get_logger(), "Position: (%d, %d), Orientation: %f", position.x, position.y, orientation);
          CarTrackingMessage message;
          message.id = id;
          message.position_x = position.x;
          message.position_y = position.y;
          message.orientation = orientation;
          if (sendto(socket_fd, &message, sizeof(message), 0, (struct sockaddr *)&multicast_addr, sizeof(multicast_addr)) < 0) {
              RCLCPP_ERROR(this->get_logger(), "Could not send message");
          }
        }
        auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_undistorted).toCompressedImageMsg();
        publisher_->publish(*message.get());

        //cv::imshow("Image", image_undistorted);
        //cv::imshow("Mask", mask);
        //cv::waitKey(1);
    }

    // Helper function to calculate distance between two points
    double distance(const cv::Point &p1, const cv::Point &p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    // Helper function to calculate perpendicular distance of p from the line formed by p1 and p2
    double perp_distance(const cv::Point &p, const cv::Point &p1, const cv::Point &p2) {
        double result;
        double num = abs((p2.y - p1.y) * p.x - (p2.x - p1.x) * p.y + p2.x * p1.y - p2.y * p1.x);
        double den = distance(p1, p2);
        result = num / den;
        return result;
    }

    double clip_angle(double angle) {
      while (angle > M_PI) angle -= 2 * M_PI;
      while (angle < -M_PI) angle += 2 * M_PI;
      return angle;
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    int socket_fd;
    struct sockaddr_in multicast_addr;
    uint8_t id;
    int min_hsv_value;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
