#include <rclcpp/rclcpp.hpp>         // Core ROS2 functionality
#include <sensor_msgs/msg/image.hpp> // ROS2 Image message type
#include <std_msgs/msg/bool.hpp>     // Boolean message for light status
#include <cv_bridge/cv_bridge.hpp>   // OpenCV-ROS2 bridge
#include <opencv2/opencv.hpp>        // OpenCV library for image processing

class BrightnessDetector : public rclcpp::Node 
{
public:
    BrightnessDetector()
        : Node("brightness_detector")  // Initialize node with name
    {
        // Declare a ROS2 parameter for the brightness threshold (default = 100.0)
        this->declare_parameter("light_threshold", 100.0);
        this->get_parameter("light_threshold", light_thresh);

        // Subscriber: Listens to image topic
        img_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&BrightnessDetector::process_image, this, std::placeholders::_1));

        // Publisher: Publishes light status as a boolean (ON/OFF)
        status_pub = this->create_publisher<std_msgs::msg::Bool>("/light_status", 10);
    }

private:
    // Image processing callback function
    void process_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;

        // Converts ROS2 image to OpenCV format
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge failed: %s", e.what());
            return;
        }

        //Computes average brightness
        double avg_brightness = cv::mean(frame)[0];

        //Determine if light is ON or OFF
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = avg_brightness > light_thresh;

        // 🔹 Publish the status
        status_pub->publish(status_msg);

        //Log the result(debugging)
        RCLCPP_INFO(this->get_logger(), "Avg Brightness: %.2f | Threshold: %.2f | Light: %s",
                    avg_brightness, light_thresh, status_msg.data ? "ON" : "OFF");
    }

    //ROS2 Communication Objects
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub; // Image subscriber
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub;     // Light status publisher

    //hreshold for brightness detection
    double light_thresh;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightnessDetector>());
    rclcpp::shutdown();
    return 0;
}
