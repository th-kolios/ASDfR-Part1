#include <rclcpp/rclcpp.hpp>         // Core ROS2 library
#include <sensor_msgs/msg/image.hpp> // ROS2 Image message type
#include <std_msgs/msg/int32_multi_array.hpp> // To publish object position (x, y)
#include <cv_bridge/cv_bridge.hpp>   // OpenCV-ROS2 bridge
#include <opencv2/opencv.hpp>        // OpenCV for image processing

class TrackerNode : public rclcpp::Node 
{
public:
    TrackerNode() : Node("tracker") 
    {
        // Declare and get threshold parameter
        this->declare_parameter("threshold", 200);
        this->get_parameter("threshold", threshold_);

        // Subscribe to the camera image topic
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&TrackerNode::processImage, this, std::placeholders::_1));

        // Publisher for detected object position
        pos_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/object_position", 10);

        // Parameter update callback
        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&TrackerNode::paramCallback, this, std::placeholders::_1));
    }

private:
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame, gray, thresholded;

        // Convert ROS2 image to OpenCV
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge failed: %s", e.what());
            return;
        }

        // Convert to grayscale
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Apply binary thresholding
        cv::threshold(gray, thresholded, threshold_, 255, cv::THRESH_BINARY);

        // Compute Center of Gravity
        cv::Moments m = cv::moments(thresholded, true);
        int x_cog = (m.m00 != 0) ? static_cast<int>(m.m10 / m.m00) : -1;
        int y_cog = (m.m00 != 0) ? static_cast<int>(m.m01 / m.m00) : -1;

        // Publish CoG position
        auto pos_msg = std_msgs::msg::Int32MultiArray();
        pos_msg.data = {x_cog, y_cog};
        pos_pub_->publish(pos_msg);

        RCLCPP_INFO(this->get_logger(), "Object Position: x=%d, y=%d", x_cog, y_cog);
    }

    // Parameter update function
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            if (param.get_name() == "threshold")
            {
                threshold_ = param.as_int();
                RCLCPP_INFO(this->get_logger(), "Updated threshold to: %d", threshold_);
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    // ROS2 Communication Objects
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pos_pub_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    int threshold_; // Threshold for object detection
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackerNode>());
    rclcpp::shutdown();
    return 0;
}
