#include <std_msgs/msg/detail/float32__struct.hpp>
#include <stdlib.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/float32.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter : public rclcpp::Node {
private:
    const std::string OPENCV_WINDOW = "Image window";
//    image_transport::ImageTransport it;
    image_transport::Subscriber sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curr_sub;
    float curr= 0;
    double nx=1.57;
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat arr = cv_ptr->image;
        
        int tot_x = 0;
        int cnt = 0;
        for(int x = 0 ; x< arr.rows; x++){
          for(int y = 0; y< arr.cols; y++){
            if(arr.ptr<cv::Vec3b>(x)[y][2] > arr.ptr<cv::Vec3b>(x)[y][1] && arr.ptr<cv::Vec3b>(x)[y][2] >arr.ptr<cv::Vec3b>(x)[y][0]){
              tot_x += y;
              cnt++;      
            }
          }
        }
        if (cnt == 0) {
            RCLCPP_WARN(this->get_logger(), "No target detected");
            std_msgs::msg::Float32 ang;
            ang.data = this->curr  + (this->nx/4);
            pub_->publish(ang);
            return;  
        }
        double error = ((arr.cols/2) - ((double)tot_x/cnt));
        double dx = (error/arr.cols) * (this->nx/4);

        std_msgs::msg::Float32 ang;
        ang.data = this->curr  + dx;
        pub_->publish(ang);
        RCLCPP_INFO(this->get_logger(), "curr : %f", ang.data);
    }

    void angle_callback(std_msgs::msg::Float32::SharedPtr msg){
        this->curr = msg->data;

    }

        

    

public:
    ImageConverter() : Node("mynode") {
//    ImageConverter() : Node("image_converter"), it(this->get()) {

        // Open demo window that will show output image
        cv::namedWindow(OPENCV_WINDOW);

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        sub_ = image_transport::create_subscription(this, "robotcam",
                std::bind(&ImageConverter::imageCallback, this, std::placeholders::_1), "raw", custom_qos);
        pub_ = this->create_publisher<std_msgs::msg::Float32>("desired_angle", 10);
        curr_sub = this->create_subscription<std_msgs::msg::Float32>(
    "/current_angle", 10, std::bind(&ImageConverter::angle_callback, this, std::placeholders::_1));
//        pub = it.advertise("out_image_base_topic", 1);
//        sub = it.subscribe("in_image_base_topic", 1, imageCallback);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}