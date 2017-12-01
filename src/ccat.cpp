#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "image window";

static const std::string node_image_raw = "camera/output";
static const std::string node_image_pub = "/CCAT_image";

class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;

public:
  ImageConverter()
  : it_(nh_)
  {
    img_sub_ = it_.subscribe(node_image_raw, 1, &ImageConverter::colorSel, this);
    img_pub_ = it_.advertise(node_image_pub,1);

  }

  void colorSel(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Something is wrong, cv_bridge exception: %s", e.what());
      return;
    }

    //cv::cvtColor(

    sensor_msgs::ImagePtr pub_img = cv_bridge::CvImage(std_msgs::Header(),"rgb8",cv_ptr->image).toImageMsg();
    img_pub_.publish(pub_img);
  }
};

int main(int argc,char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
