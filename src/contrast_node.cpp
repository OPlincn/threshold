#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <threshold/ContrastConfig.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

// 全局变量
double g_contrast = 1.0;
int g_brightness = 50;
image_transport::Publisher pub;
ros::Publisher camera_info_pub;
sensor_msgs::CameraInfo latest_camera_info;

// 动态调参回调函数
void callback(threshold::ContrastConfig &config, uint32_t level) {
  g_contrast = config.contrast;
  g_brightness = config.brightness;
}

// 图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::Mat original_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat contrast_enhanced_image;

    // 对比度和亮度调整
    original_image.convertTo(contrast_enhanced_image, -1, g_contrast, g_brightness - 50);

    // 发布对比度增强后的图像，确保使用相同的时间戳
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", contrast_enhanced_image).toImageMsg();
    pub.publish(out_msg);

    // 发布相机信息，确保使用相同的时间戳
    latest_camera_info.header.stamp = msg->header.stamp;
    camera_info_pub.publish(latest_camera_info);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

// 相机信息回调函数
void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
  latest_camera_info = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "contrast_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  // 订阅摄像头图像话题
  image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);

  // 订阅相机信息话题
  ros::Subscriber camera_info_sub = nh.subscribe("/camera/color/camera_info", 1, cameraInfoCallback);

  // 创建图像发布者
  pub = it.advertise("/contrast/image", 1);

  // 创建相机信息发布者
  camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/contrast/camera_info", 1);

  // 设置动态调参服务器
  dynamic_reconfigure::Server<threshold::ContrastConfig> server;
  dynamic_reconfigure::Server<threshold::ContrastConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // ROS消息回调处理循环
  ros::spin();
  return 0;
}
