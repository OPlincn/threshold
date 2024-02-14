#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <threshold/ThresholdConfig.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

// 全局变量，用于存储当前的阈值和发布者
int g_threshold_value = 128;
image_transport::Publisher pub;
ros::Publisher camera_info_pub;
sensor_msgs::CameraInfo latest_camera_info;

// 回调函数，用于在动态调参时更新阈值
void callback(threshold::ThresholdConfig &config, uint32_t level) {
  g_threshold_value = config.threshold;
}

// 回调函数，用于处理图像的回调
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::Mat color_image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // 将彩色图像转换为灰度图像
    cv::Mat gray_image;
    cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);

    // 应用二值化
    cv::Mat threshold_image;
    cv::threshold(gray_image, threshold_image, g_threshold_value, 255, cv::THRESH_BINARY);

    // 发布二值化后的图像，确保使用相同的时间戳
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "mono8", threshold_image).toImageMsg();
    pub.publish(out_msg);

    // 发布相机信息，确保使用相同的时间戳
    latest_camera_info.header.stamp = msg->header.stamp;
    camera_info_pub.publish(latest_camera_info);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

// 相机信息的回调函数
void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
  latest_camera_info = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "threshold_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  // 订阅摄像头图像话题
  image_transport::Subscriber sub = it.subscribe("/contrast/image", 1, imageCallback);

  // 订阅相机信息话题
  ros::Subscriber camera_info_sub = nh.subscribe("/contrast/camera_info", 1, cameraInfoCallback);

  // 创建图像发布者
  pub = it.advertise("/threshold/image", 1);

  // 创建相机信息发布者
  camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/threshold/camera_info", 1);

  // 设置动态调参服务器
  dynamic_reconfigure::Server<threshold::ThresholdConfig> server;
  dynamic_reconfigure::Server<threshold::ThresholdConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // ROS消息回调处理循环
  ros::spin();
  return 0;
}
