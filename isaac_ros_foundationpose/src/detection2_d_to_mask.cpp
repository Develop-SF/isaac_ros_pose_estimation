// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>



namespace nvidia
{
namespace isaac_ros
{
namespace foundationpose
{

/*
ROS 2 node that generates a binary segmentation mask image from
1. a vision_msgs::msg::Detection2D or vision_msgs::msg::Detection2DArray
2. image_height and image_width read from ROS parameters
*/
class Detection2DToMask : public rclcpp::Node
{
public:
  explicit Detection2DToMask(const rclcpp::NodeOptions & options)
  : Node("detection2_d_to_mask", options)
  {
    object_name_mapping_ = std::map<std::string, std::vector<std::string>>{
      {"seasoning", {"調味料","調味","料","味料"}},
      {"soy_sauce", {"醬油","醬"}},
      {"grape_juice", {"葡萄汁","葡萄","汁"}},
      {"cook_oil", {"食用油","食用"}}
    };
    
    // Create a publisher for the mono8 image
    // image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("segmentation", 10);

    this->declare_parameter<int>("mask_width", 640);
    this->declare_parameter<int>("mask_height", 640);
    this->declare_parameter<std::vector<std::string>>("object_list", {"seasoning", "soy_sauce"});
    
    this->get_parameter("object_list", object_list_);
    this->get_parameter("mask_width", mask_width_);
    this->get_parameter("mask_height", mask_height_);

    for (const auto & object : object_list_) {
      image_pub_[object] = this->create_publisher<sensor_msgs::msg::Image>("yolo_segmentation_" + object, 10);
    } 

    detection2_d_array_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "detection2_d_array", 10,
      std::bind(&Detection2DToMask::boundingBoxArrayCallback, this, std::placeholders::_1));
    // Create subscriber for Detection2D
    // detection2_d_sub_ = this->create_subscription<vision_msgs::msg::Detection2D>(
    //   "detection2_d", 10,
    //   std::bind(&Detection2DToMask::boundingBoxCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Mask Height: %d, Mask Width: %d",
      mask_height_, mask_width_);
  }

  // void boundingBoxCallback(const vision_msgs::msg::Detection2D::SharedPtr msg)
  // {
  //   for (const auto & result : msg->results) {
  //     if (result.hypothesis.class_id == target_class_id_str) {
  //       // Convert Detection2D to a binary mono8 image
  //       cv::Mat image = cv::Mat::zeros(mask_height_, mask_width_, CV_8UC1);
  //       // Draws a rectangle filled with 255
  //       cv::rectangle(
  //         image,
  //         cv::Point(
  //           msg->bbox.center.position.x - msg->bbox.size_x / 2,
  //           msg->bbox.center.position.y - msg->bbox.size_y / 2),
  //         cv::Point(
  //           msg->bbox.center.position.x + msg->bbox.size_x / 2,
  //           msg->bbox.center.position.y + msg->bbox.size_y / 2),
  //         cv::Scalar(255), -1);

  //       // Convert the OpenCV image to a ROS sensor_msgs::msg::Image and publish it
  //       std_msgs::msg::Header header(msg->header);
  //       cv_bridge::CvImage cv_image(header, "mono8", image);
  //       sensor_msgs::msg::Image image_msg;
  //       cv_image.toImageMsg(image_msg);
  //       image_pub_->publish(image_msg);
  //       break;
  //     }
  //   } 
  // }

  void boundingBoxArrayCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    // 为每个对象类别创建一个最高置信度检测结果的映射
    std::map<std::string, std::pair<float, vision_msgs::msg::Detection2D>> best_detections;
    
    // 初始化每个对象的最高置信度为0
    for (const auto& object : object_list_) {
      best_detections[object] = std::make_pair(0.0f, vision_msgs::msg::Detection2D());
    }

    // 遍历所有检测结果
    for (const auto& detection : msg->detections) {
      for (const auto& result : detection.results) {
        // 将中文类别名转换为英文
        std::string eng_name = convertChineseToEnglish(result.hypothesis.class_id);
        if (!eng_name.empty() && 
            result.hypothesis.score > best_detections[eng_name].first) {
          best_detections[eng_name] = std::make_pair(result.hypothesis.score, detection);
        }
      }
    }

    // 为每个检测到的对象生成和发布掩码
    for (const auto& [object_name, detection_pair] : best_detections) {
      if (detection_pair.first > 0) {  // 如果找到了有效的检测结果
        // 创建二值掩码图像
        cv::Mat image = cv::Mat::zeros(mask_height_, mask_width_, CV_8UC1);
        
        // 绘制填充的矩形
        const auto& detection = detection_pair.second;
        cv::rectangle(
          image,
          cv::Point(
            detection.bbox.center.position.x - detection.bbox.size_x / 2,
            detection.bbox.center.position.y - detection.bbox.size_y / 2),
          cv::Point(
            detection.bbox.center.position.x + detection.bbox.size_x / 2,
            detection.bbox.center.position.y + detection.bbox.size_y / 2),
          cv::Scalar(255), -1);

        // 转换为ROS图像消息并发布
        std_msgs::msg::Header header(msg->header);
        cv_bridge::CvImage cv_image(header, "mono8", image);
        sensor_msgs::msg::Image image_msg;
        cv_image.toImageMsg(image_msg);
        
        // 发布到对应对象的话题
        if (image_pub_.find(object_name) != image_pub_.end()) {
          RCLCPP_INFO(this->get_logger(), "Publishing mask for object: %s", object_name.c_str());
          image_pub_[object_name]->publish(image_msg);
        }
      }
    }
  }

private:
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pub_;
  // rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr detection2_d_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection2_d_array_sub_;
  int mask_height_;
  int mask_width_;
  std::vector<std::string> object_list_;
  std::map<std::string, std::vector<std::string>> object_name_mapping_;

  // 添加一个辅助函数来转换中文到英文
  std::string convertChineseToEnglish(const std::string& chinese_name) {
    for (const auto& [eng_name, ch_names] : object_name_mapping_) {
      // 遍历每个对象的多个中文名
      for (const auto& ch_name : ch_names) {
        if (chinese_name.find(ch_name) != std::string::npos) {
          return eng_name;
        }
      }
    }
    return "";  // 如果没找到匹配的返回空字符串
  }
};

}  // namespace foundationpose
}  // namespace isaac_ros
}  // namespace nvidia

// Register the component with the ROS system to create a shared library
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::foundationpose::Detection2DToMask)
