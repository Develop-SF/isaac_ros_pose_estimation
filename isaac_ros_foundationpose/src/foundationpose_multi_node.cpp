// #include <memory>
// #include <vector>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "isaac_ros_foundationpose/foundationpose_node.hpp"
// #include "detection3_d_array_message/detection3_d_array_message.hpp"

// namespace nvidia
// {
// namespace isaac_ros
// {
// namespace foundationpose
// {

// class MultiFoundationPoseNode : public rclcpp::Node
// {
// public:
//   MultiFoundationPoseNode(const rclcpp::NodeOptions & options)
//   : Node("multi_foundationpose_node", options)
//   {
//     // 声明参数
//     this->declare_parameter<std::vector<std::string>>("mesh_files", std::vector<std::string>());
//     this->declare_parameter<std::string>("output_topic", "multi_pose_estimation/output");

//     // 获取参数
//     std::vector<std::string> mesh_files = 
//       this->get_parameter("mesh_files").as_string_array();
//     std::string output_topic = 
//       this->get_parameter("output_topic").as_string();

//     // 为每个mesh文件创建一个FoundationPoseNode
//     for (const auto & mesh_file : mesh_files) {
//       rclcpp::NodeOptions node_options;
//       node_options.arguments(
//         {"--ros-args",
//          "-p", "mesh_file_path:=" + mesh_file,
//          "-p", "output_topic:=" + output_topic + "_" + std::to_string(foundationpose_nodes_.size())
//         });
      
//       auto node = std::make_shared<FoundationPoseNode>(node_options);
//       foundationpose_nodes_.push_back(node);
//     }

//     // 创建合并结果的发布者
//     merged_publisher_ = this->create_publisher<Detection3DArrayMsg>(
//       output_topic, 10);

//     // 创建定时器,定期合并和发布结果
//     merge_timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(100),
//       std::bind(&MultiFoundationPoseNode::mergeAndPublishResults, this));
//   }

// private:
//   void mergeAndPublishResults()
//   {
//     Detection3DArrayMsg merged_msg;

//     for (const auto & node : foundationpose_nodes_) {
//       // 获取每个节点的最新检测结果
//       auto latest_msg = node->getLatestDetection();
//       if (latest_msg) {
//         // 合并检测结果
//         merged_msg.detections.insert(
//           merged_msg.detections.end(),
//           latest_msg->detections.begin(),
//           latest_msg->detections.end());
//       }
//     }

//     // 发布合并后的结果
//     if (!merged_msg.detections.empty()) {
//       merged_publisher_->publish(merged_msg);
//     }
//   }

//   std::vector<std::shared_ptr<FoundationPoseNode>> foundationpose_nodes_;
//   rclcpp::Publisher<Detection3DArrayMsg>::SharedPtr merged_publisher_;
//   rclcpp::TimerBase::SharedPtr merge_timer_;
// };

// }  // namespace foundationpose
// }  // namespace isaac_ros
// }  // namespace nvidia

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::foundationpose::MultiFoundationPoseNode)