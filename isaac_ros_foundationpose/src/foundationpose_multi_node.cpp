#include "isaac_ros_foundationpose/foundationpose_multi_node.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace foundationpose
{

FoundationPoseMultiNode::FoundationPoseMultiNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("foundationpose_multi_node", options)
{
  // 配置每個 FoundationPoseNode 的參數
  // 這裡可以從參數服務器讀取配置，或是硬編碼示例
  node_configs_ = {
    { "path/to/mesh1.obj", "path/to/texture1.png", "config/foundationpose_model_config1.yaml" },
    { "path/to/mesh2.obj", "path/to/texture2.png", "config/foundationpose_model_config2.yaml" },
    // 添加更多的配置
  };

  initialize_nodes();
}

FoundationPoseMultiNode::~FoundationPoseMultiNode()
{
  for (auto & thread : node_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

void FoundationPoseMultiNode::initialize_nodes()
{
  for (const auto & config : node_configs_) {
    // 創建 FoundationPoseNode 的實例
    auto node = std::make_shared<FoundationPoseNode>(rclcpp::NodeOptions());

    // 設置特定的參數
    node->declare_parameter<std::string>("mesh_file_path", config.mesh_file_path);
    node->declare_parameter<std::string>("texture_path", config.texture_path);
    node->declare_parameter<std::string>("configuration_file", config.configuration_file);

    // 啟動節點的後台運行
    foundation_pose_nodes_.push_back(node);

    // 創建並啟動一個新線程來運行每個 FoundationPoseNode
    node_threads_.emplace_back([node]() {
      rclcpp::spin(node);
    });
  }
}

}  // namespace foundationpose
}  // namespace isaac_ros
}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::foundationpose::FoundationPoseMultiNode)