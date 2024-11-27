#ifndef ISAAC_ROS_FOUNDATIONPOSE__FOUNDATIONPOSE_MULTI_NODE_HPP_
#define ISAAC_ROS_FOUNDATIONPOSE__FOUNDATIONPOSE_MULTI_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_foundationpose/foundationpose_node.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace foundationpose
{

/**
 * @class FoundationPoseMultiNode
 * @brief 管理多個 FoundationPoseNode 的節點，並行處理多個物體的 pose 估計
 */
class FoundationPoseMultiNode : public rclcpp::Node
{
public:
  explicit FoundationPoseMultiNode(const rclcpp::NodeOptions & options);
  ~FoundationPoseMultiNode();

private:
  struct NodeConfig
  {
    std::string mesh_file_path;
    std::string texture_path;
    std::string configuration_file;
  };

  void initialize_nodes();

  std::vector<NodeConfig> node_configs_;
  std::vector<std::shared_ptr<FoundationPoseNode>> foundation_pose_nodes_;
  std::vector<std::thread> node_threads_;
};

}  // namespace foundationpose
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_FOUNDATIONPOSE__FOUNDATIONPOSE_MULTI_NODE_HPP_
