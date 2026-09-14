#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inspection_geometry_tools
{
// The original node/executable name is preserved for launch-file and topic
// compatibility. P_click is deliberately used as the final feature point.
class FeatureSnapNode : public rclcpp::Node
{
public:
  FeatureSnapNode()
  : Node("feature_snap_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&FeatureSnapNode::clickCallback, this, std::placeholders::_1));
    feature_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/snapped_feature_point", 10);
  }

private:
  void clickCallback(const geometry_msgs::msg::PointStamped::SharedPtr click) const
  {
    if (!click->header.frame_id.empty() && click->header.frame_id != frame_id_) {
      RCLCPP_WARN(get_logger(), "Ignoring click in frame '%s'; expected '%s'.",
        click->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    geometry_msgs::msg::PointStamped feature = *click;
    feature.header.frame_id = frame_id_;
    feature.header.stamp = now();
    feature_pub_->publish(feature);
  }

  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr feature_pub_;
};
}  // namespace inspection_geometry_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_geometry_tools::FeatureSnapNode>());
  rclcpp::shutdown();
  return 0;
}
