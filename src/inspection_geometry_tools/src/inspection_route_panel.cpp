#include <memory>
#include <string>

#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace inspection_geometry_tools
{
class InspectionRoutePanel : public rviz_common::Panel
{
public:
  explicit InspectionRoutePanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    auto * title = new QLabel("<b>巡检紫色路线</b>", this);
    layout->addWidget(title);
    hint_ = new QLabel("先用 RViz 的 Mark Route START / END 在紫线上各点击一次。", this);
    hint_->setWordWrap(true);
    layout->addWidget(hint_);
    status_ = new QLabel("等待路线节点…", this);
    status_->setWordWrap(true);
    status_->setMinimumWidth(290);
    layout->addWidget(status_);

    auto * save_row = new QHBoxLayout();
    save_button_ = new QPushButton("保存 START/END", this);
    load_button_ = new QPushButton("载入已保存", this);
    save_row->addWidget(save_button_);
    save_row->addWidget(load_button_);
    layout->addLayout(save_row);

    auto * action_row = new QHBoxLayout();
    follow_button_ = new QPushButton("开始路线", this);
    cancel_button_ = new QPushButton("取消", this);
    action_row->addWidget(follow_button_);
    action_row->addWidget(cancel_button_);
    layout->addLayout(action_row);

    connect(save_button_, &QPushButton::clicked, this, [this]() {call(save_client_, "保存请求已发送…");});
    connect(load_button_, &QPushButton::clicked, this, [this]() {call(load_client_, "载入请求已发送…");});
    connect(follow_button_, &QPushButton::clicked, this, [this]() {call(follow_client_, "启动请求已发送…");});
    connect(cancel_button_, &QPushButton::clicked, this, [this]() {call(cancel_client_, "取消请求已发送…");});
  }

  void onInitialize() override
  {
    const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abstraction) {
      status_->setText("无法取得 RViz ROS 节点。");
      return;
    }
    node_ = abstraction->get_raw_node();
    save_client_ = node_->create_client<std_srvs::srv::Trigger>("/inspection_route/save");
    load_client_ = node_->create_client<std_srvs::srv::Trigger>("/inspection_route/load");
    follow_client_ = node_->create_client<std_srvs::srv::Trigger>("/inspection_route/follow");
    cancel_client_ = node_->create_client<std_srvs::srv::Trigger>("/inspection_route/cancel");
    status_sub_ = node_->create_subscription<std_msgs::msg::String>("/inspection_route/status",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr message) {
          QMetaObject::invokeMethod(this, [this, message]() {
            status_->setText(QString::fromStdString(message->data));
          }, Qt::QueuedConnection);
        });
  }

private:
  void call(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, const char * pending_text)
  {
    if (!client || !client->service_is_ready()) {
      status_->setText("路线节点未启动：请先启动 inspection_geometry.launch.py。");
      return;
    }
    status_->setText(pending_text);
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        QMetaObject::invokeMethod(this, [this, response]() {
          status_->setText(QString::fromStdString((response->success ? "✓ " : "✗ ") + response->message));
        }, Qt::QueuedConnection);
      });
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr follow_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;
  QLabel * hint_{nullptr};
  QLabel * status_{nullptr};
  QPushButton * save_button_{nullptr};
  QPushButton * load_button_{nullptr};
  QPushButton * follow_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
};
}  // namespace inspection_geometry_tools

PLUGINLIB_EXPORT_CLASS(inspection_geometry_tools::InspectionRoutePanel, rviz_common::Panel)
