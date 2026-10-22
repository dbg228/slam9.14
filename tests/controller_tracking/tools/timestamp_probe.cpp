#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

uint32_t word(const uint8_t *p, bool little) {
  return little ? uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24)
                : uint32_t(p[3])|(uint32_t(p[2])<<8)|(uint32_t(p[1])<<16)|(uint32_t(p[0])<<24);
}
int main(int argc,char **argv) {
  if(argc<3) return 2;
  double duration=std::stod(argv[1]);std::string output=argv[2];
  rclcpp::init(argc,argv);auto n=std::make_shared<rclcpp::Node>("tracking_native_timestamp_probe");
  std::ofstream csv(output);csv<<"stream,receive_ns,header_ns,age_s\n"<<std::setprecision(12);
  std::map<std::string,std::vector<double>> ages;
  auto record=[&](const std::string &topic,int64_t stamp) {
    auto now=n->now().nanoseconds();double age=(now-stamp)*1e-9;
    ages[topic].push_back(age);csv<<topic<<","<<now<<","<<stamp<<","<<age<<"\n";
  };
  auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions;
  std::vector<std::pair<std::string,std::string>> topics={
    {"/livox/lidar","livox_ros_driver2/msg/CustomMsg"},{"/livox/imu","sensor_msgs/msg/Imu"},
    {"/Odometry","nav_msgs/msg/Odometry"},{"/odom","nav_msgs/msg/Odometry"},
    {"/mocap/Tracker0/pose","geometry_msgs/msg/PoseStamped"}};
  for(auto &p:topics) subscriptions.push_back(n->create_generic_subscription(p.first,p.second,qos,
    [&,topic=p.first](std::shared_ptr<rclcpp::SerializedMessage> message) {
      auto &b=message->get_rcl_serialized_message();if(b.buffer_length<12)return;
      bool little=(b.buffer[1]&1)!=0;
      int64_t stamp=int64_t(int32_t(word(b.buffer+4,little)))*1000000000LL+word(b.buffer+8,little);
      record(topic,stamp);
    }));
  auto tfcb=[&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for(auto &tf:msg->transforms) if((tf.header.frame_id=="odom" && tf.child_frame_id=="base_link") ||
      (tf.header.frame_id=="map" && tf.child_frame_id=="odom") ||
      (tf.header.frame_id=="camera_init" && tf.child_frame_id=="body"))
      record(tf.header.frame_id+"->"+tf.child_frame_id,int64_t(tf.header.stamp.sec)*1000000000LL+tf.header.stamp.nanosec);
  };
  auto a=n->create_subscription<tf2_msgs::msg::TFMessage>("/tf",qos,tfcb);
  auto b=n->create_subscription<tf2_msgs::msg::TFMessage>("/tf_lio_raw",qos,tfcb);
  auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(int(duration*1000));
  while(rclcpp::ok() && std::chrono::steady_clock::now()<deadline) {
    rclcpp::spin_some(n);std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  for(auto &kv:ages) {
    auto v=kv.second;std::sort(v.begin(),v.end());
    std::cout<<kv.first<<" n="<<v.size()<<" min="<<v.front()<<" median="<<v[v.size()/2]
      <<" p95="<<v[std::min(v.size()-1,size_t(v.size()*.95))]<<" max="<<v.back()<<std::endl;
  }
  rclcpp::shutdown();return 0;
}
