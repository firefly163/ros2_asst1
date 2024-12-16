#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "turtlesim/msg/pose.hpp"

class MultiTurtleTFBroadcaster : public rclcpp::Node
{
private:
    tf2_ros::TransformBroadcaster tf_broadcaster_; // tf2 广播器
    std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> subscriptions_; // 订阅器列表
    std::vector<std::string> turtle_names_; // 海龟名称列表

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg, const std::string &turtle_name)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";              // 父坐标系为 "world"
        t.child_frame_id = turtle_name;           // 子坐标系为海龟的名字
        t.transform.translation.x = msg->x;       // 设置 x 坐标
        t.transform.translation.y = msg->y;       // 设置 y 坐标
        t.transform.translation.z = 0.0;          // z 坐标为 0

        // 转换欧拉角 (Roll, Pitch, Yaw) 为四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_.sendTransform(t);
    }

public:
    MultiTurtleTFBroadcaster(const std::vector<std::string> &turtle_names)
        : Node("multi_turtle_tf_broadcaster"), tf_broadcaster_(this), turtle_names_(turtle_names)
    {
        // 为每只海龟创建一个订阅器
        for (const auto &turtle_name : turtle_names_)
        {
            auto subscription = this->create_subscription<turtlesim::msg::Pose>(
                turtle_name + "/pose", 10,
                [this, turtle_name](const turtlesim::msg::Pose::SharedPtr msg)
                {
                    this->pose_callback(msg, turtle_name);
                });

            subscriptions_.push_back(subscription); // 保存订阅器
        }

        RCLCPP_INFO(this->get_logger(), "Multi Turtle TF Broadcaster started for turtles: %s",
                    this->get_turtle_names().c_str());
    }

    ~MultiTurtleTFBroadcaster()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, Multi Turtle TF Broadcaster");
    }

    std::string get_turtle_names()
    {
        std::string names;
        for (const auto &name : turtle_names_)
        {
            names += name + " ";
        }
        return names;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // 从命令行获取海龟名称列表
    if (argc < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: tf <turtle_name_1> <turtle_name_2> ...");
        return 1;
    }
    // 构造海龟名称列表
    std::vector<std::string> turtle_names;
    for (int i = 1; i < argc; ++i)
    {
        turtle_names.push_back(argv[i]);
    }
    // 创建并运行多海龟 TF 广播节点
    rclcpp::spin(std::make_shared<MultiTurtleTFBroadcaster>(turtle_names));

    rclcpp::shutdown();
    return 0;
}
