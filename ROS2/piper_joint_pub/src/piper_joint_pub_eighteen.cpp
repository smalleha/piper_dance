#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class JointSequencePublisher : public rclcpp::Node
{
public:
    JointSequencePublisher()
        : Node("joint_sequence_publisher_once")
    {
        // 发布 joint_states
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        // 发布 hand_cmd
        hand_publisher_ = this->create_publisher<std_msgs::msg::String>("/hand_cmd", 10);

        std::string yaml_path = "/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/dance_pose_v1.yaml";
        load_yaml(yaml_path);


        // === 初始化 18个机械臂的108个关节 ===
        for (int i = 1; i <= 18; ++i)
        {
            for (int j = 1; j <= 6; ++j)
            {
                joint_names_.push_back("piper_" + std::to_string(i) + "/joint" + std::to_string(j));
            }
        }

        RCLCPP_INFO(this->get_logger(), "✅ Node started, loaded %zu actions from %s", actions_.size(), yaml_path.c_str());

        execute_actions_once();
    }

private:
    struct Action
    {
        std::string name;
        bool enable;
        std::vector<double> start;
        std::vector<double> end;
        double step;
        double hold_time;
        std::string hand_cmd; // ✅ 新增字段
        std::vector<std::vector<double>> interpolated;
    };

    void load_yaml(const std::string &path)
    {
        YAML::Node config = YAML::LoadFile(path);
        if (!config["actions"])
        {
            RCLCPP_ERROR(this->get_logger(), "❌ No 'actions' field in YAML file: %s", path.c_str());
            return;
        }

        for (const auto &node : config["actions"])
        {
            Action a;
            a.name = node["name"].as<std::string>();
            a.enable = node["enable"] ? node["enable"].as<bool>() : true;
            a.step = node["step"].as<double>();
            a.hold_time = node["hold_time"] ? node["hold_time"].as<double>() : 0.0;

            // ✅ 新增：读取 hand_cmd 参数（可选）
            if (node["hand_cmd"])
                a.hand_cmd = node["hand_cmd"].as<std::string>();
            else
                a.hand_cmd = "";

            for (auto v : node["start"])
                a.start.push_back(v.as<double>());
            for (auto v : node["end"])
                a.end.push_back(v.as<double>());

            if (!a.enable)
            {
                actions_.push_back(a);
                RCLCPP_WARN(this->get_logger(), "⚠️ Skipping disabled action [%s]", a.name.c_str());
                continue;
            }

            // === 插值计算 ===
            size_t n = a.start.size();
            size_t steps = 0;
            for (size_t i = 0; i < n; ++i)
            {
                double diff = std::fabs(a.end[i] - a.start[i]);
                steps = std::max(steps, static_cast<size_t>(diff / a.step));
            }

            for (size_t s = 0; s <= steps; ++s)
            {
                std::vector<double> pos(n);
                for (size_t i = 0; i < n; ++i)
                {
                    double ratio = static_cast<double>(s) / steps;
                    pos[i] = a.start[i] + (a.end[i] - a.start[i]) * ratio;
                }
                a.interpolated.push_back(pos);
            }

            actions_.push_back(a);
            RCLCPP_INFO(this->get_logger(),
                        "✅ Loaded action [%s] (%s) with %zu steps, hold_time=%.2f, hand_cmd=%s",
                        a.name.c_str(), a.enable ? "enabled" : "disabled", a.interpolated.size(),
                        a.hold_time, a.hand_cmd.empty() ? "none" : a.hand_cmd.c_str());
        }
    }

    void execute_actions_once()
    {
        sensor_msgs::msg::JointState msg;
        msg.name = joint_names_;
        msg.position.resize(joint_names_.size(), 0.0);

        for (size_t idx = 0; idx < actions_.size(); ++idx)
        {
            auto &a = actions_[idx];
            if (!a.enable)
                continue;

            RCLCPP_INFO(this->get_logger(), "▶ Executing action [%s]...", a.name.c_str());

            // ✅ 若存在 hand_cmd，则发送
            if (!a.hand_cmd.empty())
            {
                std_msgs::msg::String cmd_msg;
                cmd_msg.data = a.hand_cmd;
                hand_publisher_->publish(cmd_msg);
                RCLCPP_INFO(this->get_logger(), "🖐 Sent hand_cmd: %s", a.hand_cmd.c_str());
            }

            // === 播放插值动作 ===
            for (const auto &pos : a.interpolated)
            {
                msg.header.stamp = this->now();
                msg.position = pos;
                publisher_->publish(msg);
                std::this_thread::sleep_for(50ms);
            }

            if (a.hold_time > 0)
            {
                RCLCPP_INFO(this->get_logger(), "⏸ Holding final pose for %.1f sec", a.hold_time);
                std::this_thread::sleep_for(std::chrono::duration<double>(a.hold_time));
            }
        }

        RCLCPP_INFO(this->get_logger(), "🎉 All actions completed. Exiting...");
        rclcpp::shutdown();
    }

    // === 成员变量 ===
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hand_publisher_; // ✅ 新增 hand_cmd 发布器
    std::vector<std::string> joint_names_;
    std::vector<Action> actions_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointSequencePublisher>();
    rclcpp::spin(node);
    return 0;
}
