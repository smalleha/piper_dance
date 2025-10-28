#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class JointSequencePublisher : public rclcpp::Node
{
public:
    JointSequencePublisher()
        : Node("joint_sequence_publisher"),
          action_index_(0),
          step_index_(0),
          holding_(false)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // ✅ 固定 YAML 文件路径
        std::string yaml_path = "/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/joint_.yaml";
        load_yaml(yaml_path);

        joint_names_ = {
            "fl_joint1", "fl_joint2", "fl_joint3", "fl_joint4",
            "fl_joint5", "fl_joint6", "fl_joint7", "fl_joint8",
            "fr_joint1", "fr_joint2", "fr_joint3", "fr_joint4",
            "fr_joint5", "fr_joint6", "fr_joint7", "fr_joint8"};

        timer_ = this->create_wall_timer(50ms, std::bind(&JointSequencePublisher::update, this));
        RCLCPP_INFO(this->get_logger(), "✅ Node started, loaded %zu actions from %s", actions_.size(), yaml_path.c_str());
    }

private:
    struct Action
    {
        std::string name;
        bool enable;
        std::vector<double> start;
        std::vector<double> end;
        double step;
        double hold_time; // ✅ 新增字段：动作结束后保持时间（秒）
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
            a.hold_time = node["hold_time"] ? node["hold_time"].as<double>() : 0.0; // ✅ 默认0秒

            for (auto v : node["start"])
                a.start.push_back(v.as<double>());
            for (auto v : node["end"])
                a.end.push_back(v.as<double>());

            if (!a.enable)
            {
                a.interpolated.push_back(a.start);
                actions_.push_back(a);
                RCLCPP_WARN(this->get_logger(), "⚠️ Skipping disabled action [%s]", a.name.c_str());
                continue;
            }

            // ✅ 生成插值序列
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
                        "✅ Loaded action [%s] (%s) with %zu steps, hold_time=%.2f",
                        a.name.c_str(), a.enable ? "enabled" : "disabled", a.interpolated.size(), a.hold_time);
        }
    }

    void update()
    {
        if (actions_.empty())
            return;

        auto &current_action = actions_[action_index_];
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        // ✅ 如果正在保持当前姿态
        if (holding_)
        {
            msg.position = current_action.end;
            publisher_->publish(msg);

            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - hold_start_time_).count() >= current_action.hold_time)
            {
                holding_ = false;
                step_index_ = 0;
                action_index_++;
                if (action_index_ >= actions_.size())
                {
                    RCLCPP_INFO(this->get_logger(), "🎉 All actions completed, shutting down.");
                    rclcpp::shutdown();
                }
            }
            return;
        }

        // ✅ 正常执行插值动作
        msg.position = current_action.interpolated[step_index_];
        publisher_->publish(msg);
        step_index_++;

        if (step_index_ >= current_action.interpolated.size())
        {
            RCLCPP_INFO(this->get_logger(), "✅ Finished action [%s], holding for %.1f sec", current_action.name.c_str(), current_action.hold_time);
            holding_ = true;
            hold_start_time_ = std::chrono::steady_clock::now();
        }
    }

    // === 成员变量 ===
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
    std::vector<Action> actions_;
    size_t action_index_;
    size_t step_index_;
    bool holding_;
    std::chrono::steady_clock::time_point hold_start_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointSequencePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
