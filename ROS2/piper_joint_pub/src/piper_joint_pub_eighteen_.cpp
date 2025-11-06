#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <filesystem>
#include <mutex>

using namespace std::chrono_literals;

class MultiArmController : public rclcpp::Node
{
public:
    MultiArmController()
        : Node("multi_arm_controller")
    {
        this->declare_parameter<int>("arm_count", 18);
        this->declare_parameter<std::string>("yaml_dir", "/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config");
        this->declare_parameter<double>("publish_rate", 20.0);

        this->get_parameter("arm_count", arm_count_);
        this->get_parameter("yaml_dir", yaml_dir_);
        this->get_parameter("publish_rate", publish_rate_);

        RCLCPP_INFO(this->get_logger(), "🦾 Arm count: %d", arm_count_);
        RCLCPP_INFO(this->get_logger(), "📂 YAML directory: %s", yaml_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "⏱ Publish rate: %.2f Hz", publish_rate_);

        hand_publisher_ = this->create_publisher<std_msgs::msg::String>("/hand_cmd", 10);

        joint_names_ = {"joint1", "joint2", "joint3", "joint4","joint5", "joint6"};

        for (int i = 1; i <= arm_count_; ++i)
        {
            std::string topic = "/piper_" + std::to_string(i) + "/joint_states";
            auto pub = this->create_publisher<sensor_msgs::msg::JointState>(topic, 10);
            arm_publishers_.push_back(pub);
            std::string yaml_path = yaml_dir_ + "/piper_" + std::to_string(i) + ".yaml";

            if (!std::filesystem::exists(yaml_path))
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ YAML file not found for arm %d: %s", i, yaml_path.c_str());
                continue;
            }
            load_yaml(yaml_path, i);
        }

        RCLCPP_INFO(this->get_logger(), "✅ Loaded %zu arm configs", arm_actions_.size());
        execute_all_arms_parallel();
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
        std::string hand_cmd;
        std::vector<std::vector<double>> interpolated;
    };

    int arm_count_;
    std::string yaml_dir_;
    double publish_rate_;

    std::vector<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> arm_publishers_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hand_publisher_;
    std::vector<std::string> joint_names_;
    std::map<int, std::vector<Action>> arm_actions_;
    std::mutex log_mutex_; // 线程安全打印

    // === 加载 YAML ===
    void load_yaml(const std::string &path, int arm_id)
    {
        YAML::Node config = YAML::LoadFile(path);
        if (!config["actions"])
        {
            std::lock_guard<std::mutex> lk(log_mutex_);
            RCLCPP_ERROR(this->get_logger(), "❌ No 'actions' in %s", path.c_str());
            return;
        }

        std::vector<Action> actions;
        for (const auto &node : config["actions"])
        {
            Action a;
            a.name = node["name"].as<std::string>();
            a.enable = node["enable"] ? node["enable"].as<bool>() : true;
            a.step = node["step"].as<double>();
            a.hold_time = node["hold_time"] ? node["hold_time"].as<double>() : 0.0;
            a.hand_cmd = node["hand_cmd"] ? node["hand_cmd"].as<std::string>() : "";

            for (auto v : node["start"]) a.start.push_back(v.as<double>());
            for (auto v : node["end"]) a.end.push_back(v.as<double>());

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
                    double ratio = (steps == 0) ? 1.0 : static_cast<double>(s) / steps;
                    pos[i] = a.start[i] + (a.end[i] - a.start[i]) * ratio;
                }
                a.interpolated.push_back(pos);
            }

            actions.push_back(a);
        }
        arm_actions_[arm_id] = actions;
    }

    // === 并行执行所有机械臂 ===
    void execute_all_arms_parallel()
    {
        double sleep_ms = 1000.0 / publish_rate_;
        std::vector<std::thread> threads;

        for (const auto &[arm_id, actions] : arm_actions_)
        {
            threads.emplace_back([this, arm_id, actions, sleep_ms]() {
                sensor_msgs::msg::JointState msg;
                msg.name = joint_names_;
                msg.position.resize(joint_names_.size(), 0.0);

                for (const auto &a : actions)
                {
                    if (!a.enable)
                        continue;

                    {
                        std::lock_guard<std::mutex> lk(log_mutex_);
                        RCLCPP_INFO(this->get_logger(), "▶ Arm %d executing [%s]", arm_id, a.name.c_str());
                    }

                    if (!a.hand_cmd.empty())
                    {
                        std_msgs::msg::String cmd_msg;
                        cmd_msg.data = a.hand_cmd;
                        hand_publisher_->publish(cmd_msg);
                    }

                    for (const auto &pos : a.interpolated)
                    {
                        msg.header.stamp = this->now();
                        msg.position = pos;
                        arm_publishers_[arm_id - 1]->publish(msg);
                        std::this_thread::sleep_for(std::chrono::milliseconds((int)sleep_ms));
                    }

                    if (a.hold_time > 0)
                        std::this_thread::sleep_for(std::chrono::duration<double>(a.hold_time));
                }
            });
        }

        for (auto &t : threads)
            if (t.joinable()) t.join();

        RCLCPP_INFO(this->get_logger(), "🎉 All arms finished synchronously.");
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiArmController>();
    rclcpp::spin(node);
    return 0;
}
