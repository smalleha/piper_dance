#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include <filesystem>

class JointSequencePublisher : public rclcpp::Node
{
public:
    struct Action
    {
        std::string name;
        std::vector<double> start;
        std::vector<double> end;
        double step;
        std::vector<std::vector<double>> interpolated;
    };

    struct ArmController
    {
        std::string name;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
        std::vector<Action> actions;
        size_t action_index = 0;
        size_t step_index = 0;
        double delay = 0.0;
    };

    JointSequencePublisher()
        : Node("multi_joint_sequence_publisher"),
          start_time_(this->now())
    {
        // 声明参数
        this->declare_parameter<int>("num_arms", 3);
        this->declare_parameter<std::string>("base_path", "/home/q/ros2_ws/src/piper_joint_pub/config/");
        this->declare_parameter<std::string>("arm_prefix", "piper_");
        this->declare_parameter<double>("delay_step", 2.0);

        // 获取参数
        this->get_parameter("num_arms", num_arms_);
        this->get_parameter("base_path", base_path_);
        this->get_parameter("arm_prefix", arm_prefix_);
        this->get_parameter("delay_step", delay_step_);

        joint_names_ = {"joint1", "joint2", "joint3", "joint4",
                        "joint5", "joint6", "joint7", "joint8"};

        // 加载每个机械臂的配置
        for (int i = 1; i <= num_arms_; ++i)
        {
            std::ostringstream name_stream;
            name_stream << arm_prefix_ << i;
            std::string arm_name = name_stream.str();

            std::string yaml_file = base_path_ + arm_name + ".yaml";
            if (!std::filesystem::exists(yaml_file))
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ YAML file not found for %s, skipped: %s",
                            arm_name.c_str(), yaml_file.c_str());
                continue;
            }

            ArmController arm;
            arm.name = arm_name;
            arm.pub = this->create_publisher<sensor_msgs::msg::JointState>("/" + arm_name + "/joint_states", 10);
            arm.delay = delay_step_ * (i - 1);

            loadYAML(yaml_file, arm.actions);
            arms_.push_back(arm);

            RCLCPP_INFO(this->get_logger(), "✅ Loaded [%s], delay=%.1fs, actions=%zu",
                        arm.name.c_str(), arm.delay, arm.actions.size());
        }

        RCLCPP_INFO(this->get_logger(), "🎯 Total arms loaded: %zu (configured: %d)",
                    arms_.size(), num_arms_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JointSequencePublisher::update, this));
    }

private:
    // === 状态变量 ===
    bool all_paused_ = false;
    bool all_resumed_ = false;

    void loadYAML(const std::string &path, std::vector<Action> &actions)
    {
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(path);
        }
        catch (const YAML::BadFile &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open YAML file: %s", path.c_str());
            rclcpp::shutdown();
            return;
        }

        if (!config["actions"])
        {
            RCLCPP_ERROR(this->get_logger(), "❌ No 'actions' field in YAML file: %s", path.c_str());
            return;
        }

        for (const auto &node : config["actions"])
        {
            Action a;
            a.name = node["name"].as<std::string>();
            for (auto v : node["start"])
                a.start.push_back(v.as<double>());
            for (auto v : node["end"])
                a.end.push_back(v.as<double>());
            a.step = node["step"].as<double>();

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

            actions.push_back(a);
        }
    }

    void update()
    {
        auto now = this->now();
        double elapsed = (now - start_time_).seconds();

        if (all_paused_ && !all_resumed_)
            return;

        bool all_done = true;

        for (auto &arm : arms_)
        {
            if (arm.action_index >= arm.actions.size())
                continue;

            all_done = false;
            if (elapsed < arm.delay)
                continue;

            const auto &act = arm.actions[arm.action_index];
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = now;
            msg.name = joint_names_;
            msg.position = act.interpolated[arm.step_index];
            arm.pub->publish(msg);

            arm.step_index++;
            if (arm.step_index >= act.interpolated.size())
            {
                RCLCPP_INFO(this->get_logger(), "✅ [%s] finished [%s]", arm.name.c_str(), act.name.c_str());

                // 暂停逻辑
                if (arm.name == "piper_1" && act.name == "final_rest")
                {
                    all_paused_ = true;
                    RCLCPP_WARN(this->get_logger(), "⏸ All paused after piper_1 finished final_rest");
                }

                // 恢复逻辑
                if (arm.name == "piper_3" && act.name == "final_rest")
                {
                    all_resumed_ = true;
                    all_paused_ = false;
                    start_time_ = this->now();

                    for (auto &a : arms_)
                    {
                        a.action_index = 0;
                        a.step_index = 0;
                    }
                    RCLCPP_WARN(this->get_logger(), "🔁 All resumed after piper_3 finished final_rest");
                }

                arm.step_index = 0;
                arm.action_index++;
            }
        }

        if (all_done && !all_paused_)
        {
            RCLCPP_INFO(this->get_logger(), "🎉 All arms finished all actions, shutting down.");
            rclcpp::shutdown();
        }
    }

    // === 成员变量 ===
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    std::vector<std::string> joint_names_;
    std::vector<ArmController> arms_;

    int num_arms_;
    std::string base_path_;
    std::string arm_prefix_;
    double delay_step_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointSequencePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
