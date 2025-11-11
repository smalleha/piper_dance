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
        double hold_time;  // ✅ 新增字段
        std::vector<std::vector<double>> interpolated;
    };

    struct ArmController
    {
        std::string name;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
        std::vector<Action> actions;
        size_t action_index = 0;
        size_t step_index = 0;
        bool holding = false;
        rclcpp::Time hold_start_time;
        rclcpp::TimerBase::SharedPtr timer;  // ✅ 独立定时器
    };

    JointSequencePublisher()
        : Node("multi_joint_sequence_publisher")
    {
        // === 声明并读取参数 ===
        this->declare_parameter<int>("num_arms", 3);
        this->declare_parameter<std::string>("base_path", "/home/q/ros2_ws/src/piper_joint_pub/config/");
        this->declare_parameter<std::string>("arm_prefix", "piper_");

        this->get_parameter("num_arms", num_arms_);
        this->get_parameter("base_path", base_path_);
        this->get_parameter("arm_prefix", arm_prefix_);

        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

        // === 加载每个机械臂配置 ===
        for (int i = 1; i <= num_arms_; ++i)
        {
            std::ostringstream name_stream;
            name_stream << arm_prefix_ << i;
            std::string arm_name = name_stream.str();

            std::string yaml_file = base_path_ + arm_name + ".yaml";
            if (!std::filesystem::exists(yaml_file))
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ YAML not found for %s, skipped: %s", arm_name.c_str(), yaml_file.c_str());
                continue;
            }

            ArmController arm;
            arm.name = arm_name;
            arm.pub = this->create_publisher<sensor_msgs::msg::JointState>("/" + arm_name + "/joint_states", 10);

            loadYAML(yaml_file, arm.actions);

            // ✅ 每个机械臂创建独立定时器（20Hz）
            arm.timer = this->create_wall_timer(
                std::chrono::milliseconds(50),
                [this, i]() { this->update_arm(i - 1); });

            arms_.push_back(std::move(arm));
            RCLCPP_INFO(this->get_logger(), "✅ Loaded [%s] actions=%zu", arm_name.c_str(), arms_.back().actions.size());
        }

        RCLCPP_INFO(this->get_logger(), "🎯 Total arms loaded: %zu", arms_.size());
    }

private:
    // === 加载 YAML 动作 ===
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
            return;
        }

        if (!config["actions"])
        {
            RCLCPP_ERROR(this->get_logger(), "❌ No 'actions' in %s", path.c_str());
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
            a.hold_time = node["hold_time"] ? node["hold_time"].as<double>() : 0.0;

            // 计算插值
            size_t n = a.start.size();
            size_t steps = 0;
            for (size_t i = 0; i < n; ++i)
            {
                double diff = std::fabs(a.end[i] - a.start[i]);
                steps = std::max(steps, static_cast<size_t>(diff / std::max(a.step, 1e-6)));
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

    // === 每个机械臂独立 update 回调 ===
    void update_arm(int index)
    {
        if (index >= static_cast<int>(arms_.size()))
            return;

        auto &arm = arms_[index];
        auto now = this->now();

        if (arm.action_index >= arm.actions.size())
            return;

        auto &act = arm.actions[arm.action_index];

        // ✅ hold 阶段处理
        if (arm.holding)
        {
            double hold_elapsed = (now - arm.hold_start_time).seconds();
            if (hold_elapsed < act.hold_time)
                return;
            else
            {
                arm.holding = false;
                arm.action_index++;
                arm.step_index = 0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "[%s] Hold done, next action (%zu/%zu)",
                                     arm.name.c_str(), arm.action_index, arm.actions.size());
                return;
            }
        }

        // 发布当前步
        if (arm.step_index < act.interpolated.size())
        {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = now;
            msg.name = joint_names_;
            msg.position = act.interpolated[arm.step_index];
            arm.pub->publish(msg);
            arm.step_index++;
        }

        // ✅ 动作完成，进入 hold
        if (arm.step_index >= act.interpolated.size())
        {
            if (act.hold_time > 0.0)
            {
                arm.holding = true;
                arm.hold_start_time = now;
                RCLCPP_INFO(this->get_logger(), "✅ [%s] finished [%s], holding %.2fs",
                            arm.name.c_str(), act.name.c_str(), act.hold_time);
            }
            else
            {
                arm.action_index++;
                arm.step_index = 0;
            }

            // 所有动作完成后打印提示
            if (arm.action_index >= arm.actions.size())
            {
                RCLCPP_INFO(this->get_logger(), "🎉 [%s] all actions finished.", arm.name.c_str());
            }
        }
    }

    // === 成员变量 ===
    int num_arms_;
    std::string base_path_;
    std::string arm_prefix_;
    std::vector<std::string> joint_names_;
    std::vector<ArmController> arms_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointSequencePublisher>();

    // ✅ 多线程执行器：保证每个机械臂独立执行
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
