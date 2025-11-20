#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include <filesystem>
#include <mutex>
#include <std_msgs/msg/string.hpp>

class JointSequencePublisher : public rclcpp::Node
{
public:
    struct Action
    {
        std::string name;
        std::vector<double> start;
        std::vector<double> end;
        double step;
        double hold_time;
        double delay_time;
        bool together;
        std::string hand_cmd;

        std::vector<std::vector<double>> interpolated;
    };

    struct ArmController
    {
        std::string name;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hand_publisher_;
        std::vector<Action> actions;
        size_t action_index = 0;
        size_t step_index = 0;
        bool holding = false;
        bool waiting_sync = false;
        bool delaying = false;
        rclcpp::Time delay_start_time;
        rclcpp::Time hold_start_time;
        rclcpp::TimerBase::SharedPtr timer;
    };

    JointSequencePublisher()
        : Node("multi_joint_sequence_publisher"), sync_active_(false)
    {
        this->declare_parameter<int>("num_arms", 3);
        this->declare_parameter<std::string>("base_path", "/home/q/ros2_ws/src/piper_joint_pub/config/");
        this->declare_parameter<std::string>("arm_prefix", "piper_");

        this->get_parameter("num_arms", num_arms_);
        this->get_parameter("base_path", base_path_);
        this->get_parameter("arm_prefix", arm_prefix_);

        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

        for (int i = 1; i <= num_arms_; ++i)
        {
            std::ostringstream name_stream;
            name_stream << arm_prefix_ << i;
            std::string arm_name = name_stream.str();

            std::string yaml_file = base_path_ + arm_name + ".yaml";
            if (!std::filesystem::exists(yaml_file))
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ YAML not found for %s, skipped: %s",
                            arm_name.c_str(), yaml_file.c_str());
                continue;
            }

            ArmController arm;
            arm.name = arm_name;
            arm.pub = this->create_publisher<sensor_msgs::msg::JointState>("/" + arm_name + "/joint_states", 10);
            arm.hand_publisher_ = this->create_publisher<std_msgs::msg::String>("/" + arm_name + "/hand_cmd", 10);

            loadYAML(yaml_file, arm.actions);

            arm.timer = this->create_wall_timer(
                std::chrono::milliseconds(20),
                [this, i]() { this->update_arm(i - 1); });

            arms_.push_back(std::move(arm));
            RCLCPP_INFO(this->get_logger(), "✅ Loaded [%s] actions=%zu",
                        arm_name.c_str(), arms_.back().actions.size());
        }

        RCLCPP_INFO(this->get_logger(), "🎯 Total arms loaded: %zu", arms_.size());
    }

private:
    std::mutex sync_mutex_;
    bool sync_active_;

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
            for (auto v : node["start"]) a.start.push_back(v.as<double>());
            for (auto v : node["end"]) a.end.push_back(v.as<double>());
            a.step = node["step"].as<double>();
            a.hold_time = node["hold_time"] ? node["hold_time"].as<double>() : 0.0;
            a.delay_time = node["delay_time"] ? node["delay_time"].as<double>() : 0.0;
            a.together = node["together"] ? node["together"].as<bool>() : false;
            a.hand_cmd = node["hand_cmd"] ? node["hand_cmd"].as<std::string>() : "";

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

            // ====== 插补结束后过滤 NaN ======
            std::vector<std::vector<double>> filtered;
            filtered.reserve(a.interpolated.size());

            for (auto &vec : a.interpolated)
            {
                bool has_nan = false;
                for (double v : vec)
                {
                    if (std::isnan(v))
                    {
                        has_nan = true;
                        break;
                    }
                }

                if (!has_nan)
                    filtered.push_back(vec);
                else
                    RCLCPP_WARN(this->get_logger(),
                                "⚠️ [%s] 插补中出现 NaN，已过滤该关键帧", a.name.c_str());
            }

            if (filtered.empty())
            {
                RCLCPP_ERROR(this->get_logger(),
                             "❌ [%s] 插补之后全部为 NaN，动作将被跳过！", a.name.c_str());
                continue; // 不加入动作
            }

            a.interpolated = filtered;
            actions.push_back(a);
        }
    }

    void update_arm(int index)
    {
        if (index >= static_cast<int>(arms_.size()))
            return;

        auto &arm = arms_[index];
        auto now = this->now();

        if (arm.action_index >= arm.actions.size())
            return;

        auto &act = arm.actions[arm.action_index];

        // --------------------- 同步机制 together ---------------------
        if (act.together)
        {
            std::lock_guard<std::mutex> lock(sync_mutex_);
            if (!sync_active_ && !arm.waiting_sync)
            {
                arm.waiting_sync = true;

                bool all_ready = true;
                for (auto &a : arms_)
                {
                    if (a.action_index >= a.actions.size())
                        continue;
                    if (!a.actions[a.action_index].together)
                        all_ready = false;
                    if (!a.waiting_sync)
                        all_ready = false;
                }

                if (all_ready)
                {
                    sync_active_ = true;
                    for (auto &a : arms_)
                        a.waiting_sync = false;

                    RCLCPP_INFO(this->get_logger(), "🚀 Together mode start!");
                }
                return;
            }

            if (!sync_active_)
                return;
        }

        // --------------------- 延迟 delay_time ---------------------
        if (act.delay_time > 0.0 && !arm.delaying && arm.step_index == 0)
        {
            arm.delaying = true;
            arm.delay_start_time = now;
            RCLCPP_INFO(this->get_logger(), "[%s] delaying %.2fs before [%s]",
                        arm.name.c_str(), act.delay_time, act.name.c_str());
            return;
        }

        if (arm.delaying)
        {
            if ((now - arm.delay_start_time).seconds() < act.delay_time)
                return;
            arm.delaying = false;
        }

        // --------------------- 执行动作插补 ---------------------
        if (arm.step_index < act.interpolated.size())
        {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = now;
            msg.name = joint_names_;
            msg.position = act.interpolated[arm.step_index];

            arm.pub->publish(msg);
            arm.step_index++;
        }
        else
        {
            // --------------------- 发布 hand_cmd ---------------------
            if (!act.hand_cmd.empty())
            {
                std_msgs::msg::String hand_msg;
                hand_msg.data = act.hand_cmd;
                arm.hand_publisher_->publish(hand_msg);
                RCLCPP_INFO(this->get_logger(), "[%s] Published hand_cmd: %s",
                            arm.name.c_str(), act.hand_cmd.c_str());
            }

            // --------------------- hold_time ---------------------
            if (!act.together && act.hold_time > 0.0)
            {
                if (!arm.holding)
                {
                    arm.holding = true;
                    arm.hold_start_time = now;
                    RCLCPP_INFO(this->get_logger(), "[%s] finished [%s], holding %.2fs",
                                arm.name.c_str(), act.name.c_str(), act.hold_time);
                }
                else if ((now - arm.hold_start_time).seconds() >= act.hold_time)
                {
                    arm.holding = false;
                    arm.action_index++;
                    arm.step_index = 0;
                }
            }
            else
            {
                arm.action_index++;
                arm.step_index = 0;
            }

            // --------------------- together 动作结束判断 ---------------------
            if (act.together)
            {
                std::lock_guard<std::mutex> lock(sync_mutex_);
                bool all_done = true;

                for (auto &a : arms_)
                {
                    if (a.action_index < a.actions.size() &&
                        a.actions[a.action_index].together)
                        all_done = false;
                }

                if (all_done)
                {
                    sync_active_ = false;
                    RCLCPP_INFO(this->get_logger(), "🎯 Together mode finished.");
                }
            }

            if (arm.action_index >= arm.actions.size())
                RCLCPP_INFO(this->get_logger(), "🎉 [%s] all actions finished.", arm.name.c_str());
        }
    }

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

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
