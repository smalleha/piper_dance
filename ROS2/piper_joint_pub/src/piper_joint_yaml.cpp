#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <fstream>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class YamlJointPublisher : public rclcpp::Node
{
public:
    YamlJointPublisher(const std::string &yaml_path, const std::vector<std::string> &actions)
        : Node("yaml_joint_publisher"),
          action_index_(0),
          step_index_(0),
          elapsed_time_(0.0),
          finished_(false)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&YamlJointPublisher::update, this));

        joint_names_ = {
            "fl_joint1", "fl_joint2", "fl_joint3", "fl_joint4",
            "fl_joint5", "fl_joint6", "fl_joint7", "fl_joint8",
            "fr_joint1", "fr_joint2", "fr_joint3", "fr_joint4",
            "fr_joint5", "fr_joint6", "fr_joint7", "fr_joint8"};

        // 读取所有指定动作
        for (const auto &action_name : actions)
        {
            std::vector<Step> action_steps = load_yaml_action(yaml_path, action_name);
            if (!action_steps.empty())
            {
                all_actions_[action_name] = action_steps;
                action_order_.push_back(action_name);
                RCLCPP_INFO(this->get_logger(), "✅ Loaded action '%s' (%zu steps)", action_name.c_str(), action_steps.size());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "⚠️  Failed to load action '%s' from %s", action_name.c_str(), yaml_path.c_str());
            }
        }

        if (action_order_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ No valid actions loaded. Exiting...");
            rclcpp::shutdown();
        }
    }

private:
    struct Step
    {
        std::vector<double> positions;
        double duration;
    };

    std::vector<Step> load_yaml_action(const std::string &path, const std::string &action)
    {
        std::vector<Step> result;
        YAML::Node config = YAML::LoadFile(path);

        if (!config["actions"] || !config["actions"][action])
        {
            return result;
        }

        YAML::Node steps_node = config["actions"][action]["steps"];
        for (const auto &step_node : steps_node)
        {
            Step s;
            for (auto v : step_node["positions"])
                s.positions.push_back(v.as<double>());
            s.duration = step_node["duration"].as<double>();
            result.push_back(s);
        }
        return result;
    }

    void update()
    {
        if (finished_ || action_order_.empty())
            return;

        const std::string &current_action = action_order_[action_index_];
        const auto &steps = all_actions_[current_action];

        elapsed_time_ += 0.4; // 50ms timer

        const Step &step = steps[step_index_];
        if (elapsed_time_ >= step.duration)
        {
            elapsed_time_ = 0.0;
            step_index_++;

            // 当前动作完成
            if (step_index_ >= steps.size())
            {
                RCLCPP_INFO(this->get_logger(), "✅ Finished action '%s'", current_action.c_str());
                step_index_ = 0;
                action_index_++;

                // 所有动作完成，退出
                if (action_index_ >= action_order_.size())
                {
                    RCLCPP_INFO(this->get_logger(), "🎉 All actions completed. Exiting...");
                    finished_ = true;
                    rclcpp::sleep_for(200ms);
                    rclcpp::shutdown();
                    return;
                }
            }
        }

        // 发布当前关节状态
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;
        msg.position = step.positions;
        publisher_->publish(msg);
    }

    // ROS成员
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // YAML 动作存储
    std::map<std::string, std::vector<Step>> all_actions_;
    std::vector<std::string> action_order_;

    // 状态变量
    std::vector<std::string> joint_names_;
    size_t action_index_;
    size_t step_index_;
    double elapsed_time_;
    bool finished_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3)
    {
        std::cout << "Usage: ros2 run <pkg_name> yaml_joint_publisher <path_to_yaml> <action1> [action2] [action3] ...\n";
        return 1;
    }

    std::string yaml_path = argv[1];
    std::vector<std::string> actions;
    for (int i = 2; i < argc; ++i)
        actions.push_back(argv[i]);

    auto node = std::make_shared<YamlJointPublisher>(yaml_path, actions);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
