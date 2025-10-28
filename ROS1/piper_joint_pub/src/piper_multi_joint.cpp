#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include <filesystem>

class JointSequencePublisher
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
        ros::Publisher pub;
        std::vector<Action> actions;
        size_t action_index = 0;
        size_t step_index = 0;
        double delay = 0.0;
    };

    JointSequencePublisher(ros::NodeHandle &nh)
        : nh_(nh), start_time_(ros::Time::now())
    {
        // ✅ 从参数服务器读取配置
        nh_.param<int>("num_arms", num_arms_, 3);
        nh_.param<std::string>("base_path", base_path_, "/home/q/ros1/dual_piper_ws/src/piper_joint_pub/config/");
        nh_.param<std::string>("arm_prefix", arm_prefix_, "piper_");
        nh_.param<double>("delay_step", delay_step_, 2.0);

        // ✅ Joint 名称（可根据实际情况调整）
        joint_names_ = {"joint1", "joint2", "joint3", "joint4",
                        "joint5", "joint6", "joint7", "joint8"};

        // ✅ 自动生成机械臂控制器
        for (int i = 1; i <= num_arms_; ++i)
        {
            std::ostringstream name_stream;
            name_stream << arm_prefix_ << i;
            std::string arm_name = name_stream.str();

            std::string yaml_file = base_path_ + arm_name + ".yaml";
            if (!std::filesystem::exists(yaml_file))
            {
                ROS_WARN("⚠️ YAML file not found for %s, skipped: %s", arm_name.c_str(), yaml_file.c_str());
                continue;
            }

            ArmController arm;
            arm.name = arm_name;
            arm.pub = nh_.advertise<sensor_msgs::JointState>("/" + arm_name + "/joint_states", 10);
            arm.delay = delay_step_ * (i - 1);

            loadYAML(yaml_file, arm.actions);
            arms_.push_back(arm);

            ROS_INFO("✅ Loaded [%s], delay=%.1fs, actions=%zu",
                     arm.name.c_str(), arm.delay, arm.actions.size());
        }

        ROS_INFO("🎯 Total arms loaded: %zu (configured: %d)", arms_.size(), num_arms_);
    }

    void spin()
    {
        ros::Rate rate(20);
        while (ros::ok())
        {
            update();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void loadYAML(const std::string &path, std::vector<Action> &actions)
    {
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(path);
        }
        catch (const YAML::BadFile &e)
        {
            ROS_ERROR("❌ Failed to open YAML file: %s", path.c_str());
            ros::shutdown();
            return;
        }

        if (!config["actions"])
        {
            ROS_ERROR("❌ No 'actions' field in YAML file: %s", path.c_str());
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
        ros::Time now = ros::Time::now();
        double elapsed = (now - start_time_).toSec();

        bool all_done = true;

        for (auto &arm : arms_)
        {
            if (arm.action_index >= arm.actions.size())
                continue;

            all_done = false;
            if (elapsed < arm.delay)
                continue;

            const auto &act = arm.actions[arm.action_index];
            sensor_msgs::JointState msg;
            msg.header.stamp = now;
            msg.name = joint_names_;
            msg.position = act.interpolated[arm.step_index];
            arm.pub.publish(msg);

            arm.step_index++;
            if (arm.step_index >= act.interpolated.size())
            {
                ROS_INFO("✅ [%s] finished [%s]", arm.name.c_str(), act.name.c_str());
                arm.step_index = 0;
                arm.action_index++;
            }
        }

        if (all_done)
        {
            ROS_INFO("🎉 All arms finished all actions, shutting down.");
            ros::shutdown();
        }
    }

    // === 成员变量 ===
    ros::NodeHandle nh_;
    ros::Time start_time_;
    std::vector<std::string> joint_names_;
    std::vector<ArmController> arms_;

    int num_arms_;
    std::string base_path_;
    std::string arm_prefix_;
    double delay_step_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_joint_sequence_publisher");
    ros::NodeHandle nh("~"); // 使用私有命名空间，方便 launch 参数加载

    JointSequencePublisher node(nh);
    node.spin();

    return 0;
}
