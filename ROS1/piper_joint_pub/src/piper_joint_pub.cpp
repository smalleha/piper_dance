#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>

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
        double delay = 0.0; // 每个机械臂启动延迟
    };

    JointSequencePublisher(ros::NodeHandle &nh)
        : nh_(nh), start_time_(ros::Time::now())
    {
        // ✅ 定义要加载的机械臂名称和对应YAML文件、延迟时间
        std::vector<std::string> arm_names = {"piper_1", "piper_2", "piper_3"};
        std::vector<std::string> yaml_paths = {
            "/home/q/ros1/dual_piper_ws/src/piper_joint_pub/config/piper_1.yaml",
            "/home/q/ros1/dual_piper_ws/src/piper_joint_pub/config/piper_2.yaml",
            "/home/q/ros1/dual_piper_ws/src/piper_joint_pub/config/piper_3.yaml"};
        std::vector<double> delays = {0.0, 3.0, 6.0}; // 每个机械臂延迟启动时间

        // ✅ 生成 joint 名称
        joint_names_ = {"joint1", "joint2", "joint3", "joint4",
                        "joint5", "joint6", "joint7", "joint8"};

        // ✅ 动态加载多个机械臂
        for (size_t i = 0; i < arm_names.size(); ++i)
        {
            ArmController arm;
            arm.name = arm_names[i];
            arm.pub = nh_.advertise<sensor_msgs::JointState>("/" + arm.name + "/joint_states", 10);
            arm.delay = delays[i];
            loadYAML(yaml_paths[i], arm.actions);
            arms_.push_back(arm);
            ROS_INFO("✅ Loaded arm [%s] with delay %.1fs, actions: %zu",
                     arm.name.c_str(), arm.delay, arm.actions.size());
        }

        ROS_INFO("🎯 Total arms loaded: %zu", arms_.size());
    }

    void spin()
    {
        ros::Rate rate(20); // 20Hz
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

            // 检查延迟
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
                ROS_INFO("✅ [%s] finished action [%s]", arm.name.c_str(), act.name.c_str());
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

    ros::NodeHandle nh_;
    std::vector<ArmController> arms_;
    std::vector<std::string> joint_names_;
    ros::Time start_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_joint_sequence_publisher");
    ros::NodeHandle nh;

    JointSequencePublisher node(nh);
    node.spin();
    return 0;
}
