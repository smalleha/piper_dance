#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>

class JointSequencePublisher
{
public:
    JointSequencePublisher(ros::NodeHandle &nh)
        : nh_(nh)
    {
        pub_piper1_ = nh_.advertise<sensor_msgs::JointState>("/piper_1/joint_states", 10);
        pub_piper2_ = nh_.advertise<sensor_msgs::JointState>("/piper_2/joint_states", 10);

        // ✅ 固定路径（可改为参数加载）
        std::string yaml_path_1 = "/home/q/ros1/dual_piper_ws/src/piper_joint_pub/config/piper_1.yaml";
        // std::string yaml_path_2 = "/home/q/ros1/dual_piper_ws/src/piper_joint_pub/config/piper_2.yaml";

        loadYAML(yaml_path_1, actions_piper1_);
        // loadYAML(yaml_path_2, actions_piper2_);

        joint_names_ = {
            "joint1", "joint2", "joint3", "joint4",
            "joint5", "joint6", "joint7", "joint8"};

        step_index_1_ = 0;
        step_index_2_ = 0;
        action_index_1_ = 0;
        action_index_2_ = 0;

        ROS_INFO("✅ Node started, loaded %zu actions for Piper1 and %zu for Piper2",
                 actions_piper1_.size(), actions_piper2_.size());
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
    struct Action
    {
        std::string name;
        std::vector<double> start;
        std::vector<double> end;
        double step;
        std::vector<std::vector<double>> interpolated;
    };

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
            ROS_INFO("✅ Loaded action [%s] (%zu steps) from %s", a.name.c_str(), a.interpolated.size(), path.c_str());
        }
    }

    void update()
    {
        if (actions_piper1_.empty() && actions_piper2_.empty())
            return;

        ros::Time now = ros::Time::now();

        // ✅ Piper1
        if (action_index_1_ < actions_piper1_.size())
        {
            const auto &act1 = actions_piper1_[action_index_1_];
            sensor_msgs::JointState msg1;
            msg1.header.stamp = now;
            msg1.name = joint_names_;
            msg1.position = act1.interpolated[step_index_1_];
            pub_piper1_.publish(msg1);

            step_index_1_++;
            if (step_index_1_ >= act1.interpolated.size())
            {
                ROS_INFO("✅ Piper1 finished [%s]", act1.name.c_str());
                step_index_1_ = 0;
                action_index_1_++;
            }
        }

        // ✅ Piper2
        // if (action_index_2_ < actions_piper2_.size())
        // {
        //     const auto &act2 = actions_piper2_[action_index_2_];
        //     sensor_msgs::JointState msg2;
        //     msg2.header.stamp = now;
        //     msg2.name = joint_names_;
        //     msg2.position = act2.interpolated[step_index_2_];
        //     pub_piper2_.publish(msg2);

        //     step_index_2_++;
        //     if (step_index_2_ >= act2.interpolated.size())
        //     {
        //         ROS_INFO("✅ Piper2 finished [%s]", act2.name.c_str());
        //         step_index_2_ = 0;
        //         action_index_2_++;
        //     }
        // }

        // ✅ 若都完成动作，退出
        if (action_index_1_ >= actions_piper1_.size() &&
            action_index_2_ >= actions_piper2_.size())
        {
            ROS_INFO("🎉 Both Pipers finished all actions, shutting down.");
            ros::shutdown();
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_piper1_;
    ros::Publisher pub_piper2_;
    std::vector<std::string> joint_names_;

    std::vector<Action> actions_piper1_;
    std::vector<Action> actions_piper2_;

    size_t action_index_1_, step_index_1_;
    size_t action_index_2_, step_index_2_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_joint_sequence_publisher");
    ros::NodeHandle nh;

    JointSequencePublisher node(nh);
    node.spin();

    return 0;
}
