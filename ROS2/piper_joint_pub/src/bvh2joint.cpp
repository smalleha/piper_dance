#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <unordered_map>
#include <filesystem>

//////////////////////////////////////////////////////////
//                    BVH 数据结构
//////////////////////////////////////////////////////////
struct BVHNode
{
    std::string name;
    std::vector<double> offset;
    std::vector<std::string> channels;
    std::vector<std::shared_ptr<BVHNode>> children;
};

//////////////////////////////////////////////////////////
//                    BVH 解析器
//////////////////////////////////////////////////////////
class BVHParser
{
public:
    BVHParser(const std::string &file_path)
        : file_path_(file_path)
    {}

    void parse()
    {
        std::ifstream file(file_path_);
        if (!file.is_open())
            throw std::runtime_error("无法打开 BVH 文件: " + file_path_);

        std::string line;
        while (std::getline(file, line))
        {
            lines_.push_back(trim(line));
        }

        size_t idx = 0;
        if (lines_[idx] != "HIERARCHY")
            throw std::runtime_error("BVH 格式错误: 缺少 HIERARCHY");
        idx++;

        root_ = parseJoint(idx);

        parseMotion(idx);
    }

    const std::vector<std::vector<double>>& getFrames() const { return frames_; }
    const std::vector<std::string>& getChannelOrder() const { return channel_order_; }
    double getFrameTime() const { return frame_time_; }

private:
    std::string file_path_;
    std::vector<std::string> lines_;
    std::shared_ptr<BVHNode> root_;
    std::vector<std::vector<double>> frames_;
    std::vector<std::string> channel_order_;
    double frame_time_;

    static std::string trim(const std::string &s)
    {
        size_t start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    std::shared_ptr<BVHNode> parseJoint(size_t &idx)
    {
        auto node = std::make_shared<BVHNode>();
        std::string line = lines_[idx];

        if (line.rfind("End", 0) == 0)
        {
            node->name = "EndSite";
        }
        else
        {
            std::stringstream ss(line);
            std::string type;
            ss >> type >> node->name;
        }

        idx++;
        if (lines_[idx] != "{")
            throw std::runtime_error("BVH 格式错误: 缺少 {");
        idx++;

        while (true)
        {
            line = lines_[idx];

            if (line.rfind("OFFSET", 0) == 0)
            {
                std::stringstream ss(line);
                std::string temp;
                double x, y, z;
                ss >> temp >> x >> y >> z;
                node->offset = {x, y, z};
            }
            else if (line.rfind("CHANNELS", 0) == 0)
            {
                std::stringstream ss(line);
                std::string temp;
                int num;
                ss >> temp >> num;
                node->channels.resize(num);

                for (int i = 0; i < num; i++)
                {
                    ss >> node->channels[i];
                    channel_order_.push_back(node->name + "_" + node->channels[i]);
                }
            }
            else if (line.rfind("JOINT", 0) == 0 || line.rfind("End", 0) == 0)
            {
                node->children.push_back(parseJoint(idx));
                continue;
            }
            else if (line == "}")
            {
                idx++;
                return node;
            }

            idx++;
        }
    }

    void parseMotion(size_t &idx)
    {
        if (lines_[idx] != "MOTION")
            throw std::runtime_error("BVH 文件缺少 MOTION");

        idx++;

        // Frames: N
        {
            std::stringstream ss(lines_[idx]);
            std::string temp;
            int n;
            ss >> temp >> n;
            frames_.reserve(n);
        }
        idx++;

        // Frame Time
        {
            std::stringstream ss(lines_[idx]);
            std::string temp;
            ss >> temp >> temp >> frame_time_;
        }
        idx++;

        // Frame data
        while (idx < lines_.size())
        {
            std::stringstream ss(lines_[idx]);
            double v;
            std::vector<double> frame;
            while (ss >> v)
                frame.push_back(v);
            frames_.push_back(frame);
            idx++;
        }
    }
};

//////////////////////////////////////////////////////////
//                多 BVH JointState 发布器
//////////////////////////////////////////////////////////
class MultiBVHJointStatePublisher : public rclcpp::Node
{
public:
    MultiBVHJointStatePublisher()
        : Node("multi_bvh_joint_publisher")
    {
        // 参数：机械臂数量
        this->declare_parameter<int>("arm_count", 1);
        this->declare_parameter<std::string>("config_dir", "/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/mamo/bvh");

        arm_count_ = this->get_parameter("arm_count").as_int();
        config_dir_ = this->get_parameter("config_dir").as_string();

        joint_names_ = {"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};

        RCLCPP_INFO(this->get_logger(), "机械臂数量: %d", arm_count_);

        // 预留空间
        parsers_.reserve(arm_count_);
        frames_list_.reserve(arm_count_);
        channel_orders_.reserve(arm_count_);
        frame_times_.reserve(arm_count_);
        publishers_.reserve(arm_count_);
        timers_.reserve(arm_count_);

        // 加载每个机械臂的 BVH
        for (int i = 1; i <= arm_count_; i++)
        {
            std::string file = config_dir_ + "/piper_" + std::to_string(i) + ".bvh";

            if (!std::filesystem::exists(file))
            {
                RCLCPP_ERROR(this->get_logger(), "BVH 文件不存在: %s", file.c_str());
                continue;
            }

            auto parser = std::make_shared<BVHParser>(file);
            parser->parse();

            parsers_.push_back(parser);
            frames_list_.push_back(parser->getFrames());
            channel_orders_.push_back(parser->getChannelOrder());
            frame_times_.push_back(parser->getFrameTime());

            // 创建发布器
            std::string topic = "/piper_" + std::to_string(i) + "/joint_states";
            publishers_.push_back(
                this->create_publisher<sensor_msgs::msg::JointState>(topic, 10)
            );

            // 创建定时器
            timers_.push_back(
                this->create_wall_timer(
                    std::chrono::duration<double>(parser->getFrameTime()),
                    [this, index = i - 1]() { timerCallback(index); }
                )
            );

            RCLCPP_INFO(this->get_logger(), "机械臂 %d BVH 加载成功: %s", i, file.c_str());
        }

        frame_indices_.resize(arm_count_, 0);
    }

private:
    int arm_count_;
    std::string config_dir_;

    std::vector<std::shared_ptr<BVHParser>> parsers_;
    std::vector<std::vector<std::vector<double>>> frames_list_;
    std::vector<std::vector<std::string>> channel_orders_;
    std::vector<double> frame_times_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> publishers_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;

    std::vector<size_t> frame_indices_;
    std::vector<std::string> joint_names_;

    void timerCallback(int idx)
    {
        const auto &frames = frames_list_[idx];
        const auto &channels = channel_orders_[idx];

        size_t frame_i = frame_indices_[idx];
        const auto &frame = frames[frame_i];

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        std::unordered_map<std::string, double> joint_pos;

        for (size_t i = 0; i < channels.size(); i++)
        {
            auto parts = split(channels[i], '_');
            std::string joint = parts[0];
            std::string ch = parts[1];

            if (ch.find("rotation") != std::string::npos || ch.find("Rotation") != std::string::npos)
            {
                joint_pos[joint] += frame[i] * M_PI / 180.0;
            }
        }

        for (auto &jn : joint_names_)
        {
            if (joint_pos.count(jn))
                msg.position.push_back(joint_pos[jn]);
            else
                msg.position.push_back(0.0);
        }

        publishers_[idx]->publish(msg);

        frame_indices_[idx] = (frame_i + 1) % frames.size();
    }

    static std::vector<std::string> split(const std::string &s, char delimiter)
    {
        std::vector<std::string> result;
        std::stringstream ss(s);
        std::string item;

        while (std::getline(ss, item, delimiter))
            result.push_back(item);

        return result;
    }
};

//////////////////////////////////////////////////////////
//                      main()
//////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiBVHJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
