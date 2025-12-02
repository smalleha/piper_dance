#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <unordered_map>

/////////////////////////////////////////////////////////////////
//  *** BVH 节点结构体：保存 BVH 中每个 JOINT 的基本信息 ***
//  包括：名称、偏移、通道类型、子节点（树结构）
/////////////////////////////////////////////////////////////////
struct BVHNode
{
    std::string name;                                  // 关节名称
    std::vector<double> offset;                        // OFFSET 节点位移
    std::vector<std::string> channels;                 // CHANNELS 定义
    std::vector<std::shared_ptr<BVHNode>> children;    // 子关节
};

/////////////////////////////////////////////////////////////////
//  *** BVH 文件解析器：负责读取 BVH 文件、解析关节树和动作数据 ***
/////////////////////////////////////////////////////////////////
class BVHParser
{
public:
    BVHParser(const std::string &file_path)
        : file_path_(file_path)
    {
    }

    // 入口函数：开始解析 BVH
    void parse()
    {
        std::ifstream file(file_path_);
        if (!file.is_open())
            throw std::runtime_error("无法打开 BVH 文件: " + file_path_);

        // 逐行读取到 lines_
        std::string line;
        while (std::getline(file, line))
        {
            lines_.push_back(trim(line));
        }

        size_t idx = 0;

        // BVH 文件必须以 HIERARCHY 开头
        if (lines_[idx] != "HIERARCHY")
            throw std::runtime_error("BVH 格式错误，缺少 HIERARCHY");

        idx++;

        // 解析骨骼层级结构
        root_ = parseJoint(idx);

        // 解析 MOTION 部分（帧数据）
        parseMotion(idx);
    }

    const std::shared_ptr<BVHNode>& getRoot() const { return root_; }
    const std::vector<std::vector<double>>& getFrames() const { return frames_; }
    double getFrameTime() const { return frame_time_; }
    const std::vector<std::string>& getChannelOrder() const { return channel_order_; }

private:
    std::string file_path_;
    std::vector<std::string> lines_;                      // BVH 所有行

    std::shared_ptr<BVHNode> root_;                       // 根节点
    std::vector<std::vector<double>> frames_;             // 所有帧数据
    double frame_time_;                                   // 每帧时间
    std::vector<std::string> channel_order_;              // 全局通道顺序（用于解析 frames）

    //////////////////////////////////////////////////////////////////
    // 去除行首行尾空格
    //////////////////////////////////////////////////////////////////
    static std::string trim(const std::string &s)
    {
        size_t start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    //////////////////////////////////////////////////////////////////
    // *** 递归解析 JOINT 节点（包含 ROOT、JOINT、End Site） ***
    //////////////////////////////////////////////////////////////////
    std::shared_ptr<BVHNode> parseJoint(size_t &idx)
    {
        std::string line = lines_[idx];
        std::shared_ptr<BVHNode> node(new BVHNode());

        // 判断节点类型 ROOT / JOINT / End Site
        if (line.rfind("End", 0) == 0)
        {
            node->name = "EndSite";  // End Site 没有名字
        }
        else
        {
            std::stringstream ss(line);
            std::string type;
            ss >> type >> node->name;  // ROOT Name 或 JOINT Name
        }

        idx++;
        if (lines_[idx] != "{")
            throw std::runtime_error("BVH 结构错误，缺少 {");
        idx++;

        // 解析 { ... } 内部内容
        while (true)
        {
            line = lines_[idx];

            // OFFSET x y z
            if (line.rfind("OFFSET", 0) == 0)
            {
                std::stringstream ss(line);
                std::string temp;
                ss >> temp;
                double x, y, z;
                ss >> x >> y >> z;
                node->offset = {x, y, z};
            }
            // CHANNELS N channel1 channel2 ...
            else if (line.rfind("CHANNELS", 0) == 0)
            {
                std::stringstream ss(line);
                std::string temp;
                int num = 0;
                ss >> temp >> num;
                node->channels.resize(num);

                for (int i = 0; i < num; i++)
                {
                    ss >> node->channels[i];

                    // 保存“节点名+通道名”，用于匹配 frame 数值
                    channel_order_.push_back(node->name + "_" + node->channels[i]);
                }
            }
            // 嵌套子关节，递归解析
            else if (line.rfind("JOINT", 0) == 0 || line.rfind("End", 0) == 0)
            {
                node->children.push_back(parseJoint(idx));
                continue;
            }
            // 当前节点闭合，返回
            else if (line == "}")
            {
                idx++;
                return node;
            }

            idx++;
        }
    }

    //////////////////////////////////////////////////////////////////
    // *** 解析 MOTION 部分（帧数据、帧时间和逐帧通道值） ***
    //////////////////////////////////////////////////////////////////
    void parseMotion(size_t &idx)
    {
        if (lines_[idx] != "MOTION")
            throw std::runtime_error("BVH 缺少 MOTION");

        idx++;

        // Frames: N
        {
            std::stringstream ss(lines_[idx]);
            std::string temp;
            int frame_count;
            ss >> temp >> frame_count;
            frames_.reserve(frame_count);
        }
        idx++;

        // Frame Time: xxx
        {
            std::stringstream ss(lines_[idx]);
            std::string temp;
            ss >> temp >> temp >> frame_time_;
        }
        idx++;

        // 每行代表一帧
        while (idx < lines_.size())
        {
            std::stringstream ss(lines_[idx]);
            std::vector<double> frame;
            double v;
            while (ss >> v)
                frame.push_back(v);

            frames_.push_back(frame);
            idx++;
        }
    }
};

/////////////////////////////////////////////////////////////////
//     ****  ROS2 Node：将 BVH 逐帧转换成 JointState 发布 ****
/////////////////////////////////////////////////////////////////
class BVHJointStatePublisher : public rclcpp::Node
{
public:
    BVHJointStatePublisher()
        : Node("bvh_jointstate_publisher"), frame_index_(0)
    {
        // 固定机械臂关节名字
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6","joint7"};

        // 读取参数：BVH 文件路径
        this->declare_parameter<std::string>("file_path", "");
        file_path_ = this->get_parameter("file_path").as_string();

        if (file_path_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "请通过 file_path:=xxx.bvh 指定 BVH 文件路径");
            return;
        }

        // 解析 BVH
        parser_ = std::make_shared<BVHParser>(file_path_);
        parser_->parse();

        frames_ = parser_->getFrames();          // 所有帧数据
        frame_time_ = parser_->getFrameTime();   // 每帧间隔
        channel_order_ = parser_->getChannelOrder();

        // 创建 JointState 发布器
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 定时器：按 BVH 帧率发布动画
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(frame_time_),
            std::bind(&BVHJointStatePublisher::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "BVH 加载成功: %s", file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "总帧数: %zu   帧时间: %f sec", frames_.size(), frame_time_);
    }

private:
    std::string file_path_;
    std::shared_ptr<BVHParser> parser_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<double>> frames_;   // 帧数据
    double frame_time_;                         // 每帧时间
    std::vector<std::string> channel_order_;    // BVH 通道顺序
    std::vector<std::string> joint_names_;      // 机械臂关节名

    size_t frame_index_;

    //////////////////////////////////////////////////////////////////
    //   *** 定时器回调：每次发布 BVH 的一帧 JointState ***
    //////////////////////////////////////////////////////////////////
    void timerCallback()
    {
        const auto &frame = frames_[frame_index_];

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        // 映射：BVH 通道名 → 关节角度（rad）
        std::unordered_map<std::string, double> joint_pos;

        for (size_t i = 0; i < channel_order_.size(); i++)
        {
            auto parts = split(channel_order_[i], '_');
            std::string joint = parts[0];       // BVH 关节名
            std::string channel = parts[1];     // Xrotation / Yrotation / Zrotation

            // 只处理旋转通道
            if (channel.find("rotation") != std::string::npos ||
                channel.find("Rotation") != std::string::npos)
            {
                joint_pos[joint] += frame[i] * M_PI / 180.0; // 转弧度
            }
        }

        // 写入 position，若关节不存在则补 0
        for (auto &jn : joint_names_)
        {
            if (joint_pos.count(jn))
                msg.position.push_back(joint_pos[jn]);
            else
                msg.position.push_back(0.0);  // 自动补零
        }

        pub_->publish(msg);

        // 下一帧
        frame_index_ = (frame_index_ + 1) % frames_.size();
    }

    // 工具函数：字符串分割
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

/////////////////////////////////////////////////////////////////
//                      *** main ***
/////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BVHJointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
