import yaml
import os
import sys

def main():
    # 从命令行参数获取 x 的值
    if len(sys.argv) < 2:
        print("❌ 请提供参数 x 的值")
        print("用法: python script.py <x>")
        sys.exit(1)
    
    try:
        x = int(sys.argv[1])
    except ValueError:
        print("❌ 参数 x 必须是整数")
        sys.exit(1)
    
    # 读取原始 YAML 文件
    with open("/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/mamo/mamo_pose.yaml", "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # 顶层是字典，actions 列表在 data["actions"] 中
    actions = data["actions"]
    y = x + 1

    start_x = 6 * x
    start_y = 6 * y

    # 遍历所有 actions，提取第7~12个值
    new_actions = []
    for action in actions:
        new_action = {
            "name": action["name"],
            "enable": action["enable"],
            "start": action["start"][start_x:start_y],
            "end": action["end"][start_x:start_y],
            "step": action["step"]
        }
        new_actions.append(new_action)

    # 构造新的字典，保持和原 YAML 顶层一致
    new_data = {"actions": new_actions}

    # 指定输出路径
    output_dir = "/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/mamo/"
    os.makedirs(output_dir, exist_ok=True)  # 确保目录存在

    filename = f"piper_{x+1}.yaml"
    filepath = os.path.join(output_dir, filename)  # 完整的文件路径

    # 输出新的 YAML 文件
    with open(filepath, "w", encoding="utf-8") as f:
        # 先写入顶层结构
        f.write("actions:\n")
        
        # 为每个action单独处理格式
        for action in new_actions:
            f.write(f"  - name: {action['name']}\n")
            f.write(f"    enable: {str(action['enable']).lower()}\n")
            
            # 格式化 start 数组
            start_str = ', '.join(str(x) for x in action['start'])
            f.write(f"    start: [\n      {start_str},\n    ]\n")
            
            # 格式化 end 数组  
            end_str = ', '.join(str(x) for x in action['end'])
            f.write(f"    end: [\n      {end_str},\n    ]\n")
            
            f.write(f"    step: {action['step']}\n")

    print(f"✅ 生成文件: {filepath}")

if __name__ == "__main__":
    main()