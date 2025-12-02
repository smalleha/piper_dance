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
    input_file = "/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/dance_pose_v2.yaml"
    with open(input_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    actions = data["actions"]
    y = x + 1
    start_x = 6 * x
    start_y = 6 * y

    # 提取新动作组
    new_actions = []
    for action in actions:
        new_action = {
            "name": action["name"],
            "enable": action["enable"],
            "start": action["start"][start_x:start_y],
            "end": action["end"][start_x:start_y],
            "step": action["step"],
        }
        # ✅ 仅当存在 hold_time 时添加
        if "hold_time" in action:
            new_action["hold_time"] = action["hold_time"]

        new_actions.append(new_action)

    # 构造输出文件夹与路径
    output_dir = "/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/piper/dance_pose_v2/"
    os.makedirs(output_dir, exist_ok=True)

    filename = f"piper_{x+1}.yaml"
    filepath = os.path.join(output_dir, filename)

    # 写出 YAML 文件（保持自定义格式）
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("actions:\n")
        for action in new_actions:
            f.write(f"  - name: {action['name']}\n")
            f.write(f"    enable: {str(action['enable']).lower()}\n")

            start_str = ', '.join(str(v) for v in action['start'])
            f.write(f"    start: [\n      {start_str},\n    ]\n")

            end_str = ', '.join(str(v) for v in action['end'])
            f.write(f"    end: [\n      {end_str},\n    ]\n")

            f.write(f"    step: {action['step']}\n")

            # ✅ 仅当 hold_time 存在时写入
            if "hold_time" in action:
                f.write(f"    hold_time: {action['hold_time']}\n")

    print(f"✅ 生成文件: {filepath}")

if __name__ == "__main__":
    main()
