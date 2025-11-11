import yaml
import re

def add_zero_to_arrays(yaml_file):
    """
    简化版本：专门处理start和end数组，每行末尾添加0.0
    """
    with open(yaml_file, 'r', encoding='utf-8') as file:
        content = file.read()
    
    # 处理start数组
    def add_zero_to_array(match):
        array_content = match.group(1)
        lines = array_content.split('\n')
        new_lines = []
        
        for line in lines:
            stripped = line.strip()
            if stripped and not stripped.startswith('#') and not stripped.endswith(']'):
                if stripped.endswith(','):
                    line = line.rstrip()[:-1]  # 移除末尾逗号
                new_lines.append(line + ',0.0,')
            else:
                new_lines.append(line)
        
        return '[\n' + '\n'.join(new_lines) + '\n    ]'
    
    # 使用正则表达式匹配start和end数组
    content = re.sub(r'start:\s*\[\n(.*?)\n\s*\]', 
                    lambda m: 'start: ' + add_zero_to_array(m), 
                    content, flags=re.DOTALL)
    
    content = re.sub(r'end:\s*\[\n(.*?)\n\s*\]', 
                    lambda m: 'end: ' + add_zero_to_array(m), 
                    content, flags=re.DOTALL)
    
    # 保存文件
    with open(yaml_file, 'w', encoding='utf-8') as file:
        file.write(content)
    
    print(f"✅ 已完成处理: {yaml_file}")

# 使用示例
if __name__ == "__main__":
    add_zero_to_arrays("/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/mamo/mamo_pose_v2.yaml")