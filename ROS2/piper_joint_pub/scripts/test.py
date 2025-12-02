#!/usr/bin/env python3
import yaml
import os

# ------------ 参数 ------------
base_pos = [0.49, 0.49, -0.49, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0]
delta0    = 0.01   # base_pos[0] 递增量
delta1    = 0.01   # base_pos[1] 递增量
duration  = 1.0    # s
frames    = 100
limit_min = -3.14
limit_max = 3.14
out_file  = "trajectory2.yaml"
# ------------------------------

trajectory = []
for i in range(frames):
    p = base_pos.copy()
    # 同时递增索引 0 和 1
    if i < 50:
        p[0] = max(limit_min, min(limit_max, base_pos[0] + i * delta0))
        p[1] = max(limit_min, min(limit_max, base_pos[1] + i * delta1))


    p[2] = max(limit_min, min(limit_max, base_pos[2] - i * delta1))

    trajectory.append({
        'positions': [float(round(v, 5)) for v in p],
        'duration': duration
    })


# 写入 YAML
with open(out_file, 'w', encoding='utf-8') as f:
    yaml.dump({'wave_100': trajectory}, f, sort_keys=False, default_flow_style=None)

print(f"✅ 已生成 {frames} 帧，限幅到 [{limit_min}, {limit_max}]")
print(f"📁 保存路径：{os.path.abspath(out_file)}")