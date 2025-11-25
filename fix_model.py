import mujoco
import os

# 1. 设置路径
xml_dir = "supre_robot"
input_xml = "rf2502_new_3_std.xml"
output_xml = "rf2502_fixed_collision.xml"

full_input_path = os.path.join(xml_dir, input_xml)
full_output_path = os.path.join(xml_dir, output_xml)

# 2. 加载现有模型
print("正在加载模型以分析结构...")
model = mujoco.MjModel.from_xml_path(full_input_path)

# 3. 自动生成排除列表 (Exclude List)
# 逻辑：遍历所有物体，如果它是另一个物体的子物体（parentid不为0），
# 则它们之间一定由关节连接，必须排除碰撞。
exclude_lines = []
body_names = []

# 获取所有 body 的名字
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    body_names.append(name)

# 遍历找出父子关系
print("正在生成碰撞排除规则...")
for i in range(1, model.nbody): # 从1开始，跳过 worldbody(0)
    parent_id = model.body_parentid[i]
    if parent_id == 0: # 如果父物体是 world，通常不需要排除（除非底座穿地）
        continue
        
    child_name = body_names[i]
    parent_name = body_names[parent_id]
    
    # 生成 XML 行
    line = f'    <exclude body1="{parent_name}" body2="{child_name}"/>'
    exclude_lines.append(line)
    print(f"  排除: {parent_name} <-> {child_name}")

# 4. 读取原始 XML 文本并插入内容
with open(full_input_path, 'r') as f:
    content = f.read()

# 移除你之前添加的全局无碰撞设置 (如果有的话)
content = content.replace('<geom contype="0" conaffinity="0"/>', '')

# 准备插入 <contact> 块
contact_block = "\n  <contact>\n" + "\n".join(exclude_lines) + "\n  </contact>\n"

# 插入到 </mujoco> 之前
if "</mujoco>" in content:
    new_content = content.replace("</mujoco>", contact_block + "</mujoco>")
else:
    print("错误: 找不到 </mujoco> 标签")
    exit()

# 5. 保存新文件
with open(full_output_path, 'w') as f:
    f.write(new_content)

print(f"\n成功！已生成修复版 XML: {full_output_path}")
print("请使用这个新文件运行 move_test.py，碰撞应该正常了。")