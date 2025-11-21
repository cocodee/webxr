import mujoco
import mujoco.viewer
import numpy as np
import time

# 1. 这里替换成你的 XML 文件路径
XML_PATH = "supre_robot/rf2502_new_3_std.xml"

def main():
    # 2. 加载模型
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
        data = mujoco.MjData(model)
    except ValueError:
        print(f"错误: 找不到文件 {XML_PATH}，请确保路径正确。")
        return

    # 3. 获取执行器数量 (nu = number of actuators/controls)
    num_actuators = model.nu
    print(f"模型加载成功! 检测到 {num_actuators} 个执行器 (Actuators)。")

    if num_actuators == 0:
        print("警告: XML中没有定义 <actuator>，无法使用 data.ctrl 控制。")
        return

    # 4. 启动仿真循环
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        while viewer.is_running():
            # 时间 t
            t = time.time() - start_time

            # --- 设置所有关节的目标位置 ---
            
            # 方案 A: 所有关节设为同一个值 (例如都在动)
            # 这里用正弦波演示：所有关节都在 -0.5 到 0.5 之间摆动
            target_value = np.sin(t) * 0.5
            all_joint_targets = np.full(num_actuators, target_value)
            
            # 方案 B: 为每个关节单独设置不同的值
            # 假设你有 3 个关节，你想分别设为 0.1, 0.2, 0.3
            # if num_actuators == 3:
            #     all_joint_targets = np.array([0.1, 0.2, 0.3])

            # --- 核心步骤: 将目标值赋给 data.ctrl ---
            # data.ctrl 的顺序严格对应 XML 中 <actuator> 出现的顺序
            data.ctrl[:] = all_joint_targets

            # 5. 物理步进 (计算力矩、移动关节)
            mujoco.mj_step(model, data)
            
            # 6. 同步画面
            viewer.sync()
            
            # 简单的帧率控制
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    # 为了演示方便，如果没有文件，我生成一个临时的供你测试
    # 如果你已经有了文件，这段可以直接忽略或删除
    import os
    if not os.path.exists(XML_PATH):
        print("未找到文件，生成临时测试模型...")
        dummy_xml = """
        <mujoco>
          <option timestep="0.002"/>
          <worldbody>
            <light pos="0 0 1"/>
            <body pos="0 0 0"><joint name="j1" axis="1 0 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.05"/>
              <body pos="0.2 0 0"><joint name="j2" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.04"/></body>
            </body>
          </worldbody>
          <actuator>
            <position name="act1" joint="j1" kp="10"/>
            <position name="act2" joint="j2" kp="10"/>
          </actuator>
        </mujoco>
        """
        with open(XML_PATH, "w") as f: f.write(dummy_xml)

    main()