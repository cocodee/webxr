import mujoco
import mujoco.viewer
import numpy as np
import time
import os
# 1. 这里替换成你的 XML 文件路径
# XML_PATH = "supre_robot/rf2502_fixed_collision.xml"
XML_PATH = "supre_robot/rf2502_new_3_std.xml"
# --- Configuration ---
URDF_PATH = "supre_robot/rf2502_new_3_std.urdf" # Replace with your file path

def main():
    # 获取绝对路径 (建议这样做，防止路径错误)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, XML_PATH)

    if not os.path.exists(xml_path):
        print(f"错误: 找不到文件 {xml_path}")
        return

    print(f"正在加载模型: {xml_path}")

    try:
        # 2. 直接加载 XML 文件
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
    except ValueError as e:
        print(f"加载失败: {e}")
        return
    
    num_actuators = model.nu # Number of controls/actuators
    print(f"Created {num_actuators} actuators.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        mujoco.mj_forward(model, data) 
        viewer.sync()
        
        print("仿真已暂停。请在 Viewer 中按空格键开始，或查看红色的接触点。")
        while viewer.is_running():
            t = time.time() - start_time

            # 4. Create a target vector for ALL joints
            # Let's make them move in a sine wave
            targets = np.sin(t) * np.ones(num_actuators)
            
            # 5. Apply to control input
            # This tells the motors to try to reach these positions
            data.ctrl[:] = targets

            # 6. Step physics
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    # Create a dummy URDF for testing if you don't have one
    # with open("your_robot.urdf", "w") as f:
     #   f.write("""<robot name="test"><link name="b"/><link name="l"/><joint name="j" type="revolute"><parent link="b"/><child link="l"/><axis xyz="0 1 0"/><limit lower="-1" upper="1"/></joint></robot>""")
    
    main()