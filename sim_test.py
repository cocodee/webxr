import numpy as np
import mujoco
import time

# 定义一个简单的 XML 模型 (一个下落的球)
xml_string = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <geom name="floor" type="plane" size="1 1 .1"/>
    <body name="ball" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size=".1" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

def main():
    print(f"Numpy version: {np.__version__}")
    print(f"MuJoCo version: {mujoco.__version__}")

    # 加载模型
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    # 运行简单的仿真循环
    print("Starting simulation loop (headless)...")
    for i in range(1000):
        mujoco.mj_step(model, data)
        
        # 打印球的高度 (验证物理引擎在工作)
        if i % 100 == 0:
            # qpos[2] 是球的 Z 轴高度
            height = data.qpos[2]
            print(f"Step {i}: Ball height = {height:.4f}")

    print("Simulation finished successfully!")

if __name__ == "__main__":
    main()