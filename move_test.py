import mujoco
import mujoco.viewer
import numpy as np
import time

# 1. 这里替换成你的 XML 文件路径
XML_PATH = "supre_robot/rf2502_new_3_std.xml"
import mujoco
import mujoco.viewer
import numpy as np
import time

# --- Configuration ---
URDF_PATH = "supre_robot/rf2502_new_3_std.urdf" # Replace with your file path

# 1. HELPER: Auto-generate an XML wrapper to add actuators
def create_actuated_model(urdf_path):
    # Load the raw model first to find joint names
    temp_model = mujoco.MjModel.from_xml_path(urdf_path)
    
    # Start creating the XML string
    xml_string = f"""
    <mujoco>
      <option timestep="0.002" gravity="0 0 -9.81"/>
      <default>
        <joint damping="0.5" frictionloss="0.1"/>
        <!-- kp=20 is the stiffness of the position control -->
        <position kp="20" dampratio="1.0"/>
      </default>
      <worldbody>
        <light pos="0 0 2"/>
        <geom type="plane" size="2 2 0.1" rgba=".9 .9 .9 1"/>
        <body pos="0 0 0">
            <include file="{urdf_path}"/>
        </body>
      </worldbody>
      
      <actuator>
    """
    
    # Loop through all joints and add a <position> actuator
    # We skip joint 0 if it is a free joint (floating base) usually, 
    # but here we iterate all.
    for i in range(temp_model.njnt):
        # Get joint name
        name = mujoco.mj_id2name(temp_model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            # Add position controller
            xml_string += f'    <position name="act_{name}" joint="{name}"/>\n'
            
    xml_string += """
      </actuator>
    </mujoco>
    """
    return xml_string

def main():
    # 2. Create the wrapper model with actuators
    print(f"Loading {URDF_PATH} and auto-rigging actuators...")
    xml = create_actuated_model(URDF_PATH)
    print(f"Created model XML.{xml}")
    # 3. Load the new model
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    
    num_actuators = model.nu # Number of controls/actuators
    print(f"Created {num_actuators} actuators.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
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