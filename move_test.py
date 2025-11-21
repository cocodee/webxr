import mujoco
import mujoco.viewer
import numpy as np
import time

# --- Configuration ---
URDF_PATH = "supre_robot/rf2502_new_3_std.urdf" # Replace with your file path

def main():
    # 2. Create the wrapper model with actuators
    print(f"Loading {URDF_PATH} and auto-rigging actuators...")
    xml = create_actuated_model(URDF_PATH)
    
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
    main()