import pybullet as p
import pybullet_data
import time

# ================= 配置区域 =================
# 【重要】请将下面的路径替换为你自己的 URDF 文件绝对路径或相对路径
# 如果找不到文件，脚本会使用 PyBullet 自带的 r2d2 机器人作为演示
# 例如: urdf_path = "./lerobot/robots/my_robot.urdf"
#urdf_path = "r2d2.urdf" 
urdf_path = "../urdf/RJ2506-20251125-all-13-std.urdf"
urdf_path = "../supre_robot/rf2502_new_3_std.urdf"
# ===========================================

def main():
    # 1. 连接 PyBullet (使用 GUI 模式以便你能看到机器人)
    print("正在启动 PyBullet...")
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 设置重力 (可选)
    p.setGravity(0, 0, -9.8)

    # 2. 加载地面 (为了视觉参照)
    p.loadURDF("plane.urdf")

    # 3. 加载机器人
    try:
        print(f"正在加载模型: {urdf_path}")
        # useFixedBase=True 让机器人悬在半空，方便观察，不受重力倒下
        robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.5], useFixedBase=True)
    except Exception as e:
        print(f"\n[错误] 无法加载 URDF 文件: {urdf_path}")
        print("请检查路径是否正确。错误详情:", e)
        p.disconnect()
        return

    # 4. 获取关节信息
    num_joints = p.getNumJoints(robot_id)
    print(f"\n成功加载! 机器人 ID: {robot_id}, 总关节/连杆数: {num_joints}")
    print("-" * 80)
    print(f"{'Index (ID)':<10} | {'Joint Name':<30} | {'Type':<15} | {'Status'}")
    print("-" * 80)

    # 用于生成代码字典的容器
    joint_dict = {}

    # 5. 遍历所有关节
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        
        # joint_info[0] = jointIndex
        # joint_info[1] = jointName (bytes)
        # joint_info[2] = jointType (int)
        
        joint_id = joint_info[0]
        joint_name = joint_info[1].decode("utf-8") # 解码 bytes 为 string
        joint_type_code = joint_info[2]
        
        # 解析关节类型
        type_str = "UNKNOWN"
        if joint_type_code == p.JOINT_REVOLUTE: type_str = "REVOLUTE (旋转)"
        elif joint_type_code == p.JOINT_PRISMATIC: type_str = "PRISMATIC (移动)"
        elif joint_type_code == p.JOINT_SPHERICAL: type_str = "SPHERICAL (球)"
        elif joint_type_code == p.JOINT_PLANAR: type_str = "PLANAR (平面)"
        elif joint_type_code == p.JOINT_FIXED: type_str = "FIXED (固定)"
        
        # 判断是否应该包含在控制字典里 (通常只控制非固定关节)
        status = "✅ 可控" if joint_type_code != p.JOINT_FIXED else "🔒 固定"
        
        print(f"{joint_id:<10} | {joint_name:<30} | {type_str:<15} | {status}")
        
        # 存入字典
        joint_dict[joint_name] = joint_id

    print("-" * 80)

    # 6. 打印可直接复制的 Python 代码
    print("\n[生成的 Python 代码字典] (可直接复制到你的代码中):")
    print("self.joint_name2id = {")
    
    # 格式化输出
    items = list(joint_dict.items())
    for idx, (name, jid) in enumerate(items):
        comma = "," if idx < len(items) - 1 else ""
        # 这里的输出格式化一下，每行显示一个，方便阅读
        print(f'    "{name}": {jid}{comma}')
        
    print("}")
    
    # 7. 保持窗口开启一段时间
    print("\n仿真将在 300 秒后自动关闭，或者你可以手动关闭窗口...")
    time.sleep(300)
    p.disconnect()

if __name__ == "__main__":
    main()