import zenoh
import json
import time
import sys

# 配置
TOPIC = "lerobot/webxr/teleop"
# 连接到 Docker 暴露出来的端口
CONNECT_KEY = "tcp/107.175.133.248:7447" 

def listener(sample):
    """
    回调函数：当收到 Zenoh 消息时触发
    """
    try:
        # 1. 解码 payload (bytes -> string)
        if hasattr(sample.payload, 'to_bytes'):
            payload_bytes = sample.payload.to_bytes()
        else:
            # 兼容旧版本或直接是 bytes 的情况
            payload_bytes = sample.payload
            
        # 解码 bytes -> string
        payload_str = payload_bytes.decode('utf-8')
        
        # 2. 解析 JSON
        data = json.loads(payload_str)
        
        # 3. 格式化输出
        # WebXR 发送的数据结构: {'p': [x,y,z], 'q': [x,y,z,w], 'g': 1.0}
        pos = data.get('p', [0, 0, 0])
        quat = data.get('q', [0, 0, 0, 1])
        gripper = data.get('g', 0)
        
        # 清除当前行并打印 (简单的刷新效果)
        output = (
            f"\r[接收数据] "
            f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | "
            f"Rot: ({quat[0]:.2f}, {quat[1]:.2f}, {quat[2]:.2f}, {quat[3]:.2f}) | "
            f"Gripper: {gripper}"
        )
        sys.stdout.write(output)
        sys.stdout.flush()
        
    except Exception as e:
        print(f"\n[Error] 解析数据失败: {e}")

def main():
    print("--- Zenoh WebXR 接收测试 ---")
    
    # 1. 配置 Zenoh
    # 设置为 client 模式，并连接到本地的 7447 端口
    conf = zenoh.Config()
    
    # 构建配置 JSON (显式指定连接端点)
    config_json = {
        "mode": "client", 
        "connect": {
            "endpoints": [CONNECT_KEY]
        }
    }
    
    # 加载配置
    try:
        conf = zenoh.Config.from_json5(json.dumps(config_json))
    except Exception as e:
        print(f"配置错误: {e}")
        return

    # 2. 打开 Session
    print(f"正在连接到 {CONNECT_KEY} ...")
    session = zenoh.open(conf)
    print("Zenoh 连接成功!")

    # 3. 订阅 Topic
    print(f"正在订阅 Topic: {TOPIC}")
    sub = session.declare_subscriber(TOPIC, listener)
    
    print("等待手机数据... (按 Ctrl+C 退出)")
    
    # 4. 保持运行
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n正在停止...")
        sub.undeclare()
        session.close()

if __name__ == "__main__":
    main()