import uvicorn
import json
import zenoh
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse
import os
app = FastAPI()

# --- Zenoh 配置 ---
# 获取环境变量，默认为空（自动发现模式）
ZENOH_TOPIC = "lerobot/webxr/teleop"

ZENOH_CONNECT_KEY = os.getenv("ZENOH_CONNECT_KEY")

print("Initializing Zenoh...")
z_conf = zenoh.Config()

if ZENOH_CONNECT_KEY:
    print(f"Configuring Zenoh connect to: {ZENOH_CONNECT_KEY}")
    # 注意：Zenoh Python API 的配置方式。
    # 对于较新版本 (0.10.x+)，通常是修改 Config 对象
    # 方式 A: JSON 注入
    import json
    # mode="client" 表示它不作为路由节点，只作为客户端连接
    config_json = {"mode": "client", "connect": {"endpoints": [ZENOH_CONNECT_KEY]}}
    z_conf = zenoh.Config.from_json5(json.dumps(config_json))
    
    # 方式 B (旧版本): z_conf.insert_json5("connect/endpoints", json.dumps([ZENOH_CONNECT_KEY]))
# 如果 Zenoh Router 运行在其他地方，可以在这里配置 connect=['tcp/ip:port']
z_session = zenoh.open(z_conf)
z_pub = z_session.declare_publisher(ZENOH_TOPIC)
print(f"Zenoh Publisher declared on '{ZENOH_TOPIC}'")

# --- WebSocket 处理 ---
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("Client connected via WebSocket")
    try:
        while True:
            # 1. 接收手机发来的 JSON 字符串
            data = await websocket.receive_text()
            
            # 2. 直接转发到 Zenoh (透传，速度最快)
            # 或者在这里解析一下做一些验证
            z_pub.put(data)
            
    except WebSocketDisconnect:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")

# --- 静态文件服务 (提供 HTML) ---
# 访问 http://ip:8000/ 即可
# app.mount("/", StaticFiles(directory="static", html=True), name="static")

if __name__ == "__main__":
    # 监听 0.0.0.0 允许局域网访问
    uvicorn.run(app, host="0.0.0.0", port=8000)