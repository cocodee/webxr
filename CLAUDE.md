# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **WebXR Teleoperation System** that enables remote control of robotic arms using mobile devices in AR mode. The system follows a microservices architecture with real-time communication between components.

## Architecture

### Core Components
1. **WebXR Frontend** (`src/nginx/static/index.html`): Mobile web application for AR control
2. **FastAPI Backend** (`src/server/server.py`): WebSocket server and Zenoh message publisher
3. **Zenoh Router**: Eclipse Zenoh message router for distributed communication
4. **Nginx Proxy**: Web server with SSL termination and WebSocket proxy

### Communication Flow
```
Mobile WebXR Client ←→ WebSocket ←→ FastAPI Server ←→ Zenoh ←→ Robot Control System
```

### Data Protocol (JSON payload)
```json
{
  "p": [x, y, z],      // Position array
  "q": [qx, qy, qz, qw], // Quaternion array
  "g": 0.0,            // Gripper value (0-1)
  "m": "IDLE"          // Control mode
}
```

## Development Commands

### Containerized Development (Recommended)
```bash
# Start all services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop all services
docker-compose down

# Rebuild and start
docker-compose up --build -d
```

### Backend Development
```bash
# Navigate to server directory
cd src/server

# Install dependencies
pip install -r requirements.txt

# Run development server
uvicorn server:app --host 0.0.0.0 --port 8000 --reload
```

### Frontend Development
The frontend uses plain JavaScript with CDN-hosted Three.js. No build tools required:
- Directly edit `src/nginx/static/index.html`
- Test by accessing the Nginx service (port 443/80)

## Key Configuration

### Environment Variables
- `ZENOH_CONNECT_KEY`: Zenoh router connection string (e.g., `tcp/host:port`)
- Default: Auto-discovery mode if not set

### Zenoh Configuration
- Topic: `lerobot/webxr/teleop`
- Default port: 7447 (TCP/UDP)
- Router configuration in `docker-compose.yaml`

### SSL Configuration
- Self-signed certificates generated in Nginx container
- Access via `https://localhost` (ignore certificate warnings)

## Design Constraints

### UI Principles (from `documents/web.md`)
1. **Toggle + Clutch Interaction**: No continuous press controls
2. **Thumb-Friendly Controls**: Buttons positioned in bottom 20-30% of screen
3. **AR-First Design**: Mobile device position drives robot control
4. **Emergency Disengage**: Double-tap anywhere to return to IDLE state

### State Management
- `IDLE`: No control active
- `TRANSLATE`: Move mode (left button)
- `BOTH`: Combined move and rotate mode (center button)
- `ROTATE`: Rotate mode (right button)

### Gripper Control
- Continuous slider control for gripper (0-1 range)
- Positioned above control buttons for thumb access
- Real-time value display showing 2 decimal places

## Important Notes

### Code Style
- No continuous speed/gain adjustment UI elements
- All control driven by device pose changes
- Minimal dependencies for reliability
- Industrial-grade ergonomics focus

### Testing
- Physical device testing required for AR functionality
- No formal testing framework implemented
- Use `sim_test.py` and `test_subscriber.py` for simulation testing

### Security
- HTTPS enforced with self-signed certificates
- WebSocket connections proxied through Nginx
- Zenoh message bus for secure distributed communication

## File Structure Guide

### Key Directories
- `src/server/`: FastAPI backend code
- `src/nginx/`: Web server configuration and static files
- `supre_robot/`: MuJoCo robot model definitions
- `urdf/`: URDF robot model files
- `documents/`: Design documentation

### Critical Files
- `server.py`: Core backend logic and WebSocket handling
- `index.html`: Frontend implementation with Three.js
- `web.md`: Comprehensive UI design specifications
- `docker-compose.yaml`: Multi-service orchestration
- `default.conf`: Nginx configuration