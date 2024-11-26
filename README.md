# ROS2 Web Integration Demo

This project demonstrates the integration of ROS2 with a web interface using rosbridge and React. It includes a ROS2 node that publishes random data, a rosbridge server that enables WebSocket communication, and a React web client that displays the data in real-time.

## Components

1. ROS2 Publisher Node
   - Publishes random data to the `random_data` topic
   - Data includes random numbers, timestamps, and messages
   - Uses standard String messages with JSON-formatted data

2. Rosbridge Server
   - Runs on port 9090
   - Uses Python 3.10 environment
   - Bridges ROS2 and WebSocket communication
   - Enables web clients to subscribe to ROS2 topics

3. React Web Client
   - Connects to rosbridge via WebSocket
   - Displays real-time data from the ROS2 publisher
   - Built with TypeScript and modern React practices
   - Development: Runs on port 3000 (Vite)
   - Production: Served via Nginx on port 80

## Project Structure

```
test_publisher/
├── config/
│   ├── nginx/                   # Nginx configuration
│   └── systemd/                 # Systemd service files
├── launch/
│   ├── test_system_dev.launch.py    # Development launch file
│   └── test_system_prod.launch.py   # Production launch file
├── scripts/
│   └── setup_production.sh      # Production setup script
├── test_publisher/
│   ├── __init__.py
│   └── random_publisher.py      # Main ROS2 publisher node
├── web-client/                  # React application
│   ├── src/
│   │   ├── App.tsx             # Main React component
│   │   ├── hooks/
│   │   │   └── useROS.ts       # Custom hook for ROS connection
│   │   └── types/
│   │       └── messages.ts      # TypeScript type definitions
│   ├── package.json
│   └── tsconfig.json
├── setup.py                     # ROS2 package setup
└── package.xml                  # ROS2 package manifest
```

## Prerequisites

1. ROS2 Humble
   ```bash
   # Verify ROS2 installation
   ros2 --version
   ```

2. Python 3.10 Environment with rosbridge
   ```bash
   # Create Python 3.10 virtual environment
   python3.10 -m venv ~/ros2_py310_env
   source ~/ros2_py310_env/bin/activate
   
   # Install rosbridge and dependencies
   pip install rosbridge-server rospkg
   ```

3. Node.js (v20 or higher) and npm
   ```bash
   # Verify Node.js installation
   node --version  # Should be >=20.0.0
   npm --version
   ```

4. For Production:
   - Nginx
   - Systemd

## Installation

### Development Setup

1. Clone the repository:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/vcb88/ros2-computer-use.git
   ```

2. Build the ROS2 package:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build
   ```

3. Install React dependencies:
   ```bash
   cd ~/ros2_ws/src/ros2-computer-use/test_publisher/web-client
   npm install
   ```

### Production Setup

1. Follow the development setup steps 1-2

2. Run the production setup script:
   ```bash
   cd ~/ros2_ws/src/ros2-computer-use/test_publisher/scripts
   chmod +x setup_production.sh
   ./setup_production.sh
   ```

## Usage

### Development Environment

1. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. Launch all components:
   ```bash
   ros2 launch test_publisher test_system_dev.launch.py
   ```

Visit http://localhost:3000 to see the web interface.

### Production Environment

1. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. Launch all components:
   ```bash
   ros2 launch test_publisher test_system_prod.launch.py
   ```

Visit http://localhost to see the web interface.

### Running Components Individually (Development)

1. Start the ROS2 publisher:
   ```bash
   # Terminal 1
   source ~/ros2_ws/install/setup.bash
   ros2 run test_publisher random_publisher
   ```

2. Start the rosbridge server (with Python 3.10):
   ```bash
   # Terminal 2
   source ~/ros2_py310_env/bin/activate
   PYTHONPATH=$PYTHONPATH:/opt/ros/humble/lib/python3.10/site-packages python3 -m rosbridge_server.launch_rosbridge --port 9090
   ```

3. Start the Vite development server:
   ```bash
   # Terminal 3
   cd ~/ros2_ws/src/ros2-computer-use/test_publisher/web-client
   npm run dev
   ```

### Managing Production Services

- Start services:
  ```bash
  systemctl --user start ros2-random-publisher
  systemctl --user start ros2-web-bridge
  sudo systemctl start nginx
  ```

- Stop services:
  ```bash
  systemctl --user stop ros2-random-publisher
  systemctl --user stop ros2-web-bridge
  sudo systemctl stop nginx
  ```

- View logs:
  ```bash
  journalctl --user -u ros2-random-publisher
  journalctl --user -u ros2-web-bridge
  sudo journalctl -u nginx
  ```

## Troubleshooting

1. If rosbridge fails to start:
   - Make sure you're using Python 3.10
   - Verify the PYTHONPATH includes ROS2 packages
   - Check if port 9090 is available
   - In production, check systemd service logs

2. If the web client shows "Disconnected":
   - Verify rosbridge is running
   - Check browser console for WebSocket errors
   - In production, verify Nginx WebSocket proxy configuration
   - Ensure firewall allows connection to port 9090

3. If no data appears:
   - Check if the publisher is running with `ros2 topic list`
   - Verify data with `ros2 topic echo /random_data`
   - Check browser console for any errors
   - In production, check systemd service status

4. Production specific issues:
   - Check Nginx error logs: `sudo tail -f /var/log/nginx/error.log`
   - Verify systemd services: `systemctl --user status ros2-*`
   - Check file permissions in /var/www/ros2-web-client

## Contributing

Feel free to submit issues and enhancement requests!

## License

Apache License 2.0