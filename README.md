# ROS2 Test Project

This project demonstrates the integration of ROS2 with a web interface using rosbridge and React.

## Components

1. ROS2 Publisher Node
   - Publishes random data to the `random_data` topic
   - Data includes random numbers, timestamps, and messages

2. Rosbridge Server
   - Runs on port 9090
   - Uses Python 3.10 environment
   - Bridges ROS2 and WebSocket communication

3. React Web Client
   - Connects to rosbridge via WebSocket
   - Displays real-time data from the ROS2 publisher
   - Runs on port 3000

## Project Structure

```
test_publisher/
├── launch/
│   └── test_system.launch.py
├── test_publisher/
│   ├── __init__.py
│   └── random_publisher.py
├── web-client/
│   ├── src/
│   │   ├── App.tsx
│   │   └── hooks/
│   │       └── useROS.ts
│   ├── package.json
│   └── tsconfig.json
├── setup.py
└── package.xml
```

## Prerequisites

1. ROS2 (tested with Humble)
2. Python 3.10 environment with rosbridge-server installed
3. Node.js and npm

## Installation

1. Clone the repository:
```bash
git clone https://github.com/vcb88/ros2-computer-use.git
```

2. Build the ROS2 package:
```bash
cd ros2-computer-use
colcon build
```

3. Install React dependencies:
```bash
cd test_publisher/web-client
npm install
```

## Usage

1. Source the ROS2 workspace:
```bash
source install/setup.bash
```

2. Launch the system:
```bash
ros2 launch test_publisher test_system.launch.py
```

This will start:
- The random data publisher
- Rosbridge server on port 9090
- React development server on port 3000

Visit http://localhost:3000 to see the web interface.

## License

Apache License 2.0