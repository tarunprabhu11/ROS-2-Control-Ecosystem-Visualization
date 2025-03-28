# ROS2 Node Communication and Visualization System

## Project Overview

This project demonstrates a ROS2 node communication system with a custom visualization plugin. The system showcases inter-node communication, service interactions, and real-time graph visualization.

## Project Structure

```
project_directory/
│
├── custom_msgs/           # Custom message and service definitions
│   ├── msg/
│   │   └── NodeConnection.msg
│   └── srv/
│       └── Trigger.srv
│
├── node_system/           # Core node implementation
│   └── node_system/
│       ├── node_a.py
│       ├── node_b.py
│       ├── node_c.py
│       └── node_d.py
│
└── rqt_node_graph/        # Visualization plugin
    └── rqt_node_graph/
        └── node_graph_plugin.py
```

## Key Components

### 1. Custom Message: NodeConnection

The custom message defines the communication interface:

```python
string start         # Starting node
string end           # Destination node
bool command_interface  # Command interface flag
bool state_interface    # State interface flag
bool is_hardware        # Hardware node flag
string metadata         # Additional metadata
```

### 2. Node Communication Flow

The system implements a four-node communication chain:

- **Node A**: Initiates communication
  - Publishes messages to 'a_to_b' topic
  - 5-second timer-based publishing
  - Sets initial command interface

- **Node B**: Intermediate processing node
  - Subscribes to 'a_to_b' topic
  - Publishes to 'b_to_c' topic
  - Calls service on 'b_to_c_service'

- **Node C**: Service and message relay
  - Provides 'b_to_c_service'
  - Publishes to 'c_to_d' topic
  - Forwards messages to hardware node

- **Node D**: Hardware simulation node
  - Receives messages from 'c_to_d' topic
  - Simulates hardware processing

### 3. RQT Visualization Plugin: Advanced Features

The `node_graph_plugin.py` offers a comprehensive visualization:

#### Key Visualization Features

1. **Dynamic Node Representation**
   - Nodes colored based on type:
     - Green: Hardware nodes
     - Light Gray: Non-hardware nodes
     - Light Blue: Active nodes
     - Dark Gray: Inactive nodes

2. **Connection Visualization**
   - Color-coded connections:
     - Red: Command interface
     - Blue: State interface
   - Directional arrows showing message flow

3. **Interactive Elements**
   - Movable nodes in the graph
   - Clickable connections
   - Highlighting selected connections

4. **Detailed Message Table**
   - Real-time message tracking
   - Columns include:
     - Start/End Nodes
     - Interface Types
     - Hardware Status
     - Metadata
     - Timestamp

5. **Dynamic Updates**
   - Automatic node and connection updates
   - Inactivity detection

## Prerequisites

- ROS2 (Tested on Humble)
- Python 3.8+
- PyQt5
- rclpy

## Installation

1. Clone the repository
2. Source ROS2 environment
3. Build packages:
   ```bash
   colcon build --packages-select custom_msgs node_system rqt_node_graph
   ```

## Running the System

1. Start nodes:
   ```bash
   ros2 run node_system node_a
   ros2 run node_system node_b
   ros2 run node_system node_c
   ros2 run node_system node_d
   ```

2. Launch RQT Plugin:
   ```bash
   rqt --standalone rqt_node_graph
   ```
   or
   ```bash
   ros2 run rqt_node_graph rqt_node_graph
   ```
   

