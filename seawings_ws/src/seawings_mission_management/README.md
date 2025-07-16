# SEAWINGS Mission Management System

## Architecture

The system consists of three main ROS 2 nodes:

### 1. PowerMonitor Node
- Monitors battery status and power consumption
- Calculates remaining flight time and return-to-launch requirements
- Triggers automatic RTL when battery reaches critical levels
- Uses Haversine formula for distance calculations

### 2. FaultDetector Node
- Monitors GPS integrity and system health
- Detects GPS signal loss and sensor failures
- Triggers emergency landing when positioning systems fail
- Configurable thresholds for GPS fix quality

### 3. MissionSupervisor Node
- Central coordination and decision-making
- Monitors communication links and mission duration
- Implements failsafe backup logic
- Maintains mission state machine

## Communication Interfaces

### MAVROS 2 Integration
- Flight mode commands (`/mavros/set_mode`)
- System state monitoring (`/mavros/state`)
- Arming/disarming commands (`/mavros/cmd/arming`)

### micro XRCE-DDS Bridge
- Direct access to PX4 uORB topics
- Low-latency telemetry data
- Battery status, GPS position, vehicle status

## Installation

### Prerequisites
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install PX4 dependencies
sudo apt install python3-pip python3-colcon-common-extensions
pip3 install --user -U empy pyros-genmsg setuptools

# Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Install micro XRCE-DDS
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install

# Install Px4_msgs
# see documentation
```


## Usage

### 1. Launch Complete System
```bash
# Start PX4 SITL, Gazebo, MAVROS, and mission nodes
ros2 launch seawings_mission_management complete_system.launch.py

# Or launch components separately:
# PX4 SITL with Gazebo
ros2 launch seawings_mission_management px4_sitl_gazebo.launch.py

# Mission management nodes
ros2 launch seawings_mission_management seawings_mission.launch.py
```

### 2. Configure Parameters
Edit `config/mission_params.yaml` to adjust:
- Battery safety margins
- GPS failure thresholds  
- Mission timeout values
- Check intervals

### 3. Monitor System
```bash
# View mission status
ros2 topic echo /seawings/mission_status

# Check node logs
ros2 log view /seawings/power_monitor
ros2 log view /seawings/fault_detector
ros2 log view /seawings/mission_supervisor

# Monitor telemetry
ros2 topic echo /fmu/out/battery_status
ros2 topic echo /fmu/out/vehicle_gps_position
```

## Testing

### Simulate Battery Failure
```bash
# Run battery drain simulator
ros2 run seawings_mission_management test_battery_failure.py

# Expected behavior: RTL triggered when battery < 25%
```

### Simulate GPS Failure
```bash
# Run GPS failure simulator
ros2 run seawings_mission_management test_gps_failure.py

# Expected behavior: Emergency landing when GPS lost
```

### PX4 SITL Commands
```bash
# In PX4 console, inject failures:
failure battery
failure gps off
failure datalink

# Check failsafe behavior in QGroundControl
```

## Configuration Parameters

### PowerMonitor
- `safety_margin`: Battery safety margin (default: 0.25)
- `average_return_speed`: Expected return speed in m/s (default: 12.0)
- `battery_check_interval`: Check frequency in seconds (default: 5.0)
- `rtl_triggered_threshold`: Battery level to trigger RTL (default: 0.25)

### FaultDetector
- `gps_timeout`: GPS signal timeout in seconds (default: 15.0)
- `min_satellites`: Minimum satellites for valid fix (default: 6)
- `min_fix_type`: Minimum GPS fix type (default: 3)
- `check_interval`: Health check frequency (default: 2.0)

### MissionSupervisor
- `heartbeat_timeout`: MAVROS heartbeat timeout (default: 30.0)
- `telemetry_timeout`: Telemetry timeout (default: 10.0)
- `mission_timeout`: Maximum mission duration (default: 3600.0)
- `check_interval`: Supervision frequency (default: 1.0)


## Extension Points


1. **Additional Sensors**: Add new fault detection logic in `FaultDetector
2. **Operator Interface**: Implement command/control interfaces
3. **Data Logging**: Add flight data recording functionality


# ============================================================================
# BUILD AND DEPLOYMENT INSTRUCTIONS
# ============================================================================


## Make Scripts Executable
```bash
cd ~/seawings_ws/src/seawings_mission_management
chmod +x seawings_mission_management/*.py
chmod +x launch/*.py
chmod +x scripts/*.py
```

## Build the Package
```bash
cd ~/seawings_ws
colcon build --packages-select seawings_mission_management
source install/setup.bash
```

## Step 5: Verify Installation
```bash
# Check if nodes are available
ros2 pkg executables seawings_mission_management

# Expected output:
# seawings_mission_management fault_detector
# seawings_mission_management mission_supervisor  
# seawings_mission_management power_monitor
```

## Step 6: Test Individual Nodes
```bash
# Test each node individually (in separate terminals)
ros2 run seawings_mission_management power_monitor
ros2 run seawings_mission_management fault_detector
ros2 run seawings_mission_management mission_supervisor
```


## Test Complete System
```bash
# Terminal 1: Launch complete system
ros2 launch seawings_mission_management complete_system.launch.py

# Terminal 2: Monitor topics
ros2 topic echo /seawings/mission_status

# Terminal 3: Check battery status
ros2 topic echo /fmu/out/battery_status

# Terminal 4: Simulate failures
ros2 run seawings_mission_management test_battery_failure.py
```

## Step 10: Validation Tests

### Battery Failure Test
1. Launch system: `ros2 launch seawings_mission_management complete_system.launch.py`
2. Arm vehicle in QGroundControl
3. Start mission
4. Run battery simulator: `ros2 run seawings_mission_management test_battery_failure.py`
5. Observe RTL trigger when battery < 25%

### GPS Failure Test
1. Launch system and start mission
2. Run GPS simulator: `ros2 run seawings_mission_management test_gps_failure.py`
3. Observe emergency landing when GPS fails

### Communication Loss Test
1. Launch system and start mission
2. Kill MAVROS process: `pkill -f mavros`
3. Observe failsafe behavior in logs

## Expected Behaviors

### PowerMonitor
- Logs battery status every 5 seconds
- Triggers RTL when battery < 25% or insufficient return power
- Uses configurable safety margins

### FaultDetector  
- Monitors GPS fix status every 2 seconds
- Triggers emergency landing on GPS loss > 15 seconds
- Logs system health status

### MissionSupervisor
- Coordinates all mission decisions
- Monitors communication links
- Implements backup failsafe logic
- Maintains mission state machine

## Integration Notes

1. **PX4 Configuration**: Ensure PX4 parameters allow external mode changes
2. **MAVROS Topics**: Verify topic names match your PX4/MAVROS configuration


