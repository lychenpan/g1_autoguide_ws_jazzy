# g1_ws_jazzy

Python-only bringup for Unitree G1 SLAM + Nav2 on ROS 2 Jazzy. This repo provides bridges, launch files, maps, and helper scripts. No `colcon build` is required in this workspace.

## Workflow

### env start: use robotenv1 to activate env.

1. **Start SLAM + odom bridge**

   ```bash
   ./startslam.sh
   ```

2. **Start cmd_vel bridge**

   ```bash
   ./runcmd.sh
   ```
3. **Start pointcloud bridge**

   python3 nodes/utlidar_pcl_bridge.py

4. **Start Nav2**

   ```bash
   ./startnav2.sh
   ```

   Startup sequence should be:
   `startslam.sh` -> `runcmd.sh` -> `startnav2.sh`.

5. **Send a navigation goal**

   ```bash
   ./send_goal.sh <x> <y> <yaw_degrees>
   ```

   Example:

   ```bash
   ./send_goal.sh 3 3 0
   ```

   For the robot to move, also run `nodes/cmd_vel_bridge.py` (or your usual cmd_vel bridge) in a separate terminal.

## Layout

| Path | Purpose |
|------|---------|
| `g1_api_nav2/` | Nav2 params, map, launch file |
| `nodes/` | Odom bridge, cmd_vel bridge, camera nodes |
| `tools/` | SLAM relocation, map conversion, TTS, etc. |
| `handshake/` | Handshake / inspire hand experiments |

## Planned work

1. Voice chat and microphone integration
2. Handshake testing and optimization
