# Mobile robots playground

Repo contains experiments with mobile robots.
Currently repo contains:
* Simple Diff drive robot simulation with RGLLidar Plugin
* Lidar slam
* RGLGazeboPlugin


# How to set up

Clone repo with submodules:
* `git clone --recurse-submodules`


To run in devcontainer you need to:
1. Download [NVidia OptiX](https://developer.nvidia.com/designworks/optix/downloads/legacy) **7.2**.
2. Add following to **.bashrc** `export OptiX_INSTALL_DIR=<your-OptiX-path>`.
3. Open repo in the devcontainer via vscode .with devcontainer plugin.
4. Run following commands.


``` bash
colcon build && source install/setup.bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/src/RGLGazeboPlugin/install/RGLServerPlugin:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_GUI_PLUGIN_PATH=`pwd`/src/RGLGazeboPlugin/install/RGLVisualize:$GZ_GUI_PLUGIN_PATH
```

## How to run diffdrive simulation with lidar slam

In the first terminal run:
``` bash
ros2 launch diffdrive_bringup diffdrive_bringup.launch.py
```

In the second terminal:

```
ros2 launch diffdrive_bringup lidar_slam.launch.py
```

In the third terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To save the map use:
```
ros2 service call /map_save std_srvs/Empty
```