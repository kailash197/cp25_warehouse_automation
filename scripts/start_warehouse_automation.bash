#!/bin/bash

echo "WELCOME TO WAREHOUSE AUTOMATION SHELL"
# ==== ENVIRONMENT ====
ENV=${1:-sim} #Default: sim

# ==== Functions ====
killgazeboprocesses() {
    echo "Searching for Gazebo processes..."
    echo "Killing Gazebo processes..."
    ps -ef | grep gazebo | grep -v grep | awk '{print $2}' | xargs -r kill -9
    ps -ef | grep gz | grep -v grep | awk '{print $2}' | xargs -r kill -9
    echo "All Gazebo processes terminated."
}

# ==== Logging ====
LOG_DIR=~/ros2_ws/src/warehouse_automation/logs
rm -rf ~/ros2_ws/src/warehouse_automation/logs
mkdir -p $LOG_DIR

# ==== Install Dependencies ====
echo
echo "[INFO] Updating package list..."
sudo apt update > $LOG_DIR/install_dependencies.log 2>&1
sudo apt install -y --no-install-recommends moreutils >> $LOG_DIR/install_dependencies.log 2>&1
echo "[INFO] Dependencies installed successfully"

# ==== Build projects ====
echo
echo "Building projects..."
cd ~/ros2_ws && colcon build  > >(ts '[%Y%m%d%H%M%S]' > $LOG_DIR/ros2_ws_build.log) 2>&1
source ~/ros2_ws/install/setup.bash

# ==== Start Warehouse Automation ====

# ==== Find the environment: SIM or REAL ====
echo
source /opt/ros/${ROS_DISTRO}/setup.bash
if [[ "$ENV" == "real" ]]; then
    echo "Welcome to the WAREHOUSE LAB!"
    USE_SIM_TIME=false
else

    echo "Starting SIMULATION environment..."
    USE_SIM_TIME=true
fi

# ==== LOCALIZATION SERVER ====
echo
echo "Starting localization server"
MAX_ATTEMPTS=3
ATTEMPT=1

while [ $ATTEMPT -le $MAX_ATTEMPTS ]; do
    cd ~/ros2_ws && source install/setup.bash
    ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_${ENV}_project.yaml \
        > >(ts '[%Y%m%d%H%M%S]' > "$LOG_DIR/localization_${ENV}.log") 2>&1 &
    echo "Wait while localization service is loading..."
    sleep 40

    count=$(grep -c "Please set the initial pose..." "$LOG_DIR/localization_${ENV}.log")
    if [ "$count" -ge 1 ]; then
        echo "Localization is running successfully. Proceeding..."
        break
    else
        echo "Attempt $ATTEMPT of $MAX_ATTEMPTS failed."
        ((ATTEMPT++))
        ps -ef | grep ros2 | grep -v grep | awk '{print $2}' | xargs -r kill -9
        sleep 3
    fi
done

if [ $ATTEMPT -gt $MAX_ATTEMPTS ]; then
    echo "Failed to run localization. Abort..."
    exit 1
fi

# ==== PATH PLANNING SERVER ====
echo
echo "Starting pathplanning service"
cd ~/ros2_ws && source install/setup.bash
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=${USE_SIM_TIME} \
  > >(grep -viE "drop|rviz" | ts '[%Y%m%d%H%M%S]' > "$LOG_DIR/pathplanner_${ENV}.log") 2>&1 &

echo "Wait while pathplanner service is loading..."
sleep 25
echo "Pathplanner is running successfully. Proceeding..."

# ==== Start ROS2 nodes ====
echo
echo "Starting all ROS2 nodes"

source ~/ros2_ws/install/setup.bash
ros2 launch warehouse_automation_pkg glocalization_action_server.launch.py env:=${ENV} \
    > >(ts '[%Y%m%d%H%M%S]' > "$LOG_DIR/warehouse_nodes_${ENV}.log") 2>&1 &

# ==== Clone webpage repository ====
echo
echo "Starting webpage"
cd ~/webpage_ws
# git pull
python3 -m http.server 7000 > >(ts '[%Y%m%d%H%M%S]' > "$LOG_DIR/webpage_${ENV}.log") 2>&1 &

instance_id=$(curl -s http://169.254.169.254/latest/meta-data/instance-id)
echo "https://${instance_id}.robotigniteacademy.com/${SLOT_PREFIX}/webpage/"
echo "Rosbridge address:"
echo "wss://${instance_id}.robotigniteacademy.com/${SLOT_PREFIX}/rosbridge/"

# ==== Run rosbridge server ====
if [ $ENV == "sim" ]; then
    echo
    echo "Launch rosbridge server"
    cd ~/ros2_ws
    source ~/ros2_ws/install/setup.bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
        > >(ts '[%Y%m%d%H%M%S]' >> "$LOG_DIR/webpage_${ENV}.log") 2>&1 &
    sleep 5
fi

echo "[!] All processes launched. Check logs in $LOG_DIR"

# ==== Start warehouse manager ====
cd ~/ros2_ws/src/warehouse_project/nav2_apps/scripts
./wh_manager.py --ros-args -p config_file:=../config/wh_manager_${ENV}.yaml
# ./wh_manager.py --ros-args -p config_file:=../config/wh_manager_sim.yaml
