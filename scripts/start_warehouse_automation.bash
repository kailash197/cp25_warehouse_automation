#!/bin/bash
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
echo "[INFO] Updating package list..."
sudo apt update > $LOG_DIR/install_dependencies.log 2>&1
sudo apt install -y --no-install-recommends moreutils >> $LOG_DIR/install_dependencies.log 2>&1
echo "[INFO] Dependencies installed successfully"

# ==== Build projects ====
echo "Building projects..."
cd ~/ros2_ws && colcon build  > >(ts '[%Y%m%d%H%M%S]' > $LOG_DIR/ros2_ws_build.log) 2>&1
source ~/ros2_ws/install/setup.bash

# ==== Start Warehouse Automation ====

# ==== Find the environment: SIM or REAL ====
source /opt/ros/${ROS_DISTRO}/setup.bash
if [[ "$ENV" == "real" ]]; then
    echo "Welcome to the WAREHOUSE LAB!"
    USE_SIM_TIME=false
else
    echo "Starting SIMULATION environment..."
    USE_SIM_TIME=true

    MAX_ATTEMPTS=2
    ATTEMPT=1
    source ~/sim_ws/install/setup.bash

    while [ $ATTEMPT -le $MAX_ATTEMPTS ]; do
        ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml > >(ts '[%Y%m%d%H%M%S]' > "$LOG_DIR/gazebo.log") 2>&1 &
        echo "Wait while gazebo is loading..."
        sleep 50
        # check if gazebo is successful
        controller_count=$(grep -c "Sucessfully loaded controller" "$LOG_DIR/gazebo.log")
        if [ "$controller_count" -ge 3 ]; then
            echo "Gazebo is running successfully. Proceeding..."
            break
        else
            echo "Attempt $ATTEMPT of $MAX_ATTEMPTS failed: Found $controller_count/3 controllers."
            ((ATTEMPT++))
            ps -ef | grep gazebo | grep -v grep | awk '{print $2}' | xargs -r kill -9
            ps -ef | grep gz | grep -v grep | awk '{print $2}' | xargs -r kill -9
            sleep 3
        fi
    done
    if [ $ATTEMPT -gt $MAX_ATTEMPTS ]; then
        echo "Failed to start gazebo simulation. Abort..."
        exit 1
    fi
fi

# ==== LOCALIZATION SERVER ====
echo "Starting localization server"
MAX_ATTEMPTS=3
ATTEMPT=1

while [ $ATTEMPT -le $MAX_ATTEMPTS ]; do
    cd ~/ros2_ws
    ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_${ENV}.yaml > >(ts '[%Y%m%d%H%M%S]' > "$LOG_DIR/localization_${ENV}.log") 2>&1 &
    echo "Wait while localization service is loading..."
    sleep 25

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

sleep 5

# ==== PATH PLANNING SERVER ====
echo "Starting pathplanning service"
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=${USE_SIM_TIME} > >(ts '[%Y%m%d%H%M%S]' > "$LOG_DIR/pathplanner_${ENV}.log") 2>&1 &
sleep 25

# ==== Clone webpage repository ====
echo
cd ~/webpage_ws
# git pull
python3 -m http.server 7000 > >(ts '[%Y%m%d%H%M%S]' > "$LOG_DIR/webpage_${ENV}.log") 2>&1 &
echo "Started webpage address:"
instance_id=$(curl -s http://169.254.169.254/latest/meta-data/instance-id)
echo "https://${instance_id}.robotigniteacademy.com/${SLOT_PREFIX}/webpage/"
echo "Rosbridge address:"
echo "wss://${instance_id}.robotigniteacademy.com/${SLOT_PREFIX}/rosbridge/"

echo
cd ~/ros2_ws
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > >(ts '[%Y%m%d%H%M%S]' >> "$LOG_DIR/webpage_${ENV}.log") 2>&1 &
sleep 5

echo "[!] All processes launched. Check logs in $LOG_DIR"

# cd ros2_ws/src/warehouse_automation
# ./scripts/start_warehouse_automation.bash
# killwarehouseautomationprocesses

