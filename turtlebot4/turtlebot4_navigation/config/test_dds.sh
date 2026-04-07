#!/bin/bash

echo "chmod +x test_dds.sh"
echo "./test_dds.sh"

LOG_DIR=~/turtlebot4_ws/src/turtlebot4/turtlebot4_navigation/config/logs
mkdir -p $LOG_DIR

run_test () {
    RMW=$1
    LOG_FILE=$LOG_DIR/test_${RMW}.log

    echo "==============================" | tee $LOG_FILE
    echo "Testing RMW: $RMW" | tee -a $LOG_FILE
    echo "Time: $(date)" | tee -a $LOG_FILE
    echo "==============================" | tee -a $LOG_FILE

    export RMW_IMPLEMENTATION=$RMW

    echo "[INFO] Using RMW: $RMW_IMPLEMENTATION" | tee -a $LOG_FILE

    # Restart daemon (critical)
    ros2 daemon stop
    sleep 2
    ros2 daemon start
    sleep 2

    echo -e "\n[TEST] Topic list:" | tee -a $LOG_FILE
    ros2 topic list | tee -a $LOG_FILE

    echo -e "\n[TEST] Topic count:" | tee -a $LOG_FILE
    ros2 topic list | wc -l | tee -a $LOG_FILE

    echo -e "\n[TEST] /scan frequency (5s):" | tee -a $LOG_FILE
    timeout 5 ros2 topic hz /scan | tee -a $LOG_FILE

    echo -e "\n[TEST] Echo /odom once:" | tee -a $LOG_FILE
    timeout 3 ros2 topic echo /odom --once | tee -a $LOG_FILE

    echo -e "\n[TEST] Echo /tf once:" | tee -a $LOG_FILE
    timeout 3 ros2 topic echo /tf --once | tee -a $LOG_FILE

    echo -e "\n[TEST] Echo /map once:" | tee -a $LOG_FILE
    timeout 3 ros2 topic echo /map --once | tee -a $LOG_FILE

    echo -e "\n[TEST] Actions:" | tee -a $LOG_FILE
    ros2 action list | tee -a $LOG_FILE

    echo -e "\n[TEST] Nav2 services:" | tee -a $LOG_FILE
    ros2 service list | grep nav | tee -a $LOG_FILE

    echo -e "\n[RESULT] Completed test for $RMW\n" | tee -a $LOG_FILE
}

echo "Starting RMW Reliability Test..."

run_test rmw_fastrtps_cpp
sleep 5
echo "All tests complete."
echo "Logs saved in: $LOG_DIR"