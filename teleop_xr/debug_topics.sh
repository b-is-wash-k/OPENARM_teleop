#!/usr/bin/env bash

topics=$(
ros2 topic list
)

for t in $topics; do
    echo ""
    echo "================================="
    echo "TOPIC: $t"
    echo "================================="

    echo "[TYPE]"
    ros2 topic type $t

    echo ""
    echo "[INFO]"
    ros2 topic info $t

    echo ""
    echo "[ECHO --ONCE]"
    timeout 3 ros2 topic echo --once $t

    echo ""
done
