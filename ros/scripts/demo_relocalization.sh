#!/bin/bash
# Demo script for KISS-Matcher relocalization functionality

echo "KISS-Matcher Relocalization Demo"
echo "================================="

# Check if required parameters are provided
if [ -z "$1" ]; then
    echo "Usage: $0 <path_to_map.pcd> [scan_topic]"
    echo "Example: $0 /path/to/map.pcd /velodyne_points"
    exit 1
fi

MAP_FILE="$1"
SCAN_TOPIC="${2:-/velodyne_points}"

# Check if map file exists
if [ ! -f "$MAP_FILE" ]; then
    echo "Error: Map file '$MAP_FILE' not found!"
    exit 1
fi

echo "Map file: $MAP_FILE"
echo "Scan topic: $SCAN_TOPIC"
echo ""

# Function to cleanup background processes
cleanup() {
    echo "Cleaning up..."
    kill $RELOCALIZATION_PID 2>/dev/null
    kill $RVIZ_PID 2>/dev/null
    exit 0
}

# Set trap for cleanup
trap cleanup SIGINT SIGTERM

# Launch relocalization node
echo "1. Starting relocalization node..."
ros2 launch kiss_matcher_ros relocalization.launch.py \
    map_file:="$MAP_FILE" \
    scan_topic:="$SCAN_TOPIC" \
    use_rviz:=true &
RELOCALIZATION_PID=$!

# Wait for node to start
sleep 3

echo "2. Relocalization node started (PID: $RELOCALIZATION_PID)"
echo ""

# Check if services are available
echo "3. Checking services..."
if ros2 service list | grep -q "relocalize"; then
    echo "✓ Relocalize service available"
else
    echo "✗ Relocalize service not available"
fi

if ros2 service list | grep -q "reset_localization"; then
    echo "✓ Reset service available"
else
    echo "✗ Reset service not available"
fi
echo ""

# Interactive menu
while true; do
    echo "Choose an action:"
    echo "1) Trigger global relocalization"
    echo "2) Trigger relocalization with pose hint"
    echo "3) Reset localization"
    echo "4) Run test script"
    echo "5) Show current status"
    echo "q) Quit"
    echo ""
    read -p "Enter choice: " choice

    case $choice in
        1)
            echo "Triggering global relocalization..."
            ros2 service call /relocalize kiss_matcher_ros/srv/Relocalize "{}"
            ;;
        2)
            read -p "Enter X coordinate: " x
            read -p "Enter Y coordinate: " y
            read -p "Enter Z coordinate: " z
            echo "Triggering relocalization with hint ($x, $y, $z)..."
            ros2 service call /relocalize kiss_matcher_ros/srv/Relocalize \
                "{pose_hint: {header: {frame_id: 'map'}, pose: {pose: {position: {x: $x, y: $y, z: $z}, orientation: {w: 1.0}}}}}"
            ;;
        3)
            echo "Resetting localization..."
            ros2 service call /reset_localization std_srvs/srv/Trigger
            ;;
        4)
            echo "Running test script..."
            python3 "$(ros2 pkg prefix kiss_matcher_ros)/share/kiss_matcher_ros/scripts/test_relocalization.py"
            ;;
        5)
            echo "Current status:"
            echo "Topics:"
            ros2 topic list | grep -E "(velodyne|aligned|tf)" | head -5
            echo "Services:"
            ros2 service list | grep -E "(relocalize|reset)" 
            echo "Nodes:"
            ros2 node list | grep relocalization
            ;;
        q|Q)
            echo "Exiting..."
            cleanup
            ;;
        *)
            echo "Invalid choice. Please try again."
            ;;
    esac
    echo ""
done