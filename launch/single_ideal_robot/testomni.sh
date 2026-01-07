#!/usr/bin/env bash
set -euo pipefail

# Utility function to publish initial pose
publish_initial_pose() {
  local x=$1 y=$2 theta=$3
  sec=$(date +%s)
  nsec=$(date +%N)
  ros2 topic pub --once /robot0/initialpose amrl_msgs/msg/Localization2DMsg \
  "{header: {stamp: {sec: ${sec}, nanosec: ${nsec}}, frame_id: ''},
    pose: {x: ${x}, y: ${y}, theta: ${theta}},
    map: 'UT_Campus'}"
}

# Utility function to publish goal
publish_goal() {
  local x=$1 y=$2 z=$3 ox=$4 oy=$5 oz=$6 ow=$7
  sec=$(date +%s)
  nsec=$(date +%N)
  ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {stamp: {sec: ${sec}, nanosec: ${nsec}}, frame_id: 'map'},
    pose: {position: {x: ${x}, y: ${y}, z: ${z}},
           orientation: {x: ${ox}, y: ${oy}, z: ${oz}, w: ${ow}}}}"
}

# Utility function to wait with progress
wait_seconds() {
  local seconds=$1
  for i in $(seq 1 $seconds); do
    echo "waiting ${i} sec"
    sleep 1
  done
}

# Test 1
publish_initial_pose -19.524198532104492 16.285781860351562 -0.8567056059837341
wait_seconds 5
publish_goal -14.747281074523926 18.396955490112305 0.0 0.0 0.0 -0.6257274138343413 0.7800417960444087

# Ask user if they want to proceed to test 2
echo "Test 1 completed. Proceed to test 2? (y/n)"
read -r response
if [[ ! "$response" =~ ^[Yy]$ ]]; then
  echo "Exiting after test 1."
  exit 0
fi

echo "Starting test 2..."

# Test 2
publish_initial_pose -15.413460731506348 22.95758056640625 -1.5707963705062866
wait_seconds 5
publish_goal -13.524113655090332 20.563159942626953 0.0 0.0 0.0 -0.983953542119451 0.17842484958823943

# Ask user if they want to proceed to test 3
echo "Test 2 completed. Proceed to test 3? (y/n)"
read -r response
if [[ ! "$response" =~ ^[Yy]$ ]]; then
  echo "Exiting after test 2."
  exit 0
fi

echo "Starting test 3..."

# Test 3
publish_initial_pose -13.565356254577637 22.34521484375 -1.176005244255066
wait_seconds 5
publish_goal -12.772148132324219 20.227319717407227 0.0 0.0 0.0 -0.79053082384579 0.6124222534736116

# Ask user if they want to proceed to test 4
echo "Test 3 completed. Proceed to test 4? (y/n)"
read -r response
if [[ ! "$response" =~ ^[Yy]$ ]]; then
  echo "Exiting after test 3."
  exit 0
fi

echo "Starting test 4..."

# Test 4
publish_initial_pose -5.556819915771484 22.864652633666992 -1.5120404958724976
wait_seconds 5
publish_goal -4.624310493469238 19.606487274169922 0.0 0.0 0.0 -0.7496781783634315 0.6618025603499029

# Ask user if they want to proceed to test 5
echo "Test 4 completed. Proceed to test 5? (y/n)"
read -r response
if [[ ! "$response" =~ ^[Yy]$ ]]; then
  echo "Exiting after test 4."
  exit 0
fi

echo "Starting test 5..."

# Test 5
publish_initial_pose -13.826395034790039 19.141769409179688 0.34877100586891174
wait_seconds 5
publish_goal -10.878838539123535 22.14983367919922 0.0 0.0 0.0 -0.5547002116846947 0.832050284031533
