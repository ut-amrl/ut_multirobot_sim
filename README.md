# cobot_simulator

Add  to path
`export ROS_PACKAGE_PATH=PATH_TO_DIRECTORY:PATH_TO_DIRECTORY/cobot_linux:$ROS_PACKAGE_PATH`

Move into the cobot_linux directory
`cd PATH_TO_DIRECTORY/cobot_linux`

Install required system packages
`./InstallPackages`

Build the messages directory
`make messages`

Build the program

`make`

Run

`./bin/cobot_simulator`

Publishes a laser scan message, odometry message, and visualization message
Markers and listens to a drive message.

Uses the first map in the file maps/atlas.txt.
