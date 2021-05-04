# Romi autonomous code for Frantic fetch

Our auto code uses Ramsete commands and vision. 
The Romi executes a series of Ramsete commands to traverse the course. 
Before every Ramsete command, the robot's odometry position is updated with a pose from the vision system.
A camera is mounted on the top of the Romi and a grid of aruco markers is taped to the ceiling above the map. 
Three of the four cores on the Raspberry Pi installed on the Romi locates those vision markers, and the resulting position is sent to the computer running the robot code.  

The code loaded on the romi is located in the `vision` directory. 
The Command used to traverse the trajectory is located at `src/main/java/frc/robot/commands/FranticFetch.java`
