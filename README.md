# Morpheus Install Instructions

## 1. Install Docker

### Windows: Install Docker Desktop

Visit the Docker website and download the installer
https://www.docker.com/products/docker-desktop/

### Linux: Install Docker via apt-get

```
sudo apt-get update
sudo apt-get install docker
```

## 2. Install Docker extension in VScode

If not already installed, download VSCode from `https://code.visualstudio.com/download`.

a. Open VScode. You can do this quickly by right-clicking on the desktop, opening a terminal, and entering `code`.  
b. On the left sidebar, find and select the Extensions tab.
c. Search for and install the following extensions: 1. Docker, 2. Remote Development.

## 3. Clone Morpheus Git repository

a. In a terminal, navigate to an appropriate directory and run the following to clone Morpheus:
```
mkdir morpheus_git
cd morpheus_git
git clone https://github.com/huoshengyu/Morpheus
```
b. Clone and update all submodules:
```
git submodule update --init --recursive
git submodule update --recursive
```

## 4. Build the Docker image:

a. In a terminal, check that the current directory is morpheus_git and build the Docker image:  
```
docker compose build
```
b. Check that the build completes without errors (red messages). Warnings (yellow messages) are acceptable.  

You do not need to rebuild the docker image unless you make changes to the Dockerfile.

## 5. Start the Docker container:

a. To start the docker container on first run:
```
docker compose up
```
b. On future runs, repeat the above, or:  
i. In VScode, on the left sidebar, find and select the Docker tab.  
ii. Right click on the Morpheus Docker container and select "Start".

## 6. Open a VSCode window in the Docker container:

In VScode, on the left sidebar: 
a. Find and select the Docker tab.
b. Right click on the morpheus docker container and select "Attach Visual Studio Code". You should now have two VSCode windows: One outside the Docker container and one inside the Docker container. The one inside the Docker container will have a blue bar on the bottom left showing the container name.

## 7. Check display settings:

To allow RVIZ and/or Gazebo to open:
a. Outside the Docker container, allow connection to the display:
```
xhost +
```
b. You may need to manually set the `DISPLAY` environment variable to one of the active displays if it did not set correctly.  
i. Outside the Docker container, list active displays:
```
w
```
ii. Inside the Docker container, check what `DISPLAY` is set to:
```
echo $DISPLAY
```
iii. Set `DISPLAY` to match a result from `w`. For example, if `w` shows that display `:1` exists:
```
export DISPLAY=:1
```

## 8. Build:

a. Open a new terminal (In VSCode, on the top bar, click "Terminal" and select "New Terminal").  
b. Build catkin workspace in the new terminal (must do on first run and whenever C code is changed):
```
catkin build
```

# Morpheus Hardware Launch Instructions

## 1. Power on the computer

NOTE: Turn on the computer first to ensure all connections to it are powered on. If the computer boots into the wrong OS, hold F12 on startup to access the boot menu.

## 2. Power on the controller  

IRSS (miniature robot) controller:
a. Plug power cable into power outlet
b. Plug USB cable into computer's USB port
c. Flip the power switch ON (dot side pressed down)

Xbox/Playstation controller:
a. Plug into computer's USB port.

## 3. Power on the robot   

UR robots:  
a. Press the power button on the Universal Robots Teach Pendant (the small monitor attached to the UR's control box). Wait for the teach pendant to start up.   
b. Open the boot menu (red dot in the bottom left corner) and start the robot itself.

Trossen robots:
a. Press the power button on the side of the robot

## 4. Power on the gripper 

OnRobot RG2-FT gripper:  
a. Plug power cable into power outlet
b. Plug green control cable into OnRobot gripper
c. Plug ethernet cable into ethernet adapter (which is also plugged in to the computer and robot control box)

Robotiq 2f-85 gripper:
a. Powered automatically by the robot. You may need to launch tool_communication.py to use the IRSS (but this should run as part of the launch files).

## 5. UR5e: Set the robot control mode

If using IRSS controller:
a. Set the robot to remote control. The button is near the top right of the Teach Pendant's screen.

If using Xbox/Playstation controller:
a. Set the robot to local control. The button is near the top right of the Teach Pendant's screen.
b. On the Programs tab, load and start ros_control. You may need to perform this step after the Morpheus bringup launch file is running.

# Morpheus Software Launch Instructions

## 1. Windows: Launch Docker

Open Docker Desktop and allow ~1 minute to start up.

This step is not necessary on Linux.

## 2. Open VSCode

Open a new terminal and enter `code`.

## 3. Start the Docker container

In VSCode, on the left sidebar:
a. Select the Docker extension tab
b. Right click on the Morpheus Docker container and select "Attach Visual Studio Code"

## 4. Launch Morpheus:

### All at once:

Inside the Docker container:
```
roslaunch morpheus_main a_bot_main.launch
```
NOTE: Main currently does not launch the spawner node (to avoid race condition with the collision node) or the data collection node (to avoid unwanted file creation).

### Part by part:

Inside the Docker container:
a. Open a_bot in simulation (with keyboard controls: wasd, q/e, ijkl, u/o. NOTE: To cancel/end this command, must first press esc, then ctrl+c.): 
```
roslaunch morpheus_teleop a_bot_fake_keyboard_control.launch
```
b. Open a new terminal. Start the collision tracking node (and display nearest collisions in Rviz):
```
roslaunch morpheus_collision collision.launch
```
c. Open a new terminal. Start the data recording node:
```
roslaunch morpheus_data data.launch
```
d. Open a new terminal. Start the goal haptics node:
```
roslaunch morpheus_teleop goal_haptics.launch
```

## 5. Launch the IRSS

Inside the Docker container:
a. Check that the robot is near the home position and not near any possibe collisions. Launching the IRSS will cause the robot to move to the home position.
b. Open a new terminal. Start the IRSS node:
```
./gello_software/run_gello_onrobot.sh
```
c. Check the joint angles. The IRSS will not start unless the joint angles of the controllers also match the home position.

# Additional Installation Notes

[Morpheus Student Group Notion](https://www.notion.so/Morpheus-Student-Group-4d9b4c5947c34823b4dcc76b30b47f8b)