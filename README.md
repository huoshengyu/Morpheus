# Morpheus Install Instructions

## Install Docker Desktop

Visit the Docker website and download the installer
https://www.docker.com/products/docker-desktop/

## Install Docker extension in VScode

Open VScode. You can do this by right-clicking on the desktop, opening a terminal, and entering `code`.  
On the left sidebar, find and select the Extensions tab.
Search for and install the following extensions: 1. Docker, 2. Remote Development.

## Clone Morpheus Git repository

In an appropriate directory, run the following to clone Morpheus:
```
mkdir morpheus_git
cd morpheus_git
git clone https://github.com/huoshengyu/Morpheus
```
Clone and update all submodules:
```
git submodule update --init --recursive
git submodule update --recursive
```

## Build the Docker image:

Check that the current directory is morpheus_git.  
To build the docker image on first run:
```
docker compose build
```
Check that the build completes without errors. Warnings are acceptable.  
You do not need to rebuild the docker image unless you make changes to ./Dockerfile.

## Start the Docker container:

To start the docker container on first run:
```
docker compose up
```
On future runs, repeat the above, or:
In VScode, on the left sidebar, find and select the Docker tab.
Right click on the morpheus docker container and select "Start".

## Check display settings:

To enable connection to the display:
```
xhost +
```
To list active displays:
```
w
```
If Rviz and/or Gazebo fail to open later, you may need to set the `DISPLAY` environment variable to one of the results from the above command.  
First check what `DISPLAY` is set to:
```
echo $DISPLAY
```
Then set `DISPLAY` to match a result from `w`. For example, if `w` shows that display `:1` exists:
```
export DISPLAY=:1
```

## Open a window in the Docker container:

In VScode, on the left sidebar, find and select the Docker tab.
Right click on the morpheus docker container and select "Attach Visual Studio Code".

## Build and launch:

Open a new terminal. In VSCode, on the top bar, click "Terminal" and select "New Terminal"
Build catkin workspace in the new terminal (must do on first run and whenever C code is changed):
```
catkin build
```
Open a_bot in simulation (with keyboard controls: wasd, q/e, ijkl, u/o. NOTE: To cancel/end this command, must first press esc, then ctrl+c.): 
```
roslaunch morpheus_teleop a_bot_fake_keyboard_control.launch
```
Open a new terminal.  
Start the collision tracking node (and display nearest collisions in Rviz):
```
roslaunch morpheus_collision collision.launch
```
Open a new terminal.  
Start the data recording node:
```
roslaunch morpheus_data data.launch
```
Open a new terminal.  
Start the goal haptics node:
```
roslaunch morpheus_teleop goal_haptics.launch
```


## Additional Installation Notes

[Morpheus Student Group Notion](https://www.notion.so/Morpheus-Student-Group-4d9b4c5947c34823b4dcc76b30b47f8b)