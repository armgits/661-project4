# 661 Project 4

MoveIt!

## Setting up the workspace

See the [template repository](https://github.com/TerraformersURC/ros-humble-workspace)
to learn more about the workspace and requirements to work with this project.

Clone this repository to the home directory, open in VSCode and re-open in container
using the devcontainer extension.

For the first time, after building the container, run the setup scripts included
in the `.devcontainer/` folder to install all the dependencies for the workspace.

Run these commands in the VSCode terminal for the first time.

```bash
./.devcontainer/setup.sh && source ~/.bashrc
```

```bash
./.devcontainer/project4_setup.sh
```

Then build the packages in workspace

```bash
colcon build --mixin release && source install/setup.bash
```

## Running the project

Running the project requires running commands to execute two launch files in
seperate terminals.

Load the robot in RViz.

```bash
ros2 launch package_119399002 loadrobot.launch.py
```

Run the node to move the robot.

```bash
ros2 launch package_119399002 picknplace.launch.py
```
