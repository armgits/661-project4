# 661 Project 4

Work in progress...

## Setting up

### Requirements

Setting up the workspace at least requires Linux, Docker, Git and VSCode on your
computer.

Refer the [Docker documentation](https://docs.docker.com/engine/install/) for
installing Docker engine. Choose to install Docker Engine with CLI over
Docker Desktop.

> **Note:** Check the customization section to learn more about the ways to
> make the workspace tailor-made for your project.

### Cloning the repository

Since this is a template repository, you can create your own repository with the
current layout as a starting point if you wish to.

Otherwise, cloning the repository to your home directory is sufficient to get
started.

### Get started with development

Make sure to install Dev containers and Docker VSCode extensions on host.

Open the folder of workspace in VSCode and click on the remote icon at the
bottom-left corner of the screen. Choose **Reopen in Container** to open the
folder in the container. That option also could appear in the pop-up at the
bottom right if VSCode detects the `.devcontainer/` folder.

> **Note:** Building the container takes some time for the first time.

If the container builds and runs cleanly, the development environment is
successfully setup!! Use the terminal in VSCode to do stuff.

> **Note:** Make sure the type of terminal is setup as **bash** and set the
> default profile in the drop down menu beside the + icon.

### Customizing the workspace

#### GPU Access

Docker can provide GPU acceleration to applications running inside the container
if setup correctly. The steps to modify depends on the specific GPU.

The repository by default is set up to support Intel iGPUs (not tested yet) or no
GPU at all to maximize compatibility.

##### For NVIDIA GPUs

- Install the container toolkit on host by following the steps in [documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

- In `devcontainer.json`, set the build target to `nvidia` and comment/uncomment
  the appropriate run flags by following the comments.

##### For AMD GPUs

- Install ROCm software on host by following the steps in [documentation](https://rocm.docs.amd.com/projects/install-on-linux/en/latest/tutorial/quick-start.html#rocm-amdgpu-quick).
  It is preferable to use the Native package manager method.

- In `devcontainer.json` set the build target to `amd`, uncomment the build
  arguments and comment/uncomment the run arguments by following the comments.

#### Setting a custom workspace name and directory

This requires modifying the `devcontainer.json` file in the `.devcontainer/`
directory.

The `workspaceMount`, `workspaceFolder` and the `WORKSPACE` variable in the
`containerEnv` section deal with the way Docker mounts the workspace directory
from host onto the container.

#### VSCode Extensions

This requires modifying the `devcontainer.json` in the `.devcontainer/` directory.

Extensions inside the container are seperate from host. Extensions from the host
are accessible inside the container, same isn't true in reverse. So a desired
set of add-ons specific to the workspace can be added in the container!

Modify the `customizations` field inside `devcontainer.json`.

#### Changing the username

This requires modifying both `devcontainer.json` and `Dockerfile` in the
`.devcontainer/` directory.

Make sure the `remoteUser` field (in `devconatiner.json`) and `USERNAME` argument
(in both `devcontainer.json` and `Dockerfile`) match to your desired username.

In addition, paths mentioned across both the files that contain user directory
might need to be updated.

My suggestion: Simply find and replace the username keyword in both the files.

### Credits

This [article](https://www.allisonthackston.com/articles/vscode-docker-ros2) by Allison Thackston.
