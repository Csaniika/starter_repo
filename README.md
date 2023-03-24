# Starter repo for lidar plugin development

This repository stores the development environment for a gazebo lidar plugin
that takes samples with a time delay, as opposed to taking them all
at the same time. This simulates the error of sampling when the robot
is moving.


## How to build

The point of the docker image is to provide a development environment that is
independent of the underlying machine. Work is meant to be done inside
the running container. The workspace is mounted as a volume, so changes
don't get lost.

Dependencies are tracked in the apt-dependencies.txt and pip-dependencies.txt files,
as well as installed "manually" from within the Dockerfile. Whenever any of these
dependencies change, the image has to be rebuilt.

```bash
docker compose build starter_repo
```

## Usage

Run an instance of the image.
```bash
docker compose run starter_repo
```

Inside, you can use git for version control. To share git users, the ~/.ssh directory
is mounted inside the container. To access the remote, you need to have you GitHub
access keys set up.

The environment is set up by sourcing the file named environment from the ~/.bashrc.
To change environment variables or setup, modify that file.

ROS commands can also be run inside the container. Run the stack with
```bash
ros2 launch gazebo_rotating_lidar_plugin minimal_bringup.launch.py
```

## Contributing
To contribute, fork this repo and open a pull request.
Please make sure to add or update documentation and add tests as appropriate.


## License
Enjoy Robotics Zrt. - All rights reserved.
