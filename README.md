# Services Tool

The QCR Services tool is designed to allow users to easily manage custom start-up behaviour for a system. It has primarily been aimed at creating and managing custom ROS services on robot machines however, it could be utilised for non-ROS services. It can be used on both single and multi-user machines. The tool utilises [Systemd Services](https://www.freedesktop.org/software/systemd/man/systemd.service.html) under the hood. 

## Usage

Use the tool by calling *services* with the desired command and arguments.

Usage: `services <command> <arguments>`

Enter: `services --help` to get a list of up-to-date commands. However, commands include:

- **list**: lists all the services, whether they are enabled/disabled, and their status.
- **update**: sets the services to start on boot based on the user's service configuration.
- **new**: creates a new configuration file for the user using a default as the base.
- **current-user**: prints the username of the current configuration used.

## User Process

Your custom start-up process revolves around a services configuration file. This is a YAML file and its default location is `/home/<your-username>/.qcr/services-config.yml`. The process to create a custom start-up behaviour is:

1. Run `services new` - this will generate a new service configuration file in the default location by copying the system's default start-up configuration (created by the system administrator). If no system default exists it will be created by copying [service-config_default.yml](/service-config_default.yml) to the system default location.
2. Make the desired changes to your newly created service config file. 
3. Once you're happy run `service update` - this will create and update system service files, and enable/disable all appropriate services.

We recommend you utilise this tool in combination with the [QCR ROS-Services](https://github.com/qcr/ros-services) and set the parent to be one of the ROS meta services: *ros-sensors.service*; *ros-robot.service*; *ros-project.service*.

**Notes**:

- You will need to run `service update` if you make any changes to your service config file.
- You will need to run `service update` to change the start-up behaviour from a different user. In other words, restarting the computer and logging in does NOT automatically change the start-up behaviour to your configuration. 

### Configuration File Keys

The configuration file is a YAML file. The keys and their meanings are as follows:

```yaml
catkin_ws: (String, Optional) set the catkin workspace to source prior to running each service's command value. Set to the top most catkin workspace required across all services. Defaults to None.

append_to_pythonpath: (String, Optional) prepend a specified path(s) to the system's python path for each service. Defaults to None.

services: # A list of services
- name: (String, Required) the name of the service.
  description: (String, Required) a service description.
  parent_service: (String, Required) the parent service this service depends on.
  command: (String, Required) the shell command to be run.
  run_on_boot: (Boolean, Required) specifies if the service is run on boot. Defaults to True.
  restart_on_failure: (Boolean, Optional) specifies if the service is automatically restarted on failure. Defaults to True.
  restart_after_n_seconds: (Integer, Optional) the number of seconds to wait until attempting to restart the service. Defaults to 5.
```

**Notes**:

- Service names will be converted to lower case, and any spaces will be replaced with hyphens.
- Each service name must be unique within the configuration. Your username will be appended to the name to gaurantee uniqueness between users. In other words, different configurations can utilise the same service names however, all names must be unique within a single file.
- Prepend service names to help group them. For example, we recommend all services relating to ROS use the prefix `ros-`.

### Example Configuration

The following configuration file:

```yaml
catkin_ws: /home/qcr/qcr_agilex_payload_ws/devel/setup.bash

append_to_pythonpath: /home/qcr/.local/lib/python3.8/site-packages

services:
- name: ros-rs16-lidar
  description: Launch the RS16 LIDAR
  parent_service: ros-sensors.service
  command: roslaunch qcr_agilex_payload rs16.launch
  run_on_boot: True
  restart_on_failure: True
  restart_after_n_seconds: 5
```

Would result in the following service file with the name `ros-rs16-lidar-qcr` (assuming the username was *qcr*). The service would be enabled and brought up on start-up.

```bash
[Unit]
Description=Launch the RS16 LIDAR
Requires=ros-sensors.service
After=ros-sensors.service

[Service]
ExecStart=/bin/bash -c "export PYTHONPATH=/home/qcr/.local/lib/python3.8/site-packages:$PYTHONPATH && source /home/qcr/qcr_agilex_payload_ws/devel/setup.bash && roslaunch qcr_agilex_payload rs16.launch"
Restart=always
RestartSec=5

[Install]
WantedBy=ros-sensors.service

```


## Notes for Developers

The tool and commands have a testing flag for development. Use `--help` on each command to get more information.

### Variables & Arguments

See [scripts/common](scripts/common) for variables and their meaning.


### Implementation

- Configuration files are YAML
- Default configuration file is stored in `/etc/qcr/ros-services-config_default.yml`
- User configuration files are stored in `/home/<USER>/.qcr/ros-services-config.yml`
- Current configuration file used is set by copying a user's config to `/etc/qcr/ros-services-config.yml`
- Process when update is run:
    - Disables previous user services (yaml > python > bash)
    - Deletes existing service files for the previous user
    - Sets new configuration file (bash)
    - Generates service files for the current user (yaml > python > bash)
    - Enables current user services (bash)
