# Services Tool

The QCR Services tool is designed to allow users to easily manage custom start-up behaviour for a system. It has primarily been aimed at creating and managing custom ROS services however, it could be utilised for non-ROS services. It can be used on both single and multi-user machines. The tool utilises [Systemd Services](https://www.freedesktop.org/software/systemd/man/systemd.service.html) under the hood. 

## Usage

Use the tool by calling *services* with the desired command and arguments.

Usage: `services <command> <arguments>`

- **list**: lists all the ROS services, whether they are enabled/disabled, and their status.
- **update**: sets the services to start on boot based on the user's service configuration.
- **new**: creates a new configuration file for the user using a default as the base.
- **current**: prints the username of the current configuration used.

## User Process

Your custom start-up process revolves around a services configuration file. This is a YAML file and its default location is `/home/<your-username>/.qcr/services-config.yml`. The process to create a custom start-up behaviour is:

1. Run `services new` - this will generate a new service configuration file in the default location by copying the system's default start-up configuration (created by the system administrator).
2. Make the desired changes to your newly created service config file.
3. Once you're happy run `service update` - this will establish create and update system service files, and enable/disable all appropriate services.

**Notes**:

- You will need to run `service update` if you make any changes to your service config file.
- You will need to run `service update` to change the start-up behaviour from a different user. In other words, restarting the computer and logging in does NOT automatically change the start-up behaviour to your configuration. 

## Configuration File Keys

The configuration file is a YAML file. The keys and their meanings are as follows:

```yaml
- name: (String, Required) the name of the service.
  description: (String, Required) a service description.
  parent: (String, Required) the parent service this service depends on.
  catkwin_ws: (String, Optional) a catkin_ws space to source prior to running the command. Defaults to no workspace.
  command: (String, Required) the shell command to be run.
  enabled: (Boolean, Required) specifies if the service is enabled. Defaults to True.
  restart: (Boolean, Optional) specifies if the service is automatically restarted. Defaults to True.
  retart_after: (Integer, Optional) the number of seconds to wait until attempting to restart the service. Defaults to 5.
```

**Notes**:

- Service names will be converted to lower case, and any spaces will be replaced with hyphens.
- Each service name must be unique within the configuration. Your username will be appended to the name to gaurantee uniqueness between users. In other words, different configurations can utilise the same service names however, all names must be unique within a single file.
- Prepend service names to help group them. For example, we recommend all services relating to ROS use the prefix `ros-`.

### Example Configuration

The following configuration file:

```yaml
- name: ros-rs16-lidar
  description: Launch the RS16 LIDAR
  parent: ros-sensors.service
  catkwin_ws: /home/qcr/qcr_agilex_payload_ws/devel/setup.bash
  command: roslaunch qcr_agilex_payload rs16.launch
  enabled: True
  restart: True
  retart_after: 5
```

Would result in the following service file with the name `ros-rs16-lidar-qcr` (assuming the username was *qcr*). The service would be enabled and brought up on start-up.

```bash
[Unit]
Description=Launch the RS16 LIDAR
Requires=ros-sensors.service
After=ros-sensors.service

[Service]
ExecStart=/bin/bash -c "source /home/qcr/qcr_agilex_payload_ws/devel/setup.bash && roslaunch qcr_agilex_payload rs16.launch"
Restart=always
RestartSec=5

[Install]
WantedBy=ros-sensors.service

```


## Notes for Developers

### Variables & Arguments

The following variables and arguments are present throughout the files. The meaning of each is as follows:

- **COMMANDS_FILE** - full path to where the commands required to be run, based on a configuration file, are stored. Commands are generated via Python as it is much easier to handle YAML files within Python. Changing the default is  useful for testing.
    - Default: /tmp/ros-service-commands
- **CONFIG_FILE** - full path to a configuration file. The default path is based on the current user. Changing the default is  useful for testing.
    - Default: `/home/<current-user>/.qcr/ros-service-config.yml`
- **CONFIG_FILE_SYMLINK** - full path to the location of the symbolic link. The symbolic link points to the current configuration file in use. Changing the default is  useful for testing.
    - Default: `/etc/qcr/ros-service-config.yml`
- **CURRENT_CONFIG** - a text file storing the current user and config file location. Changing the default is useful for testing.
    - Default: `/opt/qcr/current-service-config-values`
CURRENT_CONFIG="/etc/qcr/current-service-config-values"
- **DEFAULT_CONFIG_FILE** - the location of the default config file. Changing the default is useful for testing.
    - Default: `/etc/qcr/ros-service-config_default.yml`
- **SERVICE_PATH** - contains the path to where systemd service files are stored. Changing the default is useful for testing.
    - Default: `/etc/systemd/system/`
- **SERVICE_USER** - contains a string appended to the generated service files prior to the '.service' file extension.
    - Default: `<current-user>`


### Implementation

- Configuration files are YAML
- Default configuration file is stored in `/etc/qcr/ros-services-config_default.yml`
- User configuration files are stored in `/home/<USER>/qcr/ros-services-config.yml`
- Current configuration file used is set using a symlink `/etc/qcr/ros-services-config.yml`
- Process when update is run:
    - Disables current user services (yaml > python > bash)
    - Sets new configuration file (bash)
    - Generates any new service files (yaml > python > bash)
    - Enables new user services (bash)
