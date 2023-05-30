# ROS Services

A tool to create new ROS Services. Tool assumes have the QCR ROS Daemons installed and running.


## API

**lists**: lists all the ROS services, whether they are enabled/disabled, and their status.
**set-user**: sets the services to bring on boot to be based on this user's service configuration. If a configuration file doesn't yet exist for this user, it will be created.

## Variables & Arguments

The following variables and arguments are present throughout the files. The meaning of each is as follows:
- **COMMANDS_FILE** - full path to where the commands required to be run, based on a configuration file, are stored. Commands are generated via Python as it is much easier to handle YAML files within Python. Changing the default is  useful for testing.
    - Default: /tmp/ros-service-commands
- **CONFIG_FILE** - full path to a configuration file. The default path is based on the current user. Changing the default is  useful for testing.
    - Default: `/home/<current-user>/.qcr/ros-service-config.yml`
- **CONFIG_FILE_SYMLINK** - full path to the location of the symbolic link. The symbolic link points to the current configuration file in use. Changing the default is  useful for testing.
    - Default: `/opt/qcr/ros-service-config.yml`
- **DEFAULT_CONFIG_FILE** - the location of the default config file. Changing the default is useful for testing.
    - Default: `/opt/qcr/ros-service-config_default.yml`
- **SERVICE_PATH** - contains the path to where systemd service files are stored. Changing the default is useful for testing.
    - Default: `/etc/systemd/system/`
- **SERVICE_PREFIX** - contains a string prefixed to the generated service files.
    - Default: `ros`
- **SERVICE_USER** - contains a string appended to the generated service files prior to the '.service' file extension.
    - Default: `<current-user>`

## Implementation

- Configuration files are YAML (TBD)
- Default configuration file is stored in /opt/qcr/ros-services-config_default.yml
- User configuration file is stored in /home/USER/qcr/ros-services-config.yml
- Current configuration file used is set using a symlink /opt/qcr/ros-services-config.yml
- Process when set-user is run:
    - Disables current user services (yaml > python > bash)
    - Sets new configuration file (bash)
    - Generates any new service files (yaml > python > bash)
    - Enables new user services (bash)

Configuration File Structure:

```yaml
- service_name: <Service Name> will be prefixed with ros- and suffixed with -$USER
  description: service description
  parent: name of the parent service
  command: shell command
  enabled: True/False
```
