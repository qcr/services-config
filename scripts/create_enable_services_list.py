#!/usr/bin/env python
#
#
# Creates a list of services to enable/disable based on a configuration file.

###############
### IMPORTS ###
###############

import argparse
from common import *


############
### MAIN ###
############


if __name__ == "__main__":
    
    # Arguments
    parser = argparse.ArgumentParser(description="Generates service files based on a configuration yaml file.")
    parser.add_argument('-D', '--disable_all', help='set flag to disable all services regardless of the value in the configuration file', action='store_true')
    parser.add_argument('-c', '--command_file', type=str, help="the path to where the file containing the commands to run should be created. Defaults to '/tmp/ros-service-commands'", default=SERVICES_PATH)
    parser.add_argument('-f', '--config_file', type=str, help="the path to the services configuration yaml file. Defaults to '/opt/qcr/ros-services-config.yml'", default=CONFIG_FILE)
    parser.add_argument('-p', '--service_prefix', type=str, help="the prefix to append to services. Defaults to 'ros'", default=SERVICE_PREFIX)
    parser.add_argument('-u', '--service_user', type=str, help="the username, which is applied as a suffix to generated service files. Defaults to the current user.", default=USERNAME)
    
    args = parser.parse_args()

    # Read service configuration file and check all services are valid
    services = open_config(args.config_file)
    valid_configuration_data(services)

    # Create file of commands to run
    with open(args.command_file, 'w') as f:
        for service in services:
            
            # Get service name
            service_name = get_service_name(service, args.service_user, args.service_prefix)

            # Write values to file
            if service['enabled'] == False or args.disable_all:
                f.write('sudo systemctl stop %s\n'%(service_name))
                f.write('sudo systemctl disable %s\n'%(service_name))
            else:
                f.write('sudo systemctl enable %s\n'%(service_name))
                f.write('sudo systemctl start %s\n'%(service_name))



