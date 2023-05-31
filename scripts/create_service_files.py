#!/usr/bin/env python
#
#
# Creates the service files based on a configuration file.

###############
### IMPORTS ###
###############

import argparse
from common import *
import os


############
### MAIN ###
############

if __name__ == "__main__":
    
    # Arguments
    parser = argparse.ArgumentParser(description="Generates service files based on a configuration yaml file.")
    parser.add_argument('-f', '--config_file', type=str, help="the path to the services configuration yaml file. Defaults to '/etc/qcr/services-config.yml'", default=CONFIG_FILE)
    parser.add_argument('-p', '--service_path', type=str, help="the path to where system services are located. Defaults to '/etc/systemd/system/'", default=SERVICES_PATH)
    parser.add_argument('-u', '--service_user', type=str, help="the service user which is applied as a suffix to generated service files.")

    args = parser.parse_args()
    
    # Get all current service files
    current_services = get_current_services(args.service_path)

    # Read service configuration file and check all services are valid
    services = open_config(args.config_file)
    valid_configuration_data(services)

    # Loop through services in configuration file
    for service in services:

      # Update/create service
      service_name = get_service_name(service, args.service_user)
      if service_exists(current_services, service, args.service_user):
        print("Service Already Exists: %s - Updating service if required"%(service_name))
        os.remove(os.path.join(args.service_path, service_name + '.service'))
      else:
        print("Creating Service: %s"%(get_service_name(service, args.service_user)))

      # Create service file
      write_service_file(args.service_path, service, args.service_user)