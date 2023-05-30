#!/usr/bin/env python
#
#
# Common Python Functions

###############
### IMPORTS ###
###############

import os
from typing import List
import yaml

################
### DEFAULTS ###
################

CONFIG_FILE = "/opt/qcr/ros-services-config.yml"
SERVICES_PATH = "/etc/systemd/system/"
SERVICE_PREFIX = "ros"
USERNAME = os.environ['USERNAME']
SERVICE_COMMANDS_FILE="/tmp/ros-service-commands"

#################
### FUNCTIONS ###
#################

def get_service_name(service: dict, username: str, service_prefix: str='ros') -> str:
  return '%s-%s-%s'%(service_prefix.lower(), service['name'].lower(), username.lower())

def open_config(fpath: str) -> List:
  with open(fpath, 'r') as fstream:
    try:
      data = yaml.safe_load(fstream)
    except:
      print("Failed to read configuration file")
      exit(-1)
  return data


def valid_configuration_data(data: List) -> bool:
    for service in data:
        valid_service(service) # will exit if invalid service present
    return True

def valid_service(service: dict) -> bool:
  req_keys = ['name', 'parent', 'description', 'command', 'enabled']
  for key in req_keys:
    if key not in service.keys():
      print("Error! %s not included in service"%(key.capitalize()))
      exit(-1)
  return True

def get_current_ros_services(path: str, prefix: str='ros') -> List:
  return [f.split('.', maxsplit=1)[0] for f in os.listdir(path) if os.path.isfile(os.path.join(path, f)) and prefix in f]

def service_exists(current_services: List[str], service: dict, username: str, service_prefix: str='ros') -> bool:
  service_name = get_service_name(service, username, service_prefix)
  if service_name in current_services:
    return True
  return False

def write_service_file(services_path: str, service: dict, username: str, service_prefix: str='ros'):
  service_name = get_service_name(service, username, service_prefix)
  with open(os.path.join(services_path, service_name + ".service"), 'w') as f:

    # Write Unit Elements
    f.write('[Unit]\n')
    f.write('Description=%s\n'%(service['description']))
    f.write('Requires=%s\n'%(service['parent']))
    f.write('After=%s\n'%(service['parent']))

    # Write Service Elements
    f.write('\n\n[Service]\n')
    f.write('ExecStart=/bin/bash -c "%s"\n'%(service['command']))
    f.write('Restart=always\n')
    f.write('RestartSec=5\n')

    # Write Install Elements
    f.write('\n\n[Install]\n')
    f.write('WantedBy=%s\n'%(service['parent']))