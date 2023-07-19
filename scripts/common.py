#!/usr/bin/env python
#
#
# Common Python Functions

###############
### IMPORTS ###
###############

import os
from typing import List, Tuple
import yaml

################
### DEFAULTS ###
################

CONFIG_FILE = "/etc/qcr/services-config.yml"
SERVICES_PATH = "/etc/systemd/system/"
# USERNAME = os.environ['USERNAME'] # This doesn't seem to be standard
SERVICE_COMMANDS_FILE="/tmp/service-commands"

#################
### FUNCTIONS ###
#################

def get_service_name(service: dict, username: str,) -> str:
  return '%s-%s'%( service['name'].lower().replace(' ', '-'), username.lower())


def open_config(fpath: str) -> List:
  with open(fpath, 'r') as fstream:
    try:
      data = yaml.safe_load(fstream)
    except:
      print("\n\033[0;31m[ERROR]\033[0m Failed to read configuration file")
      exit(1)

  # Check latest version
  if 'services' not in data.keys():
    print("\n\033[0;31m[ERROR]\033[0m You are using an old configuration file version. Please update your configuration file to the latest version.")
    exit(1)

  # Get catkin_ws, append_to_pythonpath, and source_env keys from data
  catkin_ws = None
  if 'catkin_ws' in data.keys():
    catkin_ws = data['catkin_ws']
  
  append_pypath = None
  if 'append_to_pythonpath' in data.keys():
    append_pypath = data['append_to_pythonpath']

  source_env = None
  if 'source_env' in data.keys():
    source_env = data['source_env']

  # Add catkin_ws and append_to_pythonpath to all services
  services = []
  for service in data['services']:
    service['catkin_ws'] = catkin_ws
    service['append_pypath'] = append_pypath
    service['source_env'] = source_env
    services.append(service)

  # Return
  return services 


def valid_configuration_data(data: List) -> bool:
  for service in data:
    valid_service(service) # will exit if invalid service present
  return True


def valid_service(service: dict) -> bool:
  req_keys = ['name', 'parent_service', 'description', 'command', 'run_on_boot', 'catkin_ws', 'append_pypath']
  for key in req_keys:
    if key not in service.keys():
      print("Error! %s not included in service"%(key.capitalize()))
      exit(-1)
  return True


def get_current_services(path: str) -> List:
  return [f.split('.', maxsplit=1)[0] for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]


def service_exists(current_services: List[str], service: dict, username: str) -> bool:
  service_name = get_service_name(service, username)
  if service_name in current_services:
    return True
  return False


def write_service_file(services_path: str, service: dict, username: str):
  service_name = get_service_name(service, username)
  with open(os.path.join(services_path, service_name + ".service"), 'w') as f:

    # Create coimmand prefix string containing Source Env, Python Path and Catkin Workspace
    cmd_prefix = ""
    if 'source_env' in service.keys() and service['source_env'] != "" and service['source_env'] != None:
      cmd_prefix += "source %s"%(service['source_env'])

    if 'append_pypath' in service.keys() and service['append_pypath'] != "" and service['append_pypath'] != None:
      if cmd_prefix != "":
        cmd_prefix += " && "
      cmd_prefix += "export PYTHONPATH=%s:$PYTHONPATH"%(service['append_pypath'])

    if 'catkin_ws' in service.keys() and service['catkin_ws'] != "" and service['catkin_ws'] != None:
      if cmd_prefix != "":
        cmd_prefix += " && "
      cmd_prefix += "source %s"%(service['catkin_ws'])

    # Write Unit Elements
    f.write('[Unit]\n')
    f.write('Description=%s\n'%(service['description']))
    f.write('Requires=%s\n'%(service['parent_service']))
    f.write('After=%s\n'%(service['parent_service']))

    # Write Service Elements
    f.write('\n[Service]\n')
    
    # pre-command
    if 'pre_command' in service.keys() and service['pre_command'] != "" and service['pre_command'] != None:
      f.write(create_command_string("ExecStartPre", cmd_prefix, service['pre_command']))

    # command
    f.write(create_command_string("ExecStart", cmd_prefix, service['command']))
    
    # post-command
    if 'post_command' in service.keys() and service['post_command'] != "" and service['post_command'] != None:
      f.write(create_command_string("ExecStartPost", cmd_prefix, service['post_command']))

    # restart
    if 'restart_on_failure' in service.keys() and service['restart_on_failure'] == True:
      f.write('Restart=always\n')
      if 'restart_after_n_seconds' in service.keys():
        f.write('RestartSec=%s\n'%(service['restart_after_n_seconds']))
      else:
        f.write('RestartSec=5\n')


    # Write Install Elements
    f.write('\n[Install]\n')
    f.write('WantedBy=%s\n'%(service['parent_service']))

  
def create_command_string(service_option: str, cmd_prefix: str, cmd: str) -> str:
  if cmd_prefix != "":
    return '%s=/bin/bash -c "%s && %s"\n'%(service_option, cmd_prefix, cmd)
  else:
    return '%s=/bin/bash -c "%s"\n'%(service_option, cmd)