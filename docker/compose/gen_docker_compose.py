#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import sys
import yaml

from jinja2 import Template


def parse_args():
    parser = argparse.ArgumentParser('Team vehicle configuration.')
    parser.add_argument('--config', dest='config_file', type=str,
                        help='Set config file')
    parser.add_argument('--out', dest='out_file', type=str,
                        help='Set output file')
    args = parser.parse_args()
    return args


def run_main():
    args = parse_args()

    if not args.config_file:
      print("Usage: python3 gen_docker_compose.py --config <config_file>")
      sys.exit(1)

    config_dict = None
    with open(args.config_file, "r") as stream:
        try:
            config_dict = yaml.safe_load(stream)
        except yaml.YAMLError as ex:
            print(ex)

    template = """
    services:
      sim:
        image: cloudsim_sim
        command: world:=coast config_file:=/home/developer/config/{{ config_file }} sim_mode:=sim
        networks:
          sim_net:
            ipv4_address: 172.28.1.1
        environment:
          - DISPLAY
          - QT_X11_NO_MITSHM=1
          - XAUTHORITY=/tmp/.docker.xauth
          - IGN_PARTITION=sim
          - IGN_IP=172.28.1.1
        privileged: true
        runtime: nvidia
        security_opt:
          - seccomp=unconfined
        volumes:
          - "/tmp/.docker.xauth:/tmp/.docker.xauth"
          - "/tmp/.X11-unix:/tmp/.X11-unix"
          - "/etc/localtime:/etc/localtime:ro"
          - "/dev/input:/dev/input"
          - "./:/home/developer/config/"
      {% for robot in config_dict -%}
      bridge{{ loop.index }}:
        image: cloudsim_bridge
        command: world:=coast config_file:=/home/developer/config/{{ config_file }} sim_mode:=bridge robot:={{ robot["model_name"] }}
        networks:
          relay_net{{ loop.index }}:
            ipv4_address: 172.{{29 + loop.index - 1 }}.1.1
          sim_net:
            ipv4_address: 172.28.1.{{ loop.index + 1 }}
        environment:
          - IGN_PARTITION=sim
          - IGN_IP=172.28.1.{{ loop.index + 1 }}
          - ROS_DOMAIN_ID={{ loop.index - 1 }}
        volumes:
          - "./:/home/developer/config/"
        depends_on:
          - "sim"
      solution{{ loop.index }}:
        image: mbzirc_sim
        stdin_open: true
        tty: true
        networks:
          relay_net{{ loop.index }}:
            ipv4_address: 172.{{ 29 + loop.index - 1 }}.1.2
        runtime: nvidia
        environment:
          - ROS_DOMAIN_ID={{ loop.index - 1 }}
        depends_on:
          - "bridge{{ loop.index }}"
      {% endfor %}
    networks:
      sim_net:
        ipam:
          driver: default
          config:
            - subnet: 172.28.0.0/16

      {% for robot in config_dict -%}
      relay_net{{ loop.index }}:
        internal: true
        ipam:
          driver: default
          config:
            - subnet: 172.{{ 29 + loop.index - 1 }}.0.0/16
      {% endfor %}
    """

    data = { "config_file": args.config_file, "config_dict": config_dict }
    j2_template = Template(template)
    out = j2_template.render(data)
    out_name = 'mbzirc_compose.yaml'
    if args.out_file:
        out_name = args.out_file
    f = open(out_name, 'w')
    f.write(out)
    f.close()
    print(f'Docker compose file written to: {out_name}')

if __name__ == '__main__':
    run_main()

