# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import codecs
import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import mbzirc_ign.bridges
import mbzirc_ign.payload_bridges

import yaml

FIXED_WING_UAVS = [
    'mbzirc_fixed_wing',
]

UAVS = [
    'mbzirc_fixed_wing',
    'mbzirc_hexrotor',
    'mbzirc_quadrotor'
]

USVS = [
    'usv',
]

WAVEFIELD_SIZE = {'simple_demo': 1000, 'coast': 6000}


class Model:

    def __init__(self, model_name, model_type, position):
        self.model_name = model_name
        self.model_type = model_type
        self.position = position
        self.battery_capacity = 0
        self.wavefield_size = 0
        self.payload = {}

    def isUAV(self):
        return self.model_type in UAVS

    def isFixedWingUAV(self):
        return self.model_type in FIXED_WING_UAVS

    def isUSV(self):
        return self.model_type in USVS

    def bridges(self, world_name):
        bridges = [
            # IMU
            mbzirc_ign.bridges.imu(world_name, self.model_name),
            # pose
            mbzirc_ign.bridges.pose(self.model_name),
            # pose static
            mbzirc_ign.bridges.pose_static(self.model_name),
        ]
        if self.isUAV():
            bridges.extend([
                # Magnetometer
                mbzirc_ign.bridges.magnetometer(world_name, self.model_name),
                # Air Pressure
                mbzirc_ign.bridges.air_pressure(world_name, self.model_name),
            ])
            if self.isFixedWingUAV():
                bridges.extend([
                    # Left Flap
                    mbzirc_ign.bridges.fixed_wing_flap(self.model_name, 'left'),
                    # Right Flap
                    mbzirc_ign.bridges.fixed_wing_flap(self.model_name, 'right'),
                    # Propeller
                    mbzirc_ign.bridges.fixed_wing_prop(self.model_name),
                ])
            else:
                bridges.extend([
                    # twist
                    mbzirc_ign.bridges.cmd_vel(self.model_name)
                ])
        elif self.isUSV():
            bridges.extend([
                # thrust cmd
                mbzirc_ign.bridges.thrust(self.model_name, 'left'),
                mbzirc_ign.bridges.thrust(self.model_name, 'right'),
                # thrust joint pos cmd
                mbzirc_ign.bridges.thrust_joint_pos(self.model_name, 'left'),
                mbzirc_ign.bridges.thrust_joint_pos(self.model_name, 'right'),
            ])
        return bridges

    def payload_bridges(self, world_name, payloads=None):
        bridges = []
        nodes = []

        if not payloads:
            payloads = self.payload
        for (idx, k) in enumerate(sorted(payloads.keys())):
            p = payloads[k]
            if not p['sensor'] or p['sensor'] == 'None' or p['sensor'] == '':
                continue
            bridges.extend(
                mbzirc_ign.payload_bridges.payload_bridges(
                    world_name, self.model_name, p['sensor'], idx))

            if p['sensor'] in ['mbzirc_vga_camera', 'mbzirc_hd_camera']:
                nodes.append(Node(
                    package='mbzirc_ros',
                    executable='optical_frame_publisher',
                    arguments=['1'],
                    remappings=[('input/image', f'slot{idx}/image_raw'),
                                ('output/image', f'slot{idx}/optical/image_raw'),
                                ('input/camera_info', f'slot{idx}/camera_info'),
                                ('output/camera_info', f'slot{idx}/optical/camera_info')]))
            elif p['sensor'] in ['mbzirc_rgbd_camera']:
                nodes.append(Node(
                    package='mbzirc_ros',
                    executable='optical_frame_publisher',
                    arguments=['1'],
                    remappings=[('input/image', f'slot{idx}/image_raw'),
                                ('output/image', f'slot{idx}/optical/image_raw'),
                                ('input/camera_info', f'slot{idx}/camera_info'),
                                ('output/camera_info', f'slot{idx}/optical/camera_info')]))
                nodes.append(Node(
                    package='mbzirc_ros',
                    executable='optical_frame_publisher',
                    arguments=['1'],
                    remappings=[('input/image', f'slot{idx}/depth'),
                                ('output/image', f'slot{idx}/optical/depth')]))
        return [bridges, nodes]

    def set_flight_time(self, flight_time):
        # UAV specific, sets flight time

        # calculate battery capacity from time
        # capacity (Ah) = flight time (in hours) * load (watts) / voltage
        # assume constant voltage for battery to keep things simple for now.
        self.battery_capacity = (float(flight_time) / 60) * 6.6 / 12.694

    def set_payload(self, payload):
        # UAV specific
        self.payload = payload

    def set_wavefield(self, world_name):
        if world_name not in WAVEFIELD_SIZE:
            print(f'Wavefield size not found for {world_name}')
        else:
            self.wavefield_size = WAVEFIELD_SIZE[world_name]

    def generate(self):
        # Generate SDF by executing ERB and populating templates
        template_file = os.path.join(
            get_package_share_directory('mbzirc_ign'),
            'models', self.model_type, 'model.sdf.erb')

        command = ['erb']
        command.append(f'name={self.model_name}')

        for (slot, payload) in self.payload.items():
            if payload['sensor'] and payload['sensor'] != 'None':
                command.append(f"{slot}={payload['sensor']}")
            if 'rpy' in payload:
                if type(payload['rpy']) is str:
                    r, p, y = payload['rpy'].split(' ')
                else:
                    r, p, y = payload['rpy']
                command.append(f'{slot}_pos={r} {p} {y}')

        if self.model_type in UAVS:
            if self.battery_capacity == 0:
                raise RuntimeError('Battery Capacity is zero, was flight_time set?')
            command.append(f'capacity={self.battery_capacity}')

        if self.model_type in USVS:
            command.append(f'wavefieldSize={self.wavefield_size}')

        command.append(template_file)
        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # evaluate error output to see if there were undefined variables
        # for the ERB process
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)

        stdout = process.communicate()[0]
        model_sdf = codecs.getdecoder('unicode_escape')(stdout)[0]
        print(command)

        return command, model_sdf

    def spawn_args(self, model_sdf=None):
        if not model_sdf:
            [command, model_sdf] = self.generate()

        return ['-string', model_sdf,
                '-name', self.model_name,
                '-allow_renaming', 'false',
                '-x', str(self.position[0]),
                '-y', str(self.position[1]),
                '-z', str(self.position[2]),
                '-R', str(self.position[3]),
                '-P', str(self.position[4]),
                '-Y', str(self.position[5])]

    @classmethod
    def FromConfig(cls, stream):
        # Generate a Model instance (or multiple instances) from a stream
        # Stream can be either a file input or string
        config = yaml.safe_load(stream)

        if type(config) == list:
            return cls._FromConfigList(config)
        elif type(config) == dict:
            return cls._FromConfigDict(config)

    @classmethod
    def _FromConfigList(cls, entries):
        # Parse an array of configurations
        ret = []
        for entry in entries:
            ret.append(cls._FromConfigDict(entry))
        return ret

    @classmethod
    def _FromConfigDict(cls, config):
        # Parse a single configuration
        if 'model_name' not in config:
            raise RuntimeError('Cannot construct model without model_name in config')
        if 'model_type' not in config:
            raise RuntimeError('Cannot construct model without model_type in config')

        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        if 'position' not in config:
            print('Position not found in config, defaulting to (0, 0, 0), (0, 0, 0)')
        else:
            if 'xyz' in config['position']:
                xyz = config['position']['xyz']
            if 'rpy' in config['position']:
                rpy = config['position']['rpy']
        model = cls(config['model_name'], config['model_type'], [*xyz, *rpy])

        if 'flight_time' in config:
            model.set_flight_time(config['flight_time'])

        if 'payload' in config:
            model.set_payload(config['payload'])

        return model
