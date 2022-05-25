import os
import unittest

from ament_index_python.packages import get_package_share_directory

from mbzirc_ign.model import Model


class TestModel(unittest.TestCase):

    def test_single_uav_config(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'single_uav_config.yaml')
        with open(config, 'r') as stream:
            model = Model.FromConfig(stream)
        self.assertTrue(model.isUAV())

        model.generate()

        args = model.spawn_args()
        self.assertEqual(len(args), 18)

        [bridges, nodes] = model.bridges('test_world_name')
        self.assertEqual(len(bridges), 8)
        self.assertEqual(len(nodes), 0)

        [payload_bridges, payload_nodes, launch] = model.payload_bridges('test_world_name')

        self.assertEqual(len(payload_bridges), 8)
        self.assertEqual(len(payload_nodes), 4)
        self.assertEqual(len(launch), 0)

    def test_single_uav_with_gripper_config(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'single_uav_with_gripper_config.yaml')
        with open(config, 'r') as stream:
            model = Model.FromConfig(stream)
        self.assertTrue(model.isUAV())

        model.generate()

        args = model.spawn_args()
        self.assertEqual(len(args), 18)

        [bridges, nodes] = model.bridges('test_world_name')
        self.assertEqual(len(bridges), 13)
        self.assertEqual(len(nodes), 0)

    def test_single_fw_uav_config(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'single_fw_uav_config.yaml')
        with open(config, 'r') as stream:
            model = Model.FromConfig(stream)
        self.assertTrue(model.isUAV())

        model.generate()

        args = model.spawn_args()
        self.assertEqual(len(args), 18)

        [bridges, nodes] = model.bridges('test_world_name')
        self.assertEqual(len(bridges), 7)
        self.assertEqual(len(nodes), 0)

        [payload_bridges, payload_nodes, launch] = model.payload_bridges('test_world_name')

        self.assertEqual(len(payload_bridges), 6)
        self.assertEqual(len(payload_nodes), 1)
        self.assertEqual(len(launch), 0)

    def test_single_usv_config(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'single_usv_config.yaml')
        with open(config, 'r') as stream:
            model = Model.FromConfig(stream)

        self.assertTrue(model.isUSV())

        model.generate()

        args = model.spawn_args()
        self.assertEqual(len(args), 18)

        [bridges, nodes] = model.bridges('test_world_name')
        self.assertEqual(len(bridges), 9)
        self.assertEqual(len(nodes), 0)

        [payload_bridges, payload_nodes, launch] = model.payload_bridges('test_world_name')

        self.assertEqual(len(payload_bridges), 6)
        self.assertEqual(len(payload_nodes), 1)
        self.assertEqual(len(launch), 0)

    def test_single_usv_with_arm_gripper_config(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'single_usv_with_arm_gripper_config.yaml')
        with open(config, 'r') as stream:
            model = Model.FromConfig(stream)

        self.assertTrue(model.isUSV())

        model.generate()

        args = model.spawn_args()
        self.assertEqual(len(args), 18)

        [bridges, nodes] = model.bridges('test_world_name')
        self.assertEqual(len(bridges), 24)
        self.assertEqual(len(nodes), 1)

    def test_coast(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'coast', 'hexrotor.yaml')
        with open(config, 'r') as stream:
            hexrotor = Model.FromConfig(stream)

        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'coast', 'quadrotor.yaml')
        with open(config, 'r') as stream:
            quadrotor = Model.FromConfig(stream)

        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'coast', 'usv.yaml')
        with open(config, 'r') as stream:
            usv = Model.FromConfig(stream)

        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'coast', 'config_team.yaml')
        with open(config, 'r') as stream:
            team = Model.FromConfig(stream)

        self.assertIsInstance(hexrotor, Model)
        self.assertIsInstance(quadrotor, Model)
        self.assertIsInstance(usv, Model)
        self.assertIsInstance(team, list)
        self.assertEqual(len(team), 3)
        self.assertIsInstance(team[0], Model)
