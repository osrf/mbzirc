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

        bridges = model.bridges('test_world_name')
        self.assertEqual(len(bridges), 6)

        [payload_bridges, payload_nodes] = model.payload_bridges('test_world_name')

        self.assertEqual(len(payload_bridges), 8)
        self.assertEqual(len(payload_nodes), 4)

    def test_single_fw_uav_config(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'single_fw_uav_config.yaml')
        with open(config, 'r') as stream:
            model = Model.FromConfig(stream)
        self.assertTrue(model.isUAV())

        model.generate()

        args = model.spawn_args()
        self.assertEqual(len(args), 18)

        bridges = model.bridges('test_world_name')
        self.assertEqual(len(bridges), 8)

        [payload_bridges, payload_nodes] = model.payload_bridges('test_world_name')

        self.assertEqual(len(payload_bridges), 4)
        self.assertEqual(len(payload_nodes), 1)

    def test_single_usv_config(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'single_usv_config.yaml')
        with open(config, 'r') as stream:
            model = Model.FromConfig(stream)

        self.assertTrue(model.isUSV())

        model.generate()

        args = model.spawn_args()
        self.assertEqual(len(args), 18)

        bridges = model.bridges('test_world_name')
        self.assertEqual(len(bridges), 7)

        [payload_bridges, payload_nodes] = model.payload_bridges('test_world_name')

        self.assertEqual(len(payload_bridges), 4)
        self.assertEqual(len(payload_nodes), 1)

    def test_multiple_config(self):
        config = os.path.join(get_package_share_directory('mbzirc_ign'),
                              'config', 'coast_config.yaml')
        with open(config, 'r') as stream:
            models = Model.FromConfig(stream)

        for model in models:
            model.generate()
            model.spawn_args()
