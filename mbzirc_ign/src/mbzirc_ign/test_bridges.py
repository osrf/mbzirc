import unittest

from mbzirc_ign import bridges
from mbzirc_ign import payload_bridges


class TestBridges(unittest.TestCase):

    world_name = 'foo'
    model_name = 'bar'
    link_name = 'baz'

    def prefix(self):
        return f'/world/{self.world_name}/model/{self.model_name}/link/{self.link_name}/sensor'

    def test_imu(self):
        bridge = bridges.imu(self.world_name, self.model_name, self.link_name)
        self.assertEqual(bridge.argument(),
                         f'{self.prefix()}/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU')

    def test_magnetometer(self):
        bridge = bridges.magnetometer(self.world_name, self.model_name, self.link_name)
        self.assertEqual(bridge.argument(),
                         f'{self.prefix()}/magnetometer/magnetometer'
                         '@sensor_msgs/msg/MagneticField[ignition.msgs.Magnetometer')

    def test_air_pressure(self):
        bridge = bridges.air_pressure(self.world_name, self.model_name, self.link_name)
        self.assertEqual(bridge.argument(),
                         f'{self.prefix()}/air_pressure/air_pressure'
                         '@sensor_msgs/msg/FluidPressure[ignition.msgs.FluidPressure')

    def test_pose(self):
        bridge = bridges.pose(self.model_name)
        self.assertEqual(bridge.argument(),
                         f'/model/{self.model_name}/pose'
                         '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V')
        bridge = bridges.pose_static(self.model_name)
        self.assertEqual(bridge.argument(),
                         f'/model/{self.model_name}/pose_static'
                         '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V')

    def test_cmd_vel(self):
        bridge = bridges.cmd_vel(self.model_name)
        self.assertEqual(bridge.argument(),
                         f'/model/{self.model_name}/cmd_vel'
                         '@geometry_msgs/msg/Twist]ignition.msgs.Twist')

    def test_fixed_wing(self):
        bridge = bridges.fixed_wing_flap(self.model_name, 'left')
        self.assertEqual(bridge.argument(),
                         f'/model/{self.model_name}/joint/left_flap_joint/cmd_pos'
                         '@std_msgs/msg/Float64]ignition.msgs.Double')

        bridge = bridges.fixed_wing_flap(self.model_name, 'right')
        self.assertEqual(bridge.argument(),
                         f'/model/{self.model_name}/joint/right_flap_joint/cmd_pos'
                         '@std_msgs/msg/Float64]ignition.msgs.Double')

        bridge = bridges.fixed_wing_prop(self.model_name)
        self.assertEqual(bridge.argument(),
                         f'/model/{self.model_name}/joint/propeller_joint/cmd_vel'
                         '@std_msgs/msg/Float64]ignition.msgs.Double')

    def test_usv(self):
        bridge = bridges.thrust(self.model_name, 'left')
        self.assertEqual(bridge.argument(),
                         f'/model/{self.model_name}/joint/left_engine_propeller_joint/cmd_thrust'
                         '@std_msgs/msg/Float64]ignition.msgs.Double')

        bridge = bridges.thrust(self.model_name, 'right')
        self.assertEqual(bridge.argument(),
                         f'/model/{self.model_name}/joint/right_engine_propeller_joint/cmd_thrust'
                         '@std_msgs/msg/Float64]ignition.msgs.Double')

        bridge = bridges.thrust_joint_pos(self.model_name, 'left')
        self.assertEqual(bridge.argument(),
                         f'{self.model_name}/left/thruster/joint/cmd_pos'
                         '@std_msgs/msg/Float64]ignition.msgs.Double')
        bridge = bridges.thrust_joint_pos(self.model_name, 'right')
        self.assertEqual(bridge.argument(),
                         f'{self.model_name}/right/thruster/joint/cmd_pos'
                         '@std_msgs/msg/Float64]ignition.msgs.Double')


class TestPayloadBridges(unittest.TestCase):
    world_name = 'foo'
    model_name = 'bar'
    link_name = 'baz'
    idx = 0

    def prefix(self):
        return (f'/world/{self.world_name}/model/{self.model_name}'
                f'/model/sensor_{self.idx}/link/sensor_link/sensor')

    def test_image(self):
        bridge = payload_bridges.image(self.world_name, self.model_name, self.idx)
        self.assertEqual(bridge.argument(),
                         f'{self.prefix()}/camera/image'
                         '@sensor_msgs/msg/Image[ignition.msgs.Image')

        bridge = payload_bridges.depth_image(self.world_name, self.model_name, self.idx)
        self.assertEqual(bridge.argument(),
                         f'{self.prefix()}/camera/depth_image'
                         '@sensor_msgs/msg/Image[ignition.msgs.Image')

    def test_camera_info(self):
        bridge = payload_bridges.camera_info(self.world_name, self.model_name, self.idx)
        self.assertEqual(bridge.argument(),
                         f'{self.prefix()}/camera/camera_info'
                         '@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo')

    def test_points(self):
        bridge = payload_bridges.lidar_points(self.world_name, self.model_name, self.idx)
        self.assertEqual(bridge.argument(),
                         f'{self.prefix()}/lidar/scan/points'
                         '@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked')

        bridge = payload_bridges.camera_points(self.world_name, self.model_name, self.idx)
        self.assertEqual(bridge.argument(),
                         f'{self.prefix()}/camera/points'
                         '@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked')

    def test_payload_bridges(self):
        bridges = payload_bridges.payload_bridges(
            self.world_name, self.model_name, 'mbzirc_vga_camera', self.idx)
        self.assertEqual(len(bridges), 2)

        bridges = payload_bridges.payload_bridges(
            self.world_name, self.model_name, 'mbzirc_hd_camera', self.idx)
        self.assertEqual(len(bridges), 2)

        bridges = payload_bridges.payload_bridges(
            self.world_name, self.model_name, 'mbzirc_planar_lidar', self.idx)
        self.assertEqual(len(bridges), 1)

        bridges = payload_bridges.payload_bridges(
            self.world_name, self.model_name, 'mbzirc_3d_lidar', self.idx)
        self.assertEqual(len(bridges), 1)

        bridges = payload_bridges.payload_bridges(
            self.world_name, self.model_name, 'mbzirc_rgbd_camera', self.idx)
        self.assertEqual(len(bridges), 4)
