from mbzirc_ign.bridge import Bridge, BridgeDirection


def camera_models():
    models = ['mbzirc_vga_camera',
              'mbzirc_hd_camera']
    return models


def rgbd_models():
    models = ['mbzirc_rgbd_camera']
    return models


def lidar_models():
    models = ['mbzirc_planar_lidar',
              'mbzirc_3d_lidar',
              ]
    return models


def slot_prefix(world_name, model_name, slot_idx):
    s = f'/world/{world_name}/model/{model_name}/model/sensor_{slot_idx}/link/sensor_link/sensor'
    return s


def image(world_name, model_name, slot_idx):
    prefix = slot_prefix(world_name, model_name, slot_idx)
    return Bridge(
        ign_topic=f'{prefix}/camera/image',
        ros_topic=f'slot{slot_idx}/image_raw',
        ign_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.IGN_TO_ROS)


def depth_image(world_name, model_name, slot_idx):
    prefix = slot_prefix(world_name, model_name, slot_idx)
    return Bridge(
        ign_topic=f'{prefix}/camera/depth_image',
        ros_topic=f'slot{slot_idx}/depth',
        ign_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.IGN_TO_ROS)


def camera_info(world_name, model_name, slot_idx):
    prefix = slot_prefix(world_name, model_name, slot_idx)
    return Bridge(
        ign_topic=f'{prefix}/camera/camera_info',
        ros_topic=f'slot{slot_idx}/camera_info',
        ign_type='ignition.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.IGN_TO_ROS)


def lidar_scan(world_name, model_name, slot_idx):
    prefix = slot_prefix(world_name, model_name, slot_idx)
    return Bridge(
        ign_topic=f'{prefix}/lidar/scan',
        ros_topic=f'slot{slot_idx}/scan',
        ign_type='ignition.msgs.LaserScan',
        ros_type='sensor_msgs/msg/LaserScan',
        direction=BridgeDirection.IGN_TO_ROS)


def lidar_points(world_name, model_name, slot_idx):
    prefix = slot_prefix(world_name, model_name, slot_idx)
    return Bridge(
        ign_topic=f'{prefix}/lidar/scan/points',
        ros_topic=f'slot{slot_idx}/points',
        ign_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.IGN_TO_ROS)


def camera_points(world_name, model_name, slot_idx):
    prefix = slot_prefix(world_name, model_name, slot_idx)
    return Bridge(
        ign_topic=f'{prefix}/camera/points',
        ros_topic=f'slot{slot_idx}/points',
        ign_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.IGN_TO_ROS)


def payload_bridges(world_name, model_name, payload, idx):
    bridges = []
    if payload in camera_models():
        bridges = [
            image(world_name, model_name, idx),
            camera_info(world_name, model_name, idx)
        ]
    elif payload in lidar_models():
        bridges = [
            lidar_scan(world_name, model_name, idx),
            lidar_points(world_name, model_name, idx)
        ]
    elif payload in rgbd_models():
        bridges = [
            image(world_name, model_name, idx),
            camera_info(world_name, model_name, idx),
            depth_image(world_name, model_name, idx),
            camera_points(world_name, model_name, idx),
        ]
    return bridges
