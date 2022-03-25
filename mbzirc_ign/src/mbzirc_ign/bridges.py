from mbzirc_ign.bridge import Bridge, BridgeDirection


def prefix(world_name, model_name, link_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'


def imu(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/imu_sensor/imu',
        ros_topic='imu/data',
        ign_type='ignition.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.IGN_TO_ROS)


def magnetometer(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/magnetometer/magnetometer',
        ros_topic='magnetic_field',
        ign_type='ignition.msgs.Magnetometer',
        ros_type='sensor_msgs/msg/MagneticField',
        direction=BridgeDirection.IGN_TO_ROS)


def air_pressure(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/air_pressure/air_pressure',
        ros_topic='air_pressure',
        ign_type='ignition.msgs.FluidPressure',
        ros_type='sensor_msgs/msg/FluidPressure',
        direction=BridgeDirection.IGN_TO_ROS)


def pose(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/pose',
        ros_topic='pose',
        ign_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.IGN_TO_ROS)


def pose_static(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/pose_static',
        ros_topic='pose_static',
        ign_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.IGN_TO_ROS)


def cmd_vel(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/cmd_vel',
        ros_topic=f'cmd_vel',
        ign_type='ignition.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_IGN)


def fixed_wing_flap(model_name, side):
    return Bridge(
        ign_topic=f'/model/{model_name}/joint/{side}_flap_joint/cmd_pos',
        ros_topic=f'cmd/{side}_flap',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def fixed_wing_prop(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/joint/propeller_joint/cmd_vel',
        ros_topic='cmd/motor_speed',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def thrust(model_name, side):
    return Bridge(
        ign_topic=f'/model/{model_name}/joint/{side}_engine_propeller_joint/cmd_thrust',
        ros_topic=f'{side}/thrust/cmd_thrust',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def thrust_joint_pos(model_name, side):
    # ROS naming policy indicates that first character of a name must be an alpha
    # character. In the case below, the ign topic has the joint index 0 as the
    # first char so the following topics fail to be created on the ROS end
    # left_joint_topic = '/model/' + model_name + '/joint/left_chasis_engine_joint/0/cmd_pos'
    # right_joint_topic = '/model/' + model_name + '/joint/right_chasis_engine_joint/0/cmd_pos'
    # For now, use erb to generate unique topic names in model.sdf.erb
    return Bridge(
        ign_topic=f'{model_name}/{side}/thruster/joint/cmd_pos',
        ros_topic=f'{side}/thrust/joint/cmd_pos',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)
