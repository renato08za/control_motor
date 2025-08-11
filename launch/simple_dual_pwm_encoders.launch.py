from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='control_motor', executable='simple_dual_pwm_encoders', name='simple_dual_pwm_encoders',
             parameters=[{
                 'pwm1':13,'in1_1':5,'in2_1':6,
                 'pwm2':19,'in1_2':16,'in2_2':26,
                 'pwm_freq_hz':1000.0,
                 'enca1':17,'encb1':18,
                 'enca2':20,'encb2':21,
                 'invert_left':False,'invert_right':False,
                 'pullup':True,'glitch_ns':20000,
                 'publish_rate_hz':50.0
             }])
    ])
