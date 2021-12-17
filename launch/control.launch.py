import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    publish_position=Node(
        package = 'ground_control',
        name = 'publish_position_node',
        executable = 'publish_position_node',
        output="screen",
        emulate_tty=True
    )
    
    gpio=Node(
        package = 'ground_control',
        name = 'gpio_node',
        executable = 'gpio_node',
        parameters=[
            {'writeable_gpio_pins'   : [17, 27, 22]},
            {'readable_gpio_pins'    : [5, 6]}
        ],
        output="screen",
        emulate_tty=True
    )
    
    ground_control=Node(
        package = 'ground_control',
        name = 'ground_control_node',
        executable = 'ground_control_node',
        parameters=[    
            {'holddown'              : 3}
        ],
        output="screen",
        emulate_tty=True
    )  
    
    ld.add_action(gpio)
    # ld.add_action(publish_position)
    ld.add_action(ground_control)

    return ld
