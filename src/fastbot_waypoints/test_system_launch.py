#!/usr/bin/env python3
import os
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # Ortam değişkenleri ile özelleştirilebilir
    pkg_name       = os.getenv('PKG_NAME', 'fastbot_waypoints')
    server_exec    = os.getenv('SERVER_EXEC', 'fastbot_action_server')   # ros2 run fastbot_waypoints fastbot_action_server
    action_name    = os.getenv('ACTION_NAME', '/fastbot_as')
    goal_x         = os.getenv('GOAL_X', '1.49999')
    goal_y         = os.getenv('GOAL_Y', '2.255136')
    expect_fail    = os.getenv('EXPECT_FAILURE', 'False')
    timeout_sec    = os.getenv('TIMEOUT_SEC', '40')
    cmd_vel_topic  = os.getenv('CMD_VEL_TOPIC', 'fastbot/cmd_vel')
    odom_topic     = os.getenv('ODOM_TOPIC', '/fastbot/odom')
    yaw_precision  = os.getenv('YAW_PRECISION', f"{3.1415926535/90.0}")  # ~2 deg
    dist_precision = os.getenv('DIST_PRECISION', '0.10')                 # 10 cm default (kodda 0.10)

    # Server node (doğrudan executable olarak)
    server_node = Node(
        package=pkg_name,
        executable=server_exec,
        name='fastbot_action_server',
        output='screen',
        parameters=[{
            'cmd_vel_topic': cmd_vel_topic,
            'odom_topic': odom_topic,
            'yaw_precision': float(yaw_precision),
            'dist_precision': float(dist_precision),
        }]
    )

    this_dir = Path(__file__).resolve().parent
    tester_path = str(this_dir / 'tester_node.py')

    tester = ExecuteProcess(
        cmd=[
            tester_path,
            '--action-name', action_name,
            '--goal-x', goal_x,
            '--goal-y', goal_y,
            '--cmd-vel-topic', cmd_vel_topic,
            '-e', expect_fail,
            '--timeout', timeout_sec
        ],
        name='tester_node',
        output='screen'
    )

    # Tester bittiğinde tüm sistemi kapat
    shutdown_on_tester_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=tester,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        server_node,
        tester,
        shutdown_on_tester_exit
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return 0 if ls.run() == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
