import os
import yaml
import rclpy

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class WaypointExecutor(Node):

    def __init__(self):
        super().__init__('waypoint_executor')

        self.declare_parameter('mission_file', 'mission_1.yaml')

        mission_file = (
            self.get_parameter('mission_file')
            .get_parameter_value()
            .string_value
        )

        package_share = get_package_share_directory(
            'mission_formalism_evaluation'
        )

        mission_path = os.path.join(
            package_share,
            'missions',
            mission_file
        )

        self.get_logger().info(
            f'Loading mission: {mission_path}'
        )

        self.load_mission(mission_path)
        self.execute_mission()

    def load_mission(self, mission_path):

        with open(mission_path, 'r') as file:
            data = yaml.safe_load(file)

        self.mission_id = data['mission']['id']
        self.mission_name = data['mission']['name']
        self.tasks = data['tasks']

    def execute_mission(self):

        self.get_logger().info(
            f'Mission ID: {self.mission_id}'
        )

        self.get_logger().info(
            f'Mission Name: {self.mission_name}'
        )

        self.get_logger().info(
            f'Mission Items: {len(self.tasks)}'
        )

        for index, task in enumerate(
            self.tasks,
            start=1
        ):

            task_type = task['type']

            if task_type == 'takeoff':

                self.get_logger().info(
                    f'[{index}/{len(self.tasks)}] '
                    'TAKEOFF'
                )

            elif task_type == 'navigate':

                waypoint = task['waypoint']

                self.get_logger().info(
                    f'[{index}/{len(self.tasks)}] '
                    f'NAVIGATE -> {waypoint}'
                )

            elif task_type == 'land':

                self.get_logger().info(
                    f'[{index}/{len(self.tasks)}] '
                    'LAND'
                )

        self.get_logger().info(
            'Mission completed.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = WaypointExecutor()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
