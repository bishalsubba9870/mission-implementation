"""Shared ROS2 mission-progress utilities."""

import json

from std_msgs.msg import String


def publish_progress(
    publisher,
    mission_id: str,
    state_name: str,
    task_type: str,
    status: str,
    completed_navigation_tasks: int,
) -> None:
    """Publish standardized mission progress."""

    message = String()

    message.data = json.dumps(
        {
            'mission_id':
                mission_id,

            'state_name':
                state_name,

            'task_type':
                task_type,

            'task_status':
                status,

            'completed_navigation_tasks':
                completed_navigation_tasks,
        }
    )

    publisher.publish(
        message
    )
