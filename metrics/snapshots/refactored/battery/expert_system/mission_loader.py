"""Shared mission loading utilities."""

import os
from typing import Any

import yaml

from ament_index_python.packages import get_package_share_directory


def load_mission(
    package_name: str,
    mission_file: str,
) -> tuple[str, str, list[dict[str, Any]], str]:
    """Load and validate a mission YAML file."""

    package_share = get_package_share_directory(
        package_name
    )

    mission_path = os.path.join(
        package_share,
        'missions',
        mission_file,
    )

    if not os.path.exists(mission_path):
        raise FileNotFoundError(
            f'Mission file not found: {mission_path}'
        )

    with open(
        mission_path,
        'r',
        encoding='utf-8',
    ) as file:
        data = yaml.safe_load(file)

    if not isinstance(data, dict):
        raise ValueError(
            'Mission YAML root must be a mapping.'
        )

    if (
        'mission' not in data
        or 'tasks' not in data
    ):
        raise ValueError(
            "Mission YAML requires 'mission' and 'tasks'."
        )

    mission_data = data['mission']
    tasks = data['tasks']

    if not isinstance(mission_data, dict):
        raise ValueError(
            "'mission' must be a mapping."
        )

    if (
        not isinstance(tasks, list)
        or not tasks
    ):
        raise ValueError(
            'Mission must contain at least one task.'
        )

    mission_id = str(
        mission_data['id']
    )

    mission_name = str(
        mission_data['name']
    )

    return (
        mission_id,
        mission_name,
        tasks,
        mission_path,
    )
