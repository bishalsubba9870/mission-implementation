"""Normalize mission tasks for Behavior Tree execution."""

import re
from typing import Any


def normalize_tasks(
    raw_tasks: list[Any],
) -> list[dict[str, Any]]:
    """Normalize all mission tasks."""

    tasks = [
        normalize_task(index, task)
        for index, task in enumerate(
            raw_tasks,
            start=1,
        )
    ]

    if not tasks:
        raise ValueError(
            'Mission has zero tasks.'
        )

    return tasks


def normalize_task(
    index: int,
    raw_task: Any,
) -> dict[str, Any]:
    """Normalize one mission task."""

    if isinstance(raw_task, str):
        raw = {
            'name': raw_task,
        }

    elif isinstance(raw_task, dict):
        raw = dict(raw_task)

    else:
        raise ValueError(
            f'Invalid task at index {index}.'
        )

    task_id = str(
        raw.get(
            'id',
            raw.get(
                'task_id',
                f'TASK_{index}',
            ),
        )
    )

    name = str(
        raw.get(
            'name',
            raw.get(
                'description',
                task_id,
            ),
        )
    )

    task_type = determine_task_type(
        raw,
        task_id,
        name,
    )

    target = str(
        raw.get(
            'target',
            raw.get(
                'waypoint',
                raw.get(
                    'location',
                    '',
                ),
            ),
        )
    )

    if (
        task_type == 'navigate'
        and not target
    ):
        target = extract_waypoint(
            f'{task_id} {name}'
        )

    return {
        'index': index,
        'id': task_id,
        'type': task_type,
        'name': name,
        'target': target,
    }


def determine_task_type(
    raw: dict[str, Any],
    task_id: str,
    name: str,
) -> str:
    """Determine normalized mission task type."""

    explicit_type = raw.get(
        'type',
        raw.get(
            'task_type',
            raw.get(
                'action',
                '',
            ),
        ),
    )

    if explicit_type:
        return normalize_task_type(
            explicit_type
        )

    text = f'{task_id} {name}'.lower()

    type_checks = (
        ('takeoff', lambda: 'takeoff' in text),
        ('land', lambda: 'land' in text),
        ('hover', lambda: 'hover' in text),
        ('inspect', lambda: 'inspect' in text),
        (
            'rtl',
            lambda: (
                'rtl' in text
                or 'return_to_home' in text
            ),
        ),
        (
            'navigate',
            lambda: (
                'navigate' in text
                or 'waypoint' in text
                or re.search(
                    r'\bwp\s*[0-9]+\b',
                    text,
                )
                is not None
            ),
        ),
    )

    for task_type, matches in type_checks:
        if matches():
            return task_type

    return 'task'


def normalize_task_type(
    value: Any,
) -> str:
    """Normalize task-type aliases."""

    task_type = str(
        value
    ).strip().lower()

    aliases = {
        'take_off': 'takeoff',
        'take-off': 'takeoff',
        'nav': 'navigate',
        'navigation': 'navigate',
        'goto': 'navigate',
        'go_to': 'navigate',
        'waypoint': 'navigate',
        'inspection': 'inspect',
        'landing': 'land',
        'return_to_home': 'rtl',
    }

    return aliases.get(
        task_type,
        task_type,
    )


def extract_waypoint(
    text: str,
) -> str:
    """Extract a WPn waypoint identifier."""

    match = re.search(
        r'\bWP\s*([0-9]+)\b',
        str(text),
        re.IGNORECASE,
    )

    if match is None:
        return ''

    return f'WP{match.group(1)}'
