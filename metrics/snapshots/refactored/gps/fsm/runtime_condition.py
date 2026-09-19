"""Shared runtime-event parsing."""

import json


def parse_runtime_event(
    raw_data: str,
) -> str:
    """Return normalized runtime-event name."""

    try:
        parsed = json.loads(
            raw_data
        )

        if isinstance(
            parsed,
            dict,
        ):
            return str(
                parsed.get(
                    'event',
                    '',
                )
            ).strip().upper()

    except json.JSONDecodeError:
        pass

    return (
        raw_data
        .strip()
        .upper()
    )
