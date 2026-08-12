import yaml


def generate_mission_3():
    """Generate Mission 3 with 100 outbound waypoints."""

    tasks = [{"type": "takeoff"}]

    for i in range(1, 101):
        tasks.append(
            {
                "type": "navigate",
                "waypoint": f"WP{i}",
            }
        )

    tasks.append({"type": "land"})

    mission = {
        "mission": {
            "id": "M3",
            "name": "Zigzag Navigation"
        },
        "tasks": tasks,
    }

    with open("mission_3.yaml", "w", encoding="utf-8") as file:
        yaml.safe_dump(
            mission,
            file,
            sort_keys=False,
        )


def generate_mission_4():
    """Generate Mission 4 with 100 outbound waypoints and return."""

    tasks = [{"type": "takeoff"}]

    # Outbound:
    # WP1 -> WP2 -> ... -> WP100
    for i in range(1, 101):
        tasks.append(
            {
                "type": "navigate",
                "waypoint": f"WP{i}",
            }
        )

    # Return:
    # WP99 -> WP98 -> ... -> WP1
    #
    # WP100 is not repeated because the drone is
    # already located at WP100 when the return starts.
    for i in range(99, 0, -1):
        tasks.append(
            {
                "type": "navigate",
                "waypoint": f"WP{i}",
            }
        )

    tasks.append({"type": "land"})

    mission = {
        "mission": {
            "id": "M4",
            "name": "Zigzag and Return"
        },
        "tasks": tasks,
    }

    with open("mission_4.yaml", "w", encoding="utf-8") as file:
        yaml.safe_dump(
            mission,
            file,
            sort_keys=False,
        )


if __name__ == "__main__":

    generate_mission_3()
    generate_mission_4()

    print("Mission 3 generated: 100 waypoints + Takeoff + Land")
    print("Mission 4 generated: 100 outbound + 99 return waypoints")
