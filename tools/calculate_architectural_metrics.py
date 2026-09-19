#!/usr/bin/env python3
"""Calculate architectural metrics from saved formalism snapshots.

This script intentionally uses the installed `radon` command-line tool
instead of importing the Radon Python package. This works with pipx installs.
"""

from __future__ import annotations

import ast
import difflib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT_ROOT = ROOT / "metrics" / "snapshots" / "refactored"

STAGES = [
    "nominal",
    "gps",
    "propulsion",
    "communication",
    "wind",
    "battery",
    "abort",
]

FORMALISMS = {
    "FSM": "fsm",
    "Behavior Tree": "bt",
    "HTN": "htn",
    "Expert System": "expert_system",
}

EXCLUDED_FILES = {
    "__init__.py",
    "mission_loader.py",
    "runtime_condition.py",
    "progress_interface.py",
    "result_logger.py",
    "scenario_loader.py",
}

SCENARIO_TOKENS = {
    "GPS": [
        "GPS_LOST",
        "GPS_AVAILABLE",
        "GPS_RECOVERY_FAILED",
        "WAIT_FOR_GPS",
    ],
    "Propulsion": [
        "PROPULSION_FAILURE",
        "EMERGENCY_LAND",
    ],
    "Communication": [
        "COMMUNICATION_LOST",
        "COMMUNICATION_AVAILABLE",
        "COMMUNICATION_RECOVERY_FAILED",
        "WAIT_FOR_COMMUNICATION",
        "RETURN_TO_HOME",
    ],
    "Wind": [
        "WIND_UNSAFE",
        "SAFE_LAND",
        "WIND",
    ],
    "Battery": [
        "BATTERY_CRITICAL",
        "EMERGENCY_LAND",
        "BATTERY",
    ],
    "Abort": [
        "ABORT_REQUESTED",
        "ABORT_MISSION",
        "RETURN_TO_HOME",
    ],
}


def find_radon() -> str:
    """Return the installed Radon executable path."""

    radon = shutil.which("radon")
    if radon:
        return radon

    known_path = Path.home() / ".local" / "bin" / "radon"
    if known_path.exists():
        return str(known_path)

    print(
        "ERROR: radon executable was not found.\n"
        "Expected either `radon` in PATH or ~/.local/bin/radon.",
        file=sys.stderr,
    )
    sys.exit(1)


RADON = find_radon()


def run_command(args: list[str]) -> str:
    """Run a command and return stdout, failing with a clear message."""

    result = subprocess.run(
        args,
        text=True,
        capture_output=True,
        check=False,
    )

    if result.returncode != 0:
        command = " ".join(args)
        print(f"\nERROR while running:\n{command}", file=sys.stderr)

        if result.stdout.strip():
            print("\nSTDOUT:", file=sys.stderr)
            print(result.stdout, file=sys.stderr)

        if result.stderr.strip():
            print("\nSTDERR:", file=sys.stderr)
            print(result.stderr, file=sys.stderr)

        sys.exit(result.returncode)

    return result.stdout


def python_files(directory: Path) -> dict[str, Path]:
    """Return formalism-specific Python files indexed by relative path."""

    if not directory.exists():
        return {}

    files: dict[str, Path] = {}

    for path in sorted(directory.rglob("*.py")):
        if path.name in EXCLUDED_FILES:
            continue

        relative = str(path.relative_to(directory))
        files[relative] = path

    return files


def normalized_source_lines(path: Path) -> list[str]:
    """Return normalized nonblank, non-comment source lines."""

    text = path.read_text(encoding="utf-8")
    lines: list[str] = []

    for line in text.splitlines():
        stripped = line.strip()

        if not stripped:
            continue

        if stripped.startswith("#"):
            continue

        lines.append(stripped)

    return lines


def radon_sloc(path: Path) -> int:
    """Return Radon SLOC for one Python file using the CLI."""

    output = run_command(
        [RADON, "raw", "-j", str(path)]
    )

    data = json.loads(output)
    key = str(path)

    if key not in data:
        if len(data) == 1:
            stats = next(iter(data.values()))
        else:
            raise RuntimeError(
                f"Could not identify Radon raw result for {path}"
            )
    else:
        stats = data[key]

    return int(stats["sloc"])


def count_radon_json_blocks(entries: list[dict]) -> int:
    """Count classes, methods, functions, and nested closures."""

    count = 0

    for entry in entries:
        count += 1

        methods = entry.get("methods", [])
        count += len(methods)

        closures = entry.get("closures", [])
        count += count_radon_json_blocks(closures)

        for method in methods:
            method_closures = method.get("closures", [])
            count += count_radon_json_blocks(method_closures)

    return count


def radon_block_count(path: Path) -> int:
    """Return number of Radon complexity blocks for one Python file."""

    output = run_command(
        [RADON, "cc", str(path), "-s", "-a"]
    )

    match = re.search(
        r"(\d+)\s+blocks?\s+\(classes,\s*functions,\s*methods\)\s+analyzed",
        output,
        flags=re.IGNORECASE,
    )

    if match:
        return int(match.group(1))

    json_output = run_command(
        [RADON, "cc", "-j", str(path)]
    )

    data = json.loads(json_output)

    key = str(path)
    entries = data.get(key)

    if entries is None and len(data) == 1:
        entries = next(iter(data.values()))

    if entries is None:
        return 0

    return count_radon_json_blocks(entries)


def snapshot_sloc(directory: Path) -> int:
    """Return total formalism-specific SLOC for a snapshot."""

    return sum(
        radon_sloc(path)
        for path in python_files(directory).values()
    )


def snapshot_blocks(directory: Path) -> int:
    """Return total formalism-specific Radon blocks for a snapshot."""

    return sum(
        radon_block_count(path)
        for path in python_files(directory).values()
    )


def compare_file_lines(
    old_path: Path | None,
    new_path: Path | None,
) -> tuple[int, int]:
    """Return added and deleted normalized lines for one file."""

    old_lines = (
        normalized_source_lines(old_path)
        if old_path is not None
        else []
    )

    new_lines = (
        normalized_source_lines(new_path)
        if new_path is not None
        else []
    )

    matcher = difflib.SequenceMatcher(
        None,
        old_lines,
        new_lines,
        autojunk=False,
    )

    added = 0
    deleted = 0

    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag == "insert":
            added += j2 - j1
        elif tag == "delete":
            deleted += i2 - i1
        elif tag == "replace":
            deleted += i2 - i1
            added += j2 - j1

    return added, deleted


def compare_snapshots(
    old_directory: Path,
    new_directory: Path,
) -> dict[str, float | int]:
    """Compare two formalism snapshots."""

    old_files = python_files(old_directory)
    new_files = python_files(new_directory)

    all_names = sorted(set(old_files) | set(new_files))

    total_added = 0
    total_deleted = 0
    changed_files = 0

    old_all_lines: list[str] = []
    new_all_lines: list[str] = []

    for name in all_names:
        old_path = old_files.get(name)
        new_path = new_files.get(name)

        added, deleted = compare_file_lines(
            old_path,
            new_path,
        )

        total_added += added
        total_deleted += deleted

        if added > 0 or deleted > 0:
            changed_files += 1

        if old_path is not None:
            old_all_lines.extend(
                normalized_source_lines(old_path)
            )

        if new_path is not None:
            new_all_lines.extend(
                normalized_source_lines(new_path)
            )

    total_files = len(all_names)

    propagation_ratio = (
        changed_files / total_files
        if total_files > 0
        else 0.0
    )

    matcher = difflib.SequenceMatcher(
        None,
        old_all_lines,
        new_all_lines,
        autojunk=False,
    )

    reuse_ratio = matcher.ratio()

    return {
        "added": total_added,
        "deleted": total_deleted,
        "edit_cost": total_added + total_deleted,
        "changed_files": changed_files,
        "total_files": total_files,
        "propagation_ratio": propagation_ratio,
        "reuse_ratio": reuse_ratio,
    }


def function_locations(
    directory: Path,
    tokens: list[str],
) -> tuple[int, int]:
    """Count files and functions/methods containing recovery knowledge."""

    matching_files: set[str] = set()
    matching_functions: set[tuple[str, str, int]] = set()

    upper_tokens = [token.upper() for token in tokens]

    for relative, path in python_files(directory).items():
        source = path.read_text(encoding="utf-8")
        upper_source = source.upper()

        if any(token in upper_source for token in upper_tokens):
            matching_files.add(relative)

        try:
            tree = ast.parse(source)
        except SyntaxError:
            continue

        source_lines = source.splitlines()

        for node in ast.walk(tree):
            if not isinstance(
                node,
                (ast.FunctionDef, ast.AsyncFunctionDef),
            ):
                continue

            start = node.lineno - 1
            end = getattr(node, "end_lineno", node.lineno)

            function_source = "\n".join(
                source_lines[start:end]
            ).upper()

            if any(
                token in function_source
                for token in upper_tokens
            ):
                matching_functions.add(
                    (relative, node.name, node.lineno)
                )

    return len(matching_files), len(matching_functions)


def available_stages(folder_name: str) -> dict[str, Path]:
    """Return existing snapshot directories for a formalism."""

    result: dict[str, Path] = {}

    for stage in STAGES:
        directory = SNAPSHOT_ROOT / stage / folder_name

        if directory.exists() and python_files(directory):
            result[stage] = directory

    return result


def print_stage_metrics(
    formalism_name: str,
    folder_name: str,
) -> None:
    """Print architectural metrics for one reactive formalism."""

    print()
    print("=" * 78)
    print(formalism_name)
    print("=" * 78)

    stages = available_stages(folder_name)

    if not stages:
        print(
            f"No snapshots found under:\n"
            f"{SNAPSHOT_ROOT}/<stage>/{folder_name}/"
        )
        return

    if "nominal" not in stages:
        print(
            "Nominal snapshot not found; overall comparison cannot be made."
        )
        print("Available stages:", ", ".join(stages))
        return

    print()
    print("STAGE SIZE")
    print("-" * 78)

    stage_sloc: dict[str, int] = {}
    stage_blocks: dict[str, int] = {}

    for stage in STAGES:
        directory = stages.get(stage)

        if directory is None:
            continue

        sloc = snapshot_sloc(directory)
        blocks = snapshot_blocks(directory)

        stage_sloc[stage] = sloc
        stage_blocks[stage] = blocks

        print(
            f"{stage:<15}"
            f"SLOC={sloc:<7}"
            f"Blocks={blocks}"
        )

    print()
    print("INCREMENTAL MODIFICATION COST / CHANGE PROPAGATION")
    print("-" * 78)

    previous_stage: str | None = None

    for stage in STAGES:
        if stage not in stages:
            continue

        if previous_stage is None:
            previous_stage = stage
            continue

        result = compare_snapshots(
            stages[previous_stage],
            stages[stage],
        )

        print(
            f"{previous_stage:>13} -> {stage:<13}"
            f" added={int(result['added']):<5}"
            f" deleted={int(result['deleted']):<5}"
            f" edit_cost={int(result['edit_cost']):<5}"
            f" changed_files="
            f"{int(result['changed_files'])}/"
            f"{int(result['total_files'])}"
            f" propagation="
            f"{float(result['propagation_ratio']):.3f}"
        )

        previous_stage = stage

    final_stage = "abort" if "abort" in stages else STAGES[
        max(
            i
            for i, stage in enumerate(STAGES)
            if stage in stages
        )
    ]

    nominal = stages["nominal"]
    final = stages[final_stage]

    overall = compare_snapshots(
        nominal,
        final,
    )

    nominal_sloc = stage_sloc["nominal"]
    final_sloc = stage_sloc[final_stage]

    nominal_blocks = stage_blocks["nominal"]
    final_blocks = stage_blocks[final_stage]

    print()
    print("OVERALL REACTIVITY COST")
    print("-" * 78)

    print("Nominal stage:          nominal")
    print(f"Final stage:            {final_stage}")
    print(f"Nominal SLOC:           {nominal_sloc}")
    print(f"Final SLOC:             {final_sloc}")
    print(
        f"Delta SLOC:             "
        f"{final_sloc - nominal_sloc:+d}"
    )

    print(f"Nominal blocks:         {nominal_blocks}")
    print(f"Final blocks:           {final_blocks}")
    print(
        f"Delta blocks:           "
        f"{final_blocks - nominal_blocks:+d}"
    )

    print(
        "Direct edit cost:       "
        f"{int(overall['edit_cost'])}"
    )

    print(
        "Files changed overall:  "
        f"{int(overall['changed_files'])}/"
        f"{int(overall['total_files'])}"
    )

    print(
        "Overall propagation:    "
        f"{float(overall['propagation_ratio']):.3f}"
    )

    print(
        "Reuse similarity ratio: "
        f"{float(overall['reuse_ratio']):.3f} "
        f"({float(overall['reuse_ratio']) * 100:.1f}%)"
    )

    print()
    print("RECOVERY LOCALITY IN FINAL SNAPSHOT")
    print("-" * 78)

    for scenario, tokens in SCENARIO_TOKENS.items():
        file_count, function_count = function_locations(
            final,
            tokens,
        )

        print(
            f"{scenario:<16}"
            f"files={file_count:<4}"
            f"functions/methods={function_count}"
        )


def print_behavioral_coverage() -> None:
    """Print scenario capability for the five formalisms."""

    print()
    print("=" * 78)
    print("BEHAVIORAL COVERAGE")
    print("=" * 78)

    rows = [
        (
            "Waypoint",
            0,
            "N/A / unsupported by baseline",
        ),
        (
            "FSM",
            6,
            "supported",
        ),
        (
            "Behavior Tree",
            6,
            "supported",
        ),
        (
            "HTN",
            6,
            "supported",
        ),
        (
            "Expert System",
            6,
            "supported",
        ),
    ]

    for name, supported, note in rows:
        coverage = (
            supported / 6.0
        ) * 100.0

        print(
            f"{name:<18}"
            f"{supported}/6  "
            f"{coverage:>6.1f}%  "
            f"{note}"
        )


def print_known_structural_growth() -> None:
    """Print structural-growth facts already established."""

    missions = {
        "M1": 4,
        "M2": 13,
        "M3": 102,
        "M4": 201,
    }

    print()
    print("=" * 78)
    print("STRUCTURAL GROWTH - CURRENTLY ESTABLISHED VALUES")
    print("=" * 78)

    print()
    print("Waypoint")
    print("-" * 78)
    print(
        "Executor structure remains fixed; "
        "mission data grows with mission items."
    )

    for mission, items in missions.items():
        print(
            f"{mission}: mission_items={items}"
        )

    print()
    print("FSM")
    print("-" * 78)

    print(
        "Established relation: "
        "states = N + 8, transitions = 7N - 6"
    )

    for mission, items in missions.items():
        states = items + 8
        transitions = (
            7 * items
        ) - 6

        print(
            f"{mission}: "
            f"items={items:<4}"
            f" states={states:<4}"
            f" transitions={transitions}"
        )

    print()
    print("Expert System")
    print("-" * 78)

    print(
        "Final scenario-capable rule base: 18 rules; "
        "rule count remains independent of M1-M4 mission size."
    )

    for mission, items in missions.items():
        print(
            f"{mission}: "
            f"items={items:<4} "
            f"rules=18"
        )

    print()
    print("Behavior Tree / HTN")
    print("-" * 78)

    print(
        "Exact M1-M4 tree/plan structural counts "
        "are intentionally not guessed here."
    )

    print(
        "They should be measured from the generated "
        "BT/HTN runtime structures."
    )


def print_method_notes() -> None:
    """Explain the operational definitions used."""

    print()
    print("=" * 78)
    print("OPERATIONAL DEFINITIONS")
    print("=" * 78)

    print(
        "Modification Cost:"
        "\n  added normalized source lines + deleted normalized source lines"
        "\n  between consecutive scenario snapshots."
    )

    print(
        "\nChange Propagation:"
        "\n  changed formalism-specific files / total formalism-specific files"
        "\n  involved in the compared snapshots."
    )

    print(
        "\nReactivity Cost:"
        "\n  nominal-to-final Delta SLOC, Delta Radon blocks, "
        "and direct edit cost."
    )

    print(
        "\nRecovery Locality:"
        "\n  number of formalism-specific files and functions/methods "
        "that contain"
        "\n  scenario/recovery event knowledge in the final snapshot."
    )

    print(
        "\nReuse Evidence:"
        "\n  similarity ratio between normalized nominal and final"
        "\n  formalism-specific source lines."
    )

    print(
        "\nBehavioral Coverage:"
        "\n  supported required runtime scenarios / 6."
    )


def main() -> None:
    """Calculate and print all derivable architectural metrics."""

    print(f"Project root: {ROOT}")
    print(f"Snapshot root: {SNAPSHOT_ROOT}")
    print(f"Radon executable: {RADON}")

    for formalism_name, folder_name in FORMALISMS.items():
        print_stage_metrics(
            formalism_name,
            folder_name,
        )

    print_behavioral_coverage()
    print_known_structural_growth()
    print_method_notes()


if __name__ == "__main__":
    main()
