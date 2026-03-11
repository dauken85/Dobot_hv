import re
from dobot_api import DobotApiDashboard


MANUAL_REFERENCE = "TCP_IP Remote Control Interface Guide (V3)_20250508_en"
DOCUMENTED_SCRIPT_COMMANDS = ["RunScript", "StopScript", "PauseScript", "ContinueScript"]


def parse_script_names(response: str) -> list[str]:
    quoted = re.findall(r'"([^"\n]+)"', response)
    if quoted:
        return sorted(set(quoted))

    bare = re.findall(r"[A-Za-z_][A-Za-z0-9_\-.]*", response)
    blacklist = {
        "OK",
        "ERR",
        "ErrorID",
        "GetScriptList",
        "GetProgramList",
        "GetProjectList",
        "GetRoutineList",
        "RunScript",
    }
    return sorted({token for token in bare if token not in blacklist})


def fetch_available_scripts(dashboard: DobotApiDashboard):
    # NOTE:
    # Based on MANUAL_REFERENCE, no official command for listing all scripts/projects
    # is documented. The commands below are fallback probes that may exist on some
    # firmware variants.
    candidate_commands = [
        "GetScriptList()",
        "GetProgramList()",
        "GetProjectList()",
        "GetRoutineList()",
    ]

    best_response = ""
    for command in candidate_commands:
        response = dashboard.sendRecvMsg(command)
        names = parse_script_names(response)
        if names:
            return command, response, names
        if response and not response.startswith("-"):
            best_response = response

    return "No working list command found", best_response, []


if __name__ == '__main__':
    robot_ip = input("Enter robot IP (default 192.168.5.1): ").strip() or "192.168.5.1"
    dashboard_port = 29999

    dashboard = None
    try:
        dashboard = DobotApiDashboard(robot_ip, dashboard_port)
        command_used, raw_response, scripts = fetch_available_scripts(dashboard)

        print(f"Connected to {robot_ip}:{dashboard_port}")
        print(f"Manual checked: {MANUAL_REFERENCE}")
        print(
            "Documented script-related commands: "
            + ", ".join(DOCUMENTED_SCRIPT_COMMANDS)
        )
        print("No official script-list command is documented in that manual.")
        print(f"Command used: {command_used}")

        if scripts:
            print("\nAvailable scripts on robot:")
            for index, name in enumerate(scripts, start=1):
                print(f"{index}. {name}")
        else:
            print("\nNo script names could be parsed.")
            print("The robot/firmware may not expose a list command over dashboard TCP.")
            if raw_response:
                print("Raw response:")
                print(raw_response)
            else:
                print("No valid response from listing commands.")

    except Exception as error:
        print(f"Failed to connect/query robot at {robot_ip}:{dashboard_port}")
        print(f"Details: {error}")
    finally:
        if dashboard is not None:
            dashboard.close()
