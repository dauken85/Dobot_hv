from time import monotonic, sleep
import socket
import re

from dobot_api import DobotApiDashboard, DobotApiFeedBack


ROBOT_IP = "192.168.201.1"
ROBOT_PORT = 29999
FEEDBACK_PORT = 30004

START_TIMEOUT_S = 5.0
DONE_TIMEOUT_S = 30.0

MODE_NAMES = {
	1: "INIT",
	2: "BRAKE_OPEN",
	3: "POWEROFF",
	4: "DISABLED",
	5: "ENABLED_IDLE",
	6: "BACKDRIVE",
	7: "RUNNING",
	8: "SINGLE_MOVE",
	9: "ERROR",
	10: "PAUSE",
	11: "COLLISION",
}


def can_connect(ip: str, port: int, timeout: float = 2.0):
	try:
		with socket.create_connection((ip, port), timeout=timeout):
			return True, ""
	except OSError as exc:
		return False, str(exc)


def parse_result_code(response: str):
	if not response:
		return None
	match = re.search(r"-?\d+", response)
	if not match:
		return None
	return int(match.group(0))


def read_feedback(feedback: DobotApiFeedBack):
	data = feedback.feedBackData()
	if data is None or len(data) == 0:
		return None
	row = data[0]
	return {
		"mode": int(row["RobotMode"]),
		"running": int(row["RunningStatus"]),
		"command_id": int(row["CurrentCommandId"]),
	}


def wait_for_start_and_done(feedback: DobotApiFeedBack):
	started = False
	start_deadline = monotonic() + START_TIMEOUT_S

	while monotonic() < start_deadline:
		snapshot = read_feedback(feedback)
		if snapshot is None:
			sleep(0.05)
			continue

		mode = snapshot["mode"]
		running = snapshot["running"]

		if mode == 9:
			return False, False, "Robot entered ERROR state"
		if running == 1 or mode in (7, 8):
			started = True
			print(f"STARTED -> mode={mode} ({MODE_NAMES.get(mode, 'UNKNOWN')})")
			break

		sleep(0.05)

	if not started:
		print("STARTED not observed (likely non-motion command); continuing to DONE check")

	done_deadline = monotonic() + DONE_TIMEOUT_S
	while monotonic() < done_deadline:
		snapshot = read_feedback(feedback)
		if snapshot is None:
			sleep(0.05)
			continue

		mode = snapshot["mode"]
		running = snapshot["running"]

		if mode == 9:
			return started, False, "Robot entered ERROR state"

		if running == 0 and mode in (4, 5):
			print(f"DONE -> mode={mode} ({MODE_NAMES.get(mode, 'UNKNOWN')})")
			print("READY_FOR_NEXT -> true")
			return started, True, ""

		sleep(0.05)

	return started, False, "Timeout waiting for DONE/READY"

# This function sends a command using the provided send_task function, 
# checks for the initial ACK response
# then waits for the command to start and complete using the feedback. 
# It returns True if the command was accepted and completed successfully, 
# or False if it was rejected or if there was an error during execution.
def send_task_with_handshake(task_name: str, send_task, feedback: DobotApiFeedBack):
	print(f"REQUEST -> {task_name}")
	response = send_task().strip()
	code = parse_result_code(response)

	if code is None:
		print(f"ACK_RECEIVED -> unknown (raw: {response})")
	elif code == 0:
		print(f"ACK_RECEIVED -> accepted(code 0) (raw: {response})")
	else:
		print(f"ACK_RECEIVED -> rejected code={code} (raw: {response})")
		return False

	_, done, reason = wait_for_start_and_done(feedback)
	if not done:
		print(f"REQUEST_FAILED -> {reason}")
		return False

	return True


def choose_script_to_run(dashboard: DobotApiDashboard):
	result = dashboard.ListScripts()
	command = result.get("command", "")
	scripts = result.get("scripts", [])
	raw = result.get("raw", "")

	print(f"ListScripts probe -> {command}")
	if not scripts:
		print("No scripts returned by robot.")
		if raw:
			print(f"Raw response: {raw.strip()}")
		return None

	print("Available scripts:")
	for index, script_name in enumerate(scripts, start=1):
		print(f"  {index}. {script_name}")

	choice = input("Enter script number to run (blank to skip): ").strip()
	if not choice:
		return None

	if not choice.isdigit():
		print("Invalid choice, skipping RunScript.")
		return None

	selected_index = int(choice)
	if selected_index < 1 or selected_index > len(scripts):
		print("Choice out of range, skipping RunScript.")
		return None

	return scripts[selected_index - 1]


def main():
	
	ok, error = can_connect(ROBOT_IP, ROBOT_PORT)
	
	if not ok:
		print(f"Cannot connect to robot at {ROBOT_IP}:{ROBOT_PORT}")
		print(f"Connection error: {error}")
		print("Check robot IP, network adapter, same subnet, and firewall.")
		return

	ok, error = can_connect(ROBOT_IP, FEEDBACK_PORT)
	if not ok:
		print(f"Cannot connect to feedback at {ROBOT_IP}:{FEEDBACK_PORT}")
		print(f"Connection error: {error}")
		print("Check robot IP, network adapter, same subnet, and firewall.")
		return

	dashboard = DobotApiDashboard(ROBOT_IP, ROBOT_PORT)
	feedback = DobotApiFeedBack(ROBOT_IP, FEEDBACK_PORT)
	script_name = choose_script_to_run(dashboard)
	try:
		print(f"Connected: dashboard={ROBOT_PORT}, feedback={FEEDBACK_PORT}")
		

		if not send_task_with_handshake("EnableRobot", dashboard.EnableRobot, feedback):
			return

		script_name = choose_script_to_run(dashboard)
		if script_name:
			send_task_with_handshake(
				f"RunScript({script_name})",
				lambda: dashboard.RunScript(script_name),
				feedback,
			)

		if input("Disable robot now? [Y/n]: ").strip().lower() not in ("n", "no"):
			send_task_with_handshake("DisableRobot", dashboard.DisableRobot, feedback)
	except Exception as exc:
		print(f"Robot command failed: {exc}")
	finally:
		try:
			dashboard.close()
		except Exception:
			pass
		try:
			feedback.close()
		except Exception:
			pass


if __name__ == "__main__":
	main()

