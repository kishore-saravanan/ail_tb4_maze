#!/usr/bin/python3
"""Exercise the actual native GUI handlers against a running challenge."""

import json
import sys
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene

from ail_tb4_maze.live_scene import request, transport
from ail_tb4_maze.setup_gui import RESULTS, STATE, Setup

app = QtWidgets.QApplication(sys.argv)
window = Setup(autostart=False)
window.show()
pid = (RESULTS / "infrastructure.pid").read_text().strip()
records = []
phase = [0]
started = [time.monotonic()]
settle = [0.0]


def scene_check(label):
    c = json.loads((STATE / "challenge.json").read_text())
    scene = request(transport(), "/world/maze_escape/scene/info", Empty(), Scene)
    names = [m.name for m in scene.model if m.name.startswith(("border", "obstacles"))]
    assert sorted(names) == sorted(c["scene_models"].values()), names
    assert (RESULTS / "infrastructure.pid").read_text().strip() == pid
    records.append(
        {
            "action": label,
            "size": c["maze_size_m"],
            "grid": c["grid_cells"],
            "same_launch_pid": pid,
            "models": names,
        }
    )


def tick():
    try:
        window.read_status()
        s = window.latest_status
        if time.monotonic() - started[0] > 100:
            raise RuntimeError("GUI validation timed out: " + window.state_label.text())
        if phase[0] == 0 and window.start.isEnabled():
            window.randomize.click()
            phase[0] = 1
        elif (
            phase[0] == 1
            and not window.busy
            and window.scene_valid
            and "Live maze updated" in window.state_label.text()
        ):
            scene_check("Randomize button")
            window.size.setValue(24)
            window.cells.setValue(12)
            phase[0] = 2
        elif (
            phase[0] == 2
            and not window.busy
            and not window.debounce.isActive()
            and window.current["maze_size_m"] == 24
            and window.current["grid_cells"] == 12
        ):
            scene_check("Size and grid controls")
            window.reset.click()
            phase[0] = 3
        elif phase[0] == 3 and not window.busy and window.start.isEnabled():
            window.start.click()
            phase[0] = 4
        elif (
            phase[0] == 4
            and s.get("status") == "running"
            and s["elapsed_sim_seconds"] > 0.7
        ):
            assert window.elapsed.text() != "00:00.0"
            window.abort.click()
            phase[0] = 5
        elif (
            phase[0] == 5
            and s.get("status") == "failure"
            and window.open_result.isEnabled()
        ):
            assert "FAIL" in window.result.text()
            assert Path(s["result_file"]).exists()
            records.append(
                {
                    "action": "Start timer / Stop trial / FAIL / result file",
                    "result_file": s["result_file"],
                    "elapsed": s["elapsed_sim_seconds"],
                }
            )
            window.reset.click()
            phase[0] = 6
        elif (
            phase[0] == 6
            and not window.busy
            and window.start.isEnabled()
            and s.get("status") == "waiting"
        ):
            assert (
                window.elapsed.text() == "00:00.0" and window.collisions.text() == "0"
            )
            records.append(
                {
                    "action": "Reset score without reopening Gazebo",
                    "same_launch_pid": pid,
                }
            )
            (RESULTS / "gui_validation.json").write_text(
                json.dumps({"passed": True, "actions": records}, indent=2)
            )
            print(
                "GUI validation passed: randomize, resize, grid, timer, "
                "failure/file display and reset.",
                flush=True,
            )
            timer.stop()
            window.message("Validation passed. Ready for a new trial.")
            if "--keep-open" not in sys.argv:
                app.quit()
    except Exception as e:
        timer.stop()
        (RESULTS / "gui_validation.json").write_text(
            json.dumps({"passed": False, "error": str(e), "actions": records}, indent=2)
        )
        print("GUI validation failed:", e, flush=True)
        window.message(str(e))
        if "--keep-open" not in sys.argv:
            app.exit(1)


timer = QtCore.QTimer()
timer.setInterval(150)
timer.timeout.connect(tick)
timer.start()
sys.exit(app.exec_())
