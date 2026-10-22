#!/usr/bin/python3
"""Native operator setup window. This is privileged infrastructure, not navigation."""

import json
import os
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from ail_tb4_maze.live_scene import reset_trial, ros_service, update
from ail_tb4_maze.paths import (
    NAVIGATION_ROOT,
    RESULTS,
    STATE,
)
from ail_tb4_maze.paths import (
    PACKAGE_SHARE as PKG,
)
from ail_tb4_maze.world_factory import generate

STATE.mkdir(parents=True, exist_ok=True)
RESULTS.mkdir(parents=True, exist_ok=True)


STYLE_SHEET = """
QMainWindow, QWidget {
    background: #101b22;
    color: #edf1ec;
    font-size: 13px;
}

QLabel#title {
    font-size: 26px;
    font-weight: 700;
}

QLabel#subtitle {
    color: #9db2b4;
}

QGroupBox {
    border: 1px solid #34474c;
    border-radius: 10px;
    margin-top: 13px;
    padding: 12px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
}

QPushButton {
    background: #253a42;
    border: 1px solid #47616a;
    border-radius: 7px;
    padding: 10px;
    font-weight: 600;
}

QPushButton:hover {
    background: #35545e;
}

QPushButton:disabled {
    color: #768889;
    background: #1b2a31;
}

QPushButton#primary {
    background: #39ad80;
    color: #061c16;
    border: 0;
}

QSpinBox, QDoubleSpinBox, QComboBox {
    background: #203139;
    border: 1px solid #45606a;
    padding: 7px;
    border-radius: 5px;
}

QLabel#metric {
    font-size: 24px;
    font-weight: 700;
}

QLabel#result {
    font-size: 22px;
    font-weight: 700;
    color: #58d8a4;
}

QPlainTextEdit {
    background: #17272e;
    border: 1px solid #39545c;
    border-radius: 6px;
    color: #b8cbcc;
}
"""


class Worker(QtCore.QThread):
    done = QtCore.pyqtSignal(object)
    error = QtCore.pyqtSignal(str)

    def __init__(self, fn):
        super().__init__()
        self.fn = fn

    def run(self):
        try:
            self.done.emit(self.fn())
        except Exception as e:
            self.error.emit(str(e))


class Preview(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.config = None
        self.setMinimumHeight(170)

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#17242c"))
        if not self.config:
            return
        c = self.config
        lo, hi, bot, top = c["bounds"]
        w = min(self.width() - 32, self.height() - 26)
        k = w / (hi - lo + 1)
        ox = (self.width() - w) / 2
        oy = 13

        def point(x, y):
            return QtCore.QPointF(ox + (x - lo) * k, oy + (top - y) * k)

        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor("#ad8268"))
        for a, b, d, e in c["wall_boxes"]:
            q = point(a, e)
            p.drawRect(QtCore.QRectF(q.x(), q.y(), (b - a) * k, (e - d) * k))
        p.setBrush(QtGui.QColor("#55deaf"))
        s = point(c["start"]["x"], c["start"]["y"])
        p.drawEllipse(s, 4, 4)
        p.setPen(QtGui.QPen(QtGui.QColor("#62dfba"), 3))
        ex = c["exit"]
        p.drawLine(point(ex["x"], ex["y_min"]), point(ex["x"], ex["y_max"]))
        p.setPen(QtGui.QColor("#edf1ec"))
        p.drawText(12, 22, f"Applied: {hi - lo:g} × {top - bot:g} m")


class ExistingProcess:
    def __init__(self, pid):
        self.pid = pid

    def poll(self):
        return None if Path(f"/proc/{self.pid}").exists() else 0


class Setup(QtWidgets.QMainWindow):
    def __init__(self, autostart=True):
        super().__init__()
        self.setWindowTitle("AI Legion · Maze setup")
        self.resize(540, 930)
        self.proc = None
        self.owns_server = False
        self.scene_valid = True
        self.log = None
        self.worker = None
        self.busy = False
        self.current = None
        self.queued = False
        self.latest_status = {}
        self.controller = None
        self.controller_log = None
        self.seed = 42
        self.sim_log = RESULTS / "visible_environment.log"
        self.setStyleSheet(STYLE_SHEET)
        base = QtWidgets.QWidget()
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(base)
        self.setCentralWidget(scroll)
        layout = QtWidgets.QVBoxLayout(base)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(10)
        title = QtWidgets.QLabel("Maze Escape")
        title.setObjectName("title")
        layout.addWidget(title)
        sub = QtWidgets.QLabel("TurtleBot 4  ·  Outdoor ground, brick & green hedges")
        sub.setObjectName("subtitle")
        layout.addWidget(sub)
        controls = QtWidgets.QGroupBox("Live maze controls")
        form = QtWidgets.QFormLayout(controls)
        self.preset = QtWidgets.QComboBox()
        self.preset.addItems(["Official maze", "Random maze"])
        form.addRow("Layout", self.preset)
        self.size = QtWidgets.QSpinBox()
        self.size.setRange(12, 40)
        self.size.setValue(20)
        self.size.setSuffix(" m")
        self.size.setKeyboardTracking(False)
        self.size.setToolTip(
            "Square maze side: 12–40 metres. Press Enter or Apply after typing."
        )
        form.addRow("Maze side (12–40 m)", self.size)
        self.cells = QtWidgets.QSpinBox()
        self.cells.setRange(5, 20)
        self.cells.setValue(10)
        self.cells.setSuffix(" × cells")
        self.cells.setKeyboardTracking(False)
        self.cells.setEnabled(False)
        form.addRow("Grid per side", self.cells)
        self.details = QtWidgets.QLabel("")
        self.details.setWordWrap(True)
        form.addRow(self.details)
        row = QtWidgets.QHBoxLayout()
        self.randomize = QtWidgets.QPushButton("⟳  Randomize")
        self.reset = QtWidgets.QPushButton("Reset robot / trial")
        row.addWidget(self.randomize)
        row.addWidget(self.reset)
        form.addRow(row)
        self.apply_button = QtWidgets.QPushButton("Apply size / grid")
        self.apply_button.clicked.connect(self.apply_controls)
        form.addRow(self.apply_button)
        layout.addWidget(controls)
        self.preview = Preview()
        layout.addWidget(self.preview)
        note = QtWidgets.QLabel(
            "Size and grid changes apply live after a short pause.\n"
            "Changing the maze resets the trial. Gazebo stays open."
        )
        note.setObjectName("subtitle")
        layout.addWidget(note)
        self.score_group = score = QtWidgets.QGroupBox("Trial")
        grid = QtWidgets.QGridLayout(score)
        self.elapsed = QtWidgets.QLabel("00:00.0")
        self.elapsed.setObjectName("metric")
        self.collisions = QtWidgets.QLabel("0")
        self.collisions.setObjectName("metric")
        self.distance = QtWidgets.QLabel("0.00 m")
        self.distance.setObjectName("metric")
        for col, (name, label) in enumerate(
            [
                ("Elapsed", self.elapsed),
                ("Collisions", self.collisions),
                ("Travelled", self.distance),
            ]
        ):
            grid.addWidget(QtWidgets.QLabel(name), 0, col)
            grid.addWidget(label, 1, col)
        self.result = QtWidgets.QLabel("PREPARING")
        self.result.setObjectName("result")
        grid.addWidget(self.result, 2, 0, 1, 3)
        self.start = QtWidgets.QPushButton("▶  Start timer")
        self.start.setObjectName("primary")
        self.abort = QtWidgets.QPushButton("Stop trial")
        grid.addWidget(self.start, 3, 0, 1, 2)
        grid.addWidget(self.abort, 3, 2)
        self.file = QtWidgets.QLabel("Result file appears after a trial.")
        self.file.setWordWrap(True)
        self.file.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        grid.addWidget(self.file, 4, 0, 1, 3)
        self.open_result = QtWidgets.QPushButton("Open result file")
        self.open_result.setEnabled(False)
        grid.addWidget(self.open_result, 5, 0, 1, 3)
        self.time_limit = QtWidgets.QDoubleSpinBox()
        self.time_limit.setRange(0, 1e9)
        self.time_limit.setDecimals(1)
        self.time_limit.setSingleStep(60)
        self.time_limit.setSpecialValueText("No limit")
        self.time_limit.setSuffix(" s")
        self.time_limit.setToolTip(
            "0 = unlimited. Applied when Start timer is pressed; "
            "reset for a new attempt."
        )
        self.time_limit.setValue(float(os.environ.get("AIL_TB4_MAZE_TIME_LIMIT", "0")))
        grid.addWidget(QtWidgets.QLabel("Time limit"), 6, 0)
        grid.addWidget(self.time_limit, 6, 1, 1, 2)
        self.time_limit.valueChanged.connect(self.update_time_limit_title)
        self.update_time_limit_title()
        layout.addWidget(score)
        controller_box = QtWidgets.QGroupBox("Navigation controller")
        controller_layout = QtWidgets.QVBoxLayout(controller_box)
        self.controller_path = QtWidgets.QLineEdit(
            str(NAVIGATION_ROOT / "controller.py")
        )
        controller_layout.addWidget(self.controller_path)
        controller_row = QtWidgets.QHBoxLayout()
        self.controller_start = QtWidgets.QPushButton("Start solver")
        self.controller_kill = QtWidgets.QPushButton("Stop solver")
        controller_row.addWidget(self.controller_start)
        controller_row.addWidget(self.controller_kill)
        controller_layout.addLayout(controller_row)
        self.controller_status = QtWidgets.QLabel("Stopped · Live-map frontier routing")
        controller_layout.addWidget(self.controller_status)
        self.controller_start.clicked.connect(self.start_controller)
        self.controller_kill.clicked.connect(
            lambda: self.task(
                self.stop_controller, lambda _: self.message("Navigation stopped.")
            )
        )
        layout.addWidget(controller_box)
        self.state_label = QtWidgets.QLabel("Preparing environment…")
        self.state_label.setWordWrap(True)
        layout.addWidget(self.state_label)
        bottom = QtWidgets.QGridLayout()
        self.launch = QtWidgets.QPushButton("Launch environment")
        self.rviz = QtWidgets.QPushButton("Open RViz")
        self.navmap = QtWidgets.QPushButton("Navigation map")
        self.folder = QtWidgets.QPushButton("Results folder")
        bottom.addWidget(self.launch, 0, 0)
        bottom.addWidget(self.rviz, 0, 1)
        bottom.addWidget(self.navmap, 1, 0)
        bottom.addWidget(self.folder, 1, 1)
        layout.addLayout(bottom)
        self.start.clicked.connect(self.start_trial)
        self.abort.clicked.connect(
            lambda: self.task(lambda: ros_service("/referee/abort"), self.message)
        )
        self.randomize.clicked.connect(self.random)
        self.reset.clicked.connect(self.reset_trial)
        self.launch.clicked.connect(self.launch_environment)
        self.rviz.clicked.connect(self.open_rviz)
        self.navmap.clicked.connect(self.open_navigation_map)
        self.folder.clicked.connect(
            lambda: QtGui.QDesktopServices.openUrl(
                QtCore.QUrl.fromLocalFile(str(RESULTS))
            )
        )
        self.open_result.clicked.connect(self.show_result)
        self.debounce = QtCore.QTimer()
        self.debounce.setSingleShot(True)
        self.debounce.setInterval(600)
        self.debounce.timeout.connect(self.apply)
        self.size.valueChanged.connect(self.controls_changed)
        self.cells.valueChanged.connect(self.controls_changed)
        self.preset.currentIndexChanged.connect(self.controls_changed)
        self.poll = QtCore.QTimer()
        self.poll.setInterval(200)
        self.poll.timeout.connect(self.read_status)
        self.poll.start()
        pid_file = RESULTS / "infrastructure.pid"
        if pid_file.exists():
            pid = int(pid_file.read_text().strip())
            cmdline = Path(f"/proc/{pid}/cmdline")
            if cmdline.exists() and b"config_file:" in cmdline.read_bytes():
                self.proc = ExistingProcess(pid)
        self.current = (
            json.loads((STATE / "challenge.json").read_text())
            if (STATE / "challenge.json").exists()
            else generate(PKG, STATE, 20, 10, self.seed, "official")
        )
        self.seed = self.current["seed"]
        for widget, value in [
            (self.size, int(self.current["maze_size_m"])),
            (self.cells, self.current["grid_cells"]),
            (self.preset, 0 if self.current["preset"] == "official" else 1),
        ]:
            widget.blockSignals(True)
            if widget is self.preset:
                widget.setCurrentIndex(value)
            else:
                widget.setValue(value)
            widget.blockSignals(False)
        self.cells.setEnabled(self.preset.currentIndex() == 1)
        self.preview.config = self.current
        self.preview.update()
        self.controls_details()
        self.config_mtime = (STATE / "challenge.json").stat().st_mtime_ns
        if autostart:
            QtCore.QTimer.singleShot(300, self.launch_environment)
        else:
            self.message("Preview ready. Launch environment to begin.")

    def task(self, fn, done):
        if self.busy:
            self.queued = True
            return
        self.busy = True
        self.update_buttons()
        self.worker = Worker(fn)
        self.worker.done.connect(lambda value: self.completed(value, done))
        self.worker.error.connect(self.failed)
        self.worker.start()

    def completed(self, value, done):
        self.busy = False
        done(value)
        self.update_buttons()
        if self.queued:
            self.queued = False
            self.debounce.start()

    def failed(self, error):
        self.busy = False
        self.queued = False
        self.message("Update failed: " + error)
        self.update_buttons()

    def message(self, text):
        self.state_label.setText(str(text))

    def update_time_limit_title(self):
        limit = self.time_limit.value()
        self.score_group.setTitle(
            "Trial · no time limit"
            if limit == 0
            else f"Trial · {limit:g} simulated seconds"
        )

    def start_trial(self):
        limit = self.time_limit.value()

        def work():
            subprocess.run(
                [
                    "ros2",
                    "param",
                    "set",
                    "/challenge_referee",
                    "time_limit",
                    str(limit),
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=15,
            )
            return ros_service("/referee/start")

        self.task(work, self.message)

    def controls_details(self):
        if self.preset.currentIndex() == 0:
            text = (
                f"Official layout · {self.size.value()} × {self.size.value()} m "
                "· checked for clearance"
            )
        else:
            text = (
                f"{self.cells.value()} × {self.cells.value()} grid · "
                f"{self.size.value() / self.cells.value():.2f} m cells "
                f"· seed {self.seed}"
            )
        self.details.setText(text)

    def controls_changed(self):
        self.cells.setEnabled(self.preset.currentIndex() == 1)
        maximum = min(20, self.size.value())
        self.cells.setMaximum(maximum)
        self.scene_valid = False
        self.update_buttons()
        self.controls_details()
        self.message("Pending size/grid update…")
        self.debounce.start()

    def random(self):
        self.seed = secrets.randbelow(2**31)
        self.preset.blockSignals(True)
        self.preset.setCurrentIndex(1)
        self.preset.blockSignals(False)
        self.cells.setEnabled(True)
        self.controls_details()
        self.debounce.stop()
        self.apply()

    def apply_controls(self):
        self.size.interpretText()
        self.cells.interpretText()
        self.debounce.stop()
        self.apply()

    def apply(self):
        if self.busy:
            self.queued = True
            return
        size = self.size.value()
        cells = self.cells.value()
        seed = self.seed
        preset = "official" if self.preset.currentIndex() == 0 else "random"
        running = self.proc is not None and self.proc.poll() is None
        self.stop_controller()
        self.scene_valid = False
        self.message("Checking clearance and updating Gazebo…")

        def work():
            draft = STATE / "drafts" / str(time.monotonic_ns())
            c = generate(PKG, draft, size, cells, seed, preset)
            if running:
                update(draft, STATE)
            else:
                for name in ["world.sdf", "challenge.json"]:
                    (STATE / name).write_bytes((draft / name).read_bytes())
            return c

        self.task(work, self.applied)

    def applied(self, c):
        self.scene_valid = True
        self.config_mtime = (STATE / "challenge.json").stat().st_mtime_ns
        self.current = c
        self.preview.config = c
        self.preview.update()
        self.controls_details()
        self.message(
            (
                f"Live maze updated: {c['maze_size_m']:g} × "
                f"{c['maze_size_m']:g} m. Same Gazebo session; timer ready."
            )
            if self.proc and self.proc.poll() is None
            else (
                f"Maze prepared: {c['maze_size_m']:g} × "
                f"{c['maze_size_m']:g} m. Launch environment to begin."
            )
        )

    def launch_environment(self):
        if self.proc is not None and self.proc.poll() is None:
            self.message(
                "Connected to the running environment. Live updates are ready."
            )
            return
        (STATE / "live_models.json").write_text(
            json.dumps(self.current["scene_models"])
        )
        self.log = self.sim_log.open("w")
        s = self.current["start"]
        self.owns_server = True
        self.proc = subprocess.Popen(
            [
                "ros2",
                "launch",
                "ail_tb4_maze",
                "challenge.launch.py",
                f"results_dir:={RESULTS}",
                f"world_file:={STATE}/world.sdf",
                f"config_file:={STATE}/challenge.json",
                f"x:={s['x']}",
                f"y:={s['y']}",
                f"time_limit:={self.time_limit.value()}",
                "rviz:=true",
            ],
            stdout=self.log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        (RESULTS / "infrastructure.pid").write_text(str(self.proc.pid) + "\n")
        self.message("Opening Gazebo and RViz…")
        self.update_buttons()

    def reset_trial(self):
        if self.current is None:
            return
        self.stop_controller()
        # Reapply current scene to safely stop, reposition and clear the referee.
        self.message("Resetting robot and score in the current Gazebo session…")
        self.task(
            lambda: reset_trial(STATE),
            lambda value: (setattr(self, "scene_valid", True), self.message(value)),
        )

    def start_controller(self):
        if self.controller and self.controller.poll() is None:
            return
        path = Path(self.controller_path.text()).expanduser().resolve()
        if not path.is_file():
            self.message("Controller script does not exist: " + str(path))
            return
        env = dict(os.environ, ROS_DOMAIN_ID="62", GZ_PARTITION="ai_legion_navigation")
        self.controller_log = (RESULTS / "controller.log").open("w")
        self.controller = subprocess.Popen(
            [
                "/usr/bin/python3",
                str(path),
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "-p",
                f"recording_path:={RESULTS}/exploration_{time.time_ns()}.jsonl",
            ],
            cwd=str(NAVIGATION_ROOT),
            env=env,
            stdout=self.controller_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.update_buttons()

    def stop_controller(self):
        if self.controller and self.controller.poll() is None:
            os.killpg(self.controller.pid, signal.SIGINT)
            try:
                self.controller.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.controller.pid, signal.SIGTERM)
                try:
                    self.controller.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(self.controller.pid, signal.SIGKILL)
                    self.controller.wait()
        if self.controller_log:
            self.controller_log.close()
            self.controller_log = None

    def cleanup(self):
        self.stop_controller()
        if self.owns_server and self.proc and self.proc.poll() is None:
            os.kill(self.proc.pid, signal.SIGINT)

    def open_rviz(self):
        self.open_navigation_map()

    def open_navigation_map(self):
        env = dict(os.environ, ROS_DOMAIN_ID="62")
        subprocess.Popen(
            ["ros2", "launch", "ail_tb4_maze", "navigation_view.launch.py"],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def read_status(self):
        if (
            not self.busy
            and not self.debounce.isActive()
            and not self.size.hasFocus()
            and not self.cells.hasFocus()
        ):
            try:
                stamp = (STATE / "challenge.json").stat().st_mtime_ns
                if stamp != self.config_mtime:
                    c = json.loads((STATE / "challenge.json").read_text())
                    self.current = c
                    self.seed = c["seed"]
                    self.config_mtime = stamp
                    for widget, value in [
                        (self.size, int(c["maze_size_m"])),
                        (self.cells, c["grid_cells"]),
                        (self.preset, 0 if c["preset"] == "official" else 1),
                    ]:
                        widget.blockSignals(True)
                        if widget is self.preset:
                            widget.setCurrentIndex(value)
                        else:
                            widget.setValue(value)
                        widget.blockSignals(False)
                    self.cells.setEnabled(self.preset.currentIndex() == 1)
                    self.preview.config = c
                    self.preview.update()
                    self.controls_details()
            except (OSError, ValueError, AttributeError):
                pass
        if self.proc is not None and self.proc.poll() is not None:
            self.result.setText("ENVIRONMENT STOPPED")
            self.update_buttons()
            return
        try:
            s = json.loads((RESULTS / "status.json").read_text())
        except (OSError, ValueError):
            self.update_buttons()
            return
        self.latest_status = s
        t = s["elapsed_sim_seconds"]
        self.elapsed.setText(f"{int(t // 60):02}:{t % 60:04.1f}")
        self.collisions.setText(str(s["collision_episodes"]))
        self.distance.setText(f"{s['distance_meters']:.2f} m")
        names = {
            "waiting": "READY",
            "running": "RUNNING",
            "success": "PASS · escaped",
            "failure": "FAIL",
            "timeout": "TIMEOUT · FAIL",
        }
        self.result.setText(names.get(s["status"], s["status"]))
        self.result.setStyleSheet(
            "color:"
            + (
                "#59d9a7"
                if s["status"] in ["success", "waiting"]
                else "#f1bd6f"
                if s["status"] == "running"
                else "#f38b7e"
            )
        )
        if s.get("result_file"):
            self.file.setText("Saved: " + Path(s["result_file"]).name)
            self.file.setToolTip(s["result_file"])
            self.open_result.setEnabled(True)
        else:
            self.file.setText("Result file appears after a trial.")
            self.open_result.setEnabled(False)
        self.update_buttons()

    def update_buttons(self):
        running = self.proc is not None and self.proc.poll() is None
        controller_running = (
            self.controller is not None and self.controller.poll() is None
        )
        self.controller_start.setEnabled(
            running and not self.busy and not controller_running
        )
        self.controller_kill.setEnabled(controller_running)
        default_solver = (
            Path(self.controller_path.text()).expanduser().resolve()
            == (NAVIGATION_ROOT / "controller.py").resolve()
        )
        label = "Live-map frontier routing" if default_solver else "Custom controller"
        self.controller_status.setText(
            ("Running · " if controller_running else "Stopped · ") + label
        )
        s = self.latest_status
        self.time_limit.setEnabled(not self.busy and s.get("status") != "running")
        self.start.setEnabled(
            running
            and self.scene_valid
            and not self.busy
            and s.get("status") == "waiting"
            and s.get("pose_ready", False)
        )
        self.abort.setEnabled(
            running and not self.busy and s.get("status") == "running"
        )
        self.apply_button.setEnabled(not self.busy)
        self.reset.setEnabled(running and not self.busy)
        self.randomize.setEnabled(not self.busy)
        self.launch.setEnabled(not running and not self.busy)

    def show_result(self):
        path = self.latest_status.get("result_file")
        if path:
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(path))

    def closeEvent(self, event):
        if self.busy:
            self.message("Wait for the live update to finish before closing.")
            event.ignore()
            return
        self.cleanup()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Setup("--no-autostart" not in sys.argv)
    window.show()
    app.aboutToQuit.connect(window.cleanup)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
