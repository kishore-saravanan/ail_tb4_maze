# Screenshots, recording and strategy illustration

These assets support the learning and experiment guide. The recording is actual visible Gazebo/RViz screen capture. The vector strategy diagram is an explicitly labeled illustration; it is not reconstructed simulator evidence.

| Asset | What it shows |
| --- | --- |
| [`demo-preview.gif`](demo-preview.gif) | Approximately eight-second excerpt at **2× screen playback**, cropped to the two viewports, labeled and palette-compressed for GitHub's README |
| [`gazebo-rviz-demo.mp4`](gazebo-rviz-demo.mp4) | Approximately 18.5-second excerpt at **1× screen playback**, H.264, no audio, with viewport labels and playback-speed disclosure |
| [`gazebo-rviz.png`](gazebo-rviz.png) | Still extracted from that recording, with the same labels |
| [`setup.png`](setup.png) | Actual setup GUI, captured as its full scrollable panel; the controller path is entered relative to the source checkout |
| [`strategy.svg`](strategy.svg) | Editable vector explanation of the map/choose/plan/drive loop and early retirement of a fully observed target |
| [`demo-capture.json`](demo-capture.json) | Trial parameters, independent referee result, source hashes and edit notes |

## Recorded trial

- Date: **2026-10-22**.
- Random preset, **12 m**, **6×6 cells**, **seed 7**.
- Backend: `direct`; requested simulation real-time factor: **1**.
- Robot speed cap: **0.30 m/s**; simulated-time budget: **300 s**.
- Referee result: **escaped**, **126.381 simulated seconds**, **26.774 m**, **0 contact episodes**.
- The screen capture begins near **107.63 simulated seconds** and shows a late-run excerpt. It is not an uncut recording of the entire attempt, nor evidence of a particular replanning event by itself.

The diagram explains the policy; the actual recording shows the robot, live sensor-built map and path. No simulator geometry was supplied to the navigation algorithm. The left Gazebo pane is an operator's world view, not an input to the solver.

The raw capture and logs remain local in `.runtime/`. Edits crop application chrome, file paths, desktop content and the gap between windows; stack the two viewport images; trim the excerpt; resize; and add captions. No trajectories, walls, sensor returns or robot motion were synthesized or removed within those viewports. There are no cuts inside the published MP4 excerpt.

The GIF has reduced frame rate and color depth and runs twice as fast as the screen recording. Playback speed, requested simulator real-time factor and robot velocity are separate quantities. Simulator real-time factor is a target, not a guarantee of exact wall-time synchronization under recording load.

GitHub can display the GIF directly in Markdown. The MP4 is provided as a repository file link for playback/download; the README does not rely on HTML video embedding or an external video host.
